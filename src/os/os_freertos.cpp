// AllQuad - FreeRTOS backend
//   ESP32 / ESP32-S3 : FreeRTOS built into Arduino-ESP32, tasks can be pinned to a core.
//   STM32            : FreeRTOS kernel (lib/freertos, ARM_CM4F port) on the STM32Cube HAL
//
// Periodic tasks use vTaskDelayUntil on the 1 kHz tick. The hardware-timed task
// (the gyro/PID loop) is woken by a µs-accurate timer through a task notification,
// which allows 2 kHz loops and removes tick jitter.
#if (defined(ARDUINO) || defined(AQ_STM32)) && !defined(ALLQUAD_HOST)
#if defined(ARDUINO)
#include <Arduino.h>
#endif

#if defined(ESP32)
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#elif defined(AQ_STM32)
#include "FreeRTOS.h"
#include "task.h"
#include "../hal/board.h"
#include "../hal/hal.h"
#include "../hal/stm32/board_stm32.h"
#include "../hal/stm32/stm32_hal.h"
#endif

#include <string.h>

#include "os.h"

namespace os {

#if defined(AQ_STM32)
static inline uint32_t micros() { return hal::micros(); }
#endif

namespace {
struct RtosTask {
  TaskConfig cfg;
  TaskHandle_t handle;
  TaskStats stats;
};
RtosTask tasks[MAX_TASKS];
int nTasks = 0;
TaskHandle_t timedTask = nullptr;

const char* trigSrc = "timer";

// ---- IMU data-ready pin: the sample is fresh when the loop starts (with the timer it is 0..1 period
// old and the two clocks beat against each other, so the age wanders). Every divider-th pulse wakes the
// fast task; the pulse counter also tells a stalled CPU (pulses kept coming) from a dead pin. ----
int drdyPin = -1;
uint32_t drdyHz = 0;
uint8_t drdyDiv = 1;
volatile uint32_t drdyPulses = 0;
volatile uint8_t drdyCount = 0;
volatile bool drdyActive = false;

#if defined(ESP32)
#define AQ_ISR IRAM_ATTR
#else
#define AQ_ISR
#endif

AQ_ISR void notifyFromIsr() {
  BaseType_t woken = pdFALSE;
  if (timedTask) vTaskNotifyGiveFromISR(timedTask, &woken);
#if defined(ESP32)
  if (woken == pdTRUE) portYIELD_FROM_ISR();
#else
  portYIELD_FROM_ISR(woken);
#endif
}
AQ_ISR void drdyPulse() {
  drdyPulses = drdyPulses + 1;
  if (!drdyActive) return;
  uint8_t n = drdyCount + 1;
  if (n < drdyDiv) { drdyCount = n; return; }
  drdyCount = 0;
  notifyFromIsr();
}

#if defined(ESP32)
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
esp_timer_handle_t hwTimer = nullptr;
// The loop timer is a hardware GPTimer whose interrupt is allocated on the core that creates it:
// it is started from inside the fast task, so the interrupt lands on core 1 next to the loop and
// away from WiFi (setup() and the driver interrupts run on core 0). The notification goes straight
// from that ISR to the task: no hop through the esp_timer task on core 0, which adds 10-50 µs of
// jitter whenever WiFi is busy. esp_timer stays as the fallback.
gptimer_handle_t gpTimer = nullptr;
bool IRAM_ATTR gpAlarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*) {
  BaseType_t woken = pdFALSE;
  if (timedTask) vTaskNotifyGiveFromISR(timedTask, &woken);
  return woken == pdTRUE;
}
void timerCb(void*) {
  if (timedTask) xTaskNotifyGive(timedTask);
}
bool startGpTimer(uint32_t periodUs) {
  gptimer_config_t tc = {};
  tc.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  tc.direction = GPTIMER_COUNT_UP;
  tc.resolution_hz = 1000000;
  if (gptimer_new_timer(&tc, &gpTimer) != ESP_OK) return false;
  gptimer_event_callbacks_t cb = {};
  cb.on_alarm = gpAlarm;
  gptimer_alarm_config_t al = {};
  al.alarm_count = periodUs;
  al.reload_count = 0;
  al.flags.auto_reload_on_alarm = true;
  if (gptimer_register_event_callbacks(gpTimer, &cb, nullptr) != ESP_OK || gptimer_set_alarm_action(gpTimer, &al) != ESP_OK ||
      gptimer_enable(gpTimer) != ESP_OK || gptimer_start(gpTimer) != ESP_OK) {
    gptimer_del_timer(gpTimer);
    gpTimer = nullptr;
    return false;
  }
  return true;
}
void startTimer(uint32_t periodUs) {
  if (gpTimer || hwTimer) return;
  if (startGpTimer(periodUs)) return;
  esp_timer_create_args_t a = {};
  a.callback = timerCb;
  a.name = "fastloop";
  esp_timer_create(&a, &hwTimer);
  esp_timer_start_periodic(hwTimer, periodUs);
}
void IRAM_ATTR drdyIsr(void*) { drdyPulse(); }
// called inside the fast task (core 1), so the GPIO interrupt is serviced on that core
bool drdyAttach(int p) {
  gpio_num_t pin = (gpio_num_t)p;
  gpio_set_direction(pin, GPIO_MODE_INPUT);
  gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);  // quiet if the pin is not wired
  gpio_set_intr_type(pin, GPIO_INTR_POSEDGE);
  esp_err_t e = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return false;
  if (gpio_isr_handler_add(pin, drdyIsr, nullptr) != ESP_OK) return false;
  gpio_intr_enable(pin);
  return true;
}
void drdyDetach(int p) {
  gpio_intr_disable((gpio_num_t)p);
  gpio_isr_handler_remove((gpio_num_t)p);
}
#elif defined(AQ_STM32)
// AQ_LOOP_TIM wakes the loop (1 MHz tick, auto-reload = period); its interrupt priority (5) allows
// FreeRTOS calls from the ISR. The data-ready pin uses its EXTI line at the same priority.
bool loopTimerOn = false;
void startTimer(uint32_t periodUs) {
  if (loopTimerOn) return;
  TIM_TypeDef* t = AQ_LOOP_TIM;
  hal::stm32::timerClockEnable(t);
  t->CR1 = 0;
  t->PSC = hal::stm32::timerClock(t) / 1000000u - 1;
  t->ARR = periodUs - 1;
  t->CNT = 0;
  t->EGR = TIM_EGR_UG;
  t->SR = 0;
  t->DIER = TIM_DIER_UIE;
  HAL_NVIC_SetPriority(AQ_LOOP_TIM_IRQn, AQ_PRIO_LOOP_TIM, 0);
  NVIC_EnableIRQ(AQ_LOOP_TIM_IRQn);
  t->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
  loopTimerOn = true;
}
IRQn_Type extiIrq(int line) {
  switch (line) {
    case 0: return EXTI0_IRQn;
    case 1: return EXTI1_IRQn;
    case 2: return EXTI2_IRQn;
    case 3: return EXTI3_IRQn;
    case 4: return EXTI4_IRQn;
    default: return line < 10 ? EXTI9_5_IRQn : EXTI15_10_IRQn;
  }
}
#if defined(STM32H7xx)
#define AQ_EXTI_IMR EXTI_D1->IMR1
#define AQ_EXTI_PR EXTI_D1->PR1
#define AQ_EXTI_RTSR EXTI->RTSR1
#define AQ_EXTI_FTSR EXTI->FTSR1
#else
#define AQ_EXTI_IMR EXTI->IMR
#define AQ_EXTI_PR EXTI->PR
#define AQ_EXTI_RTSR EXTI->RTSR
#define AQ_EXTI_FTSR EXTI->FTSR
#endif
bool drdyAttach(int pin) {
  if (pin < 0) return false;
  int line = pin & 15;
  gpio::config(pin, gpio::IN, gpio::DOWN);  // quiet if the pin is not wired
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  uint32_t sh = (uint32_t)(line & 3) * 4;
  SYSCFG->EXTICR[line >> 2] = (SYSCFG->EXTICR[line >> 2] & ~(0xFu << sh)) | ((uint32_t)(pin >> 4) << sh);
  AQ_EXTI_RTSR |= 1u << line;
  AQ_EXTI_FTSR &= ~(1u << line);
  AQ_EXTI_PR = 1u << line;
  AQ_EXTI_IMR |= 1u << line;
  HAL_NVIC_SetPriority(extiIrq(line), AQ_PRIO_EXTI, 0);
  NVIC_EnableIRQ(extiIrq(line));
  return true;
}
void drdyDetach(int pin) {
  if (pin < 0) return;
  int line = pin & 15;
  AQ_EXTI_IMR &= ~(1u << line);
  AQ_EXTI_PR = 1u << line;
}
#endif

void stopDataReady() {
  drdyActive = false;
  drdyDetach(drdyPin);
}
// Runs inside the fast task. The pulse rate must be within 25 % of the expected one.
bool startDataReady() {
  if (drdyPin < 0 || drdyHz == 0) return false;
  if (!drdyAttach(drdyPin)) { trigSrc = "timer (imu-int: no isr)"; return false; }
  uint32_t n0 = drdyPulses, t0 = micros();
  vTaskDelay(pdMS_TO_TICKS(30));
  uint32_t n = drdyPulses - n0, dt = micros() - t0;
  float hz = dt ? n * 1e6f / dt : 0;
  if (hz < drdyHz * 0.75f || hz > drdyHz * 1.25f) {
    stopDataReady();
    trigSrc = n ? "timer (imu-int: wrong rate)" : "timer (imu-int: no pulses)";
    return false;
  }
  drdyCount = 0;
  drdyActive = true;
  trigSrc = "imu-int";
  return true;
}

void measure(RtosTask& t, uint32_t startUs, uint32_t expectedUs) {
  uint32_t used = (uint32_t)micros() - startUs;
  t.stats.lastUs = used;
  if (used > t.stats.maxUs) t.stats.maxUs = used;
  if ((int32_t)(startUs - expectedUs) > (int32_t)t.cfg.periodUs) t.stats.overruns++;
  t.stats.runs++;
  if ((t.stats.runs & 1023) == 1) {  // occasionally: the high-water mark walks the stack
#if defined(ESP32)
    t.stats.stackFree = uxTaskGetStackHighWaterMark(nullptr);  // bytes on ESP32
#else
    t.stats.stackFree = uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
#endif
  }
}

void taskEntry(void* p) {
  RtosTask& t = *static_cast<RtosTask*>(p);
  if (t.cfg.hardwareTimed) {
    timedTask = xTaskGetCurrentTaskHandle();
    if (!startDataReady()) startTimer(t.cfg.periodUs);
    // a lost data-ready signal must not stop the motors loop: ~4 periods without a pulse -> timer
    TickType_t drdyWait = pdMS_TO_TICKS(4 * t.cfg.periodUs / 1000 + 2);
    if (drdyWait < 3) drdyWait = 3;
    uint32_t expected = micros();
    uint32_t lastStart = expected;
    for (;;) {
      if (drdyActive) {
        uint32_t pulsesBefore = drdyPulses;
        if (ulTaskNotifyTake(pdTRUE, drdyWait) == 0) {
          // pulses still coming (e.g. both cores were held by a flash write): keep waiting for them
          if (drdyPulses != pulsesBefore) continue;
          stopDataReady();
          trigSrc = "timer (imu-int lost)";
          startTimer(t.cfg.periodUs);
          continue;
        }
      } else {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      }
      uint32_t start = micros();
      t.cfg.run(t.cfg.arg);
      // the IMU clock is not the CPU clock: judge each start against the previous one
      if (drdyActive) expected = lastStart + t.cfg.periodUs;
      else
        expected += t.cfg.periodUs;
      if ((int32_t)(start - expected) > (int32_t)(4 * t.cfg.periodUs)) expected = start;  // resync
      lastStart = start;
      measure(t, start, expected);
    }
  } else {
    TickType_t last = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(t.cfg.periodUs / 1000);
    if (period == 0) period = 1;
    for (;;) {
      vTaskDelayUntil(&last, period);
      uint32_t start = micros();
      t.cfg.run(t.cfg.arg);
      measure(t, start, start);
    }
  }
}
}  // namespace

bool createTask(const TaskConfig& cfg) {
  if (nTasks >= MAX_TASKS) return false;
  RtosTask& t = tasks[nTasks];
  t.cfg = cfg;
  memset(&t.stats, 0, sizeof(t.stats));
  t.stats.name = cfg.name;
  t.stats.periodUs = cfg.periodUs;
  BaseType_t ok;
#if defined(ESP32)
  // ESP32 stack size is in bytes
  ok = xTaskCreatePinnedToCore(taskEntry, cfg.name, cfg.stackBytes, &t, cfg.priority, &t.handle,
                               cfg.core < 0 ? tskNO_AFFINITY : (cfg.core < portNUM_PROCESSORS ? cfg.core : 0));
#else
  // vanilla FreeRTOS stack size is in words
  ok = xTaskCreate(taskEntry, cfg.name, cfg.stackBytes / sizeof(StackType_t), &t, cfg.priority, &t.handle);
#endif
  if (ok != pdPASS) return false;
  nTasks++;
  if (cfg.hardwareTimed) {
    timedTask = t.handle;
    // the timer or the data-ready interrupt is started by the task itself (taskEntry), on its core
  }
  return true;
}

void setDataReadyTrigger(int pin, uint32_t pulseHz, uint8_t divider) {
  drdyPin = pin;
  drdyHz = pulseHz;
  drdyDiv = divider ? divider : 1;
}
const char* triggerSource() { return trigSrc; }

void startScheduler() {
#if defined(AQ_STM32)
  vTaskStartScheduler();  // never returns
#endif
}

uint64_t timeUs() {
#if defined(ESP32)
  return (uint64_t)esp_timer_get_time();
#elif defined(AQ_STM32)
  return hal::stm32::timeUs64();
#else
  static uint32_t lastLow = 0;
  static uint32_t high = 0;
  CriticalGuard g;
  uint32_t now = micros();
  if (now < lastLow) high++;
  lastLow = now;
  return ((uint64_t)high << 32) | now;
#endif
}
uint32_t timeMs() { return (uint32_t)(timeUs() / 1000); }
int taskCount() { return nTasks; }
void heapStats(uint32_t& freeNow, uint32_t& lowest) {
  freeNow = (uint32_t)xPortGetFreeHeapSize();
  lowest = (uint32_t)xPortGetMinimumEverFreeHeapSize();
}
TaskStats taskStats(int i) { return tasks[i].stats; }
void resetTaskStats() {
  for (int i = 0; i < nTasks; i++) tasks[i].stats.maxUs = 0;
}

#if defined(ESP32)
static_assert(sizeof(SpinLock) == sizeof(portMUX_TYPE), "SpinLock must match portMUX_TYPE");
void lock(SpinLock& l) { portENTER_CRITICAL(reinterpret_cast<portMUX_TYPE*>(&l)); }
void unlock(SpinLock& l) { portEXIT_CRITICAL(reinterpret_cast<portMUX_TYPE*>(&l)); }
#else
// Before the scheduler runs (setup: config load, sensor probing) there is one thread and no task to
// protect against. A FreeRTOS critical section at that point is harmful on Cortex-M: the nesting counter
// starts at 0xAAAAAAAA, so taskEXIT_CRITICAL() does not lower BASEPRI again and every interrupt below
// configMAX_SYSCALL_INTERRUPT_PRIORITY (SysTick -> HAL_GetTick, delay(), I2C / UART timeouts) stays
// masked until the scheduler starts. A config migration message at boot was enough to trigger it.
static inline bool rtosRunning() { return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED; }
void lock(SpinLock&) { if (rtosRunning()) taskENTER_CRITICAL(); }
void unlock(SpinLock&) { if (rtosRunning()) taskEXIT_CRITICAL(); }
#endif

void enterCritical() {
#if defined(ESP32)
  portENTER_CRITICAL(&mux);
#else
  if (rtosRunning()) taskENTER_CRITICAL();
#endif
}
void exitCritical() {
#if defined(ESP32)
  portEXIT_CRITICAL(&mux);
#else
  if (rtosRunning()) taskEXIT_CRITICAL();
#endif
}

}  // namespace os

#if defined(AQ_STM32)
extern "C" void AQ_LOOP_TIM_IRQHandler(void) {
  if (AQ_LOOP_TIM->SR & TIM_SR_UIF) {
    AQ_LOOP_TIM->SR = ~TIM_SR_UIF;
    os::notifyFromIsr();
  }
}
static inline void extiService(uint32_t lines) {
  uint32_t pend = AQ_EXTI_PR & lines;
  if (!pend) return;
  AQ_EXTI_PR = pend;
  if (os::drdyPin >= 0 && (pend & (1u << (os::drdyPin & 15)))) os::drdyPulse();
}
extern "C" void EXTI0_IRQHandler(void) { extiService(1u << 0); }
extern "C" void EXTI1_IRQHandler(void) { extiService(1u << 1); }
extern "C" void EXTI2_IRQHandler(void) { extiService(1u << 2); }
extern "C" void EXTI3_IRQHandler(void) { extiService(1u << 3); }
extern "C" void EXTI4_IRQHandler(void) { extiService(1u << 4); }
extern "C" void EXTI9_5_IRQHandler(void) { extiService(0x03E0u); }
extern "C" void EXTI15_10_IRQHandler(void) { extiService(0xFC00u); }
#endif
#endif
