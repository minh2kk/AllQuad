// AllQuad - STM32 system layer (STM32Cube HAL, no Arduino): main(), clocks, 1 MHz time base,
// HAL tick, ROM bootloader entry, heap (FreeRTOS heap_4 behind malloc / new), fault and RTOS hooks.
#if defined(AQ_STM32) && !defined(ALLQUAD_HOST)
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "FreeRTOS.h"
#include "task.h"
#include "board_stm32.h"
#include "stm32_hal.h"
#include "../../app/app.h"
#include "../hal.h"

// ============================================================ clocks
namespace {
uint32_t hseHz = 0;      // 0: running from the internal oscillator
bool usTimerUp = false;  // 1 MHz time base running (HAL_GetTick switches to it)
volatile uint32_t usHigh = 0;
}  // namespace

#if !defined(STM32H7xx) && !defined(AQ_QEMU)
// Crystal frequency: TIM11 captures HSE/16 (HSE_RTC remapped to TIM11 channel 1) every 8th edge while
// the CPU still runs from the 16 MHz HSI. Boards differ (8 MHz on most flight controllers, 25 MHz on the
// WeAct Black Pill v2-v3.0, 8 MHz on some v3.1): measuring avoids a USB clock off by 3x.
static uint32_t measureHse() {
  RCC->CR |= RCC_CR_HSEON;
  for (uint32_t i = 0; !(RCC->CR & RCC_CR_HSERDY); i++)
    if (i > 2000000) { RCC->CR &= ~RCC_CR_HSEON; return 0; }  // ~0.5 s: no crystal
  uint32_t cfgr = RCC->CFGR;
  RCC->CFGR = (cfgr & ~RCC_CFGR_RTCPRE) | (16u << RCC_CFGR_RTCPRE_Pos);  // HSE / 16
  __HAL_RCC_TIM11_CLK_ENABLE();
  TIM11->CR1 = 0;
  TIM11->OR = 2;  // TI1_RMP = HSE_RTC
  TIM11->PSC = 0;
  TIM11->ARR = 0xFFFF;
  TIM11->CCMR1 = (1u << 0) | (3u << 2);  // CC1 = input TI1, capture every 8th edge
  TIM11->CCER = TIM_CCER_CC1E;
  TIM11->EGR = TIM_EGR_UG;
  TIM11->SR = 0;
  TIM11->CR1 = TIM_CR1_CEN;
  uint32_t first = 0, last = 0;
  bool ok = true;
  for (int n = 0; n <= 16 && ok; n++) {
    uint32_t guard = 0;
    while (!(TIM11->SR & TIM_SR_CC1IF))
      if (++guard > 100000) { ok = false; break; }
    uint32_t c = TIM11->CCR1;
    if (n == 0) first = c;
    last = c;
  }
  TIM11->CR1 = 0;
  TIM11->OR = 0;
  __HAL_RCC_TIM11_CLK_DISABLE();
  RCC->CFGR = cfgr;
  if (!ok) return 0;
  uint32_t ticks = (last - first) & 0xFFFF;  // 16 captures x 128 HSE cycles at 16 MHz
  if (!ticks) return 0;
  uint32_t mhz = (uint32_t)((16ull * 128u * 16u + ticks / 2) / ticks);
  return (mhz >= 4 && mhz <= 26) ? mhz * 1000000u : 0;
}
#endif

static void clockError() {
  // the clock tree could not be set up: keep running from the reset oscillator (slow but alive)
  SystemCoreClockUpdate();
}

static void clockInit() {
#if defined(AQ_QEMU)
  // QEMU does not model the RCC: the olimex-stm32-h405 machine runs the core (and SysTick) at 168 MHz
  SystemCoreClock = 168000000u;
#elif defined(STM32H7xx)
  // 25 MHz crystal -> PLL1 400 MHz (VOS1, valid for every silicon revision), AHB 200, APBs 100 MHz.
  // PLL1Q 100 MHz: SPI1-3 and SDMMC kernel clock. PLL3Q 48 MHz: USB (exact, no CRS needed).
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  RCC_OscInitTypeDef osc = {};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = AQ_HSE_HZ / 5000000u;  // 5 MHz into the PLL
  osc.PLL.PLLN = 160;                   // VCO 800 MHz
  osc.PLL.PLLP = 2;                     // 400 MHz
  osc.PLL.PLLQ = 8;                     // 100 MHz
  osc.PLL.PLLR = 2;
  osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;  // 4-8 MHz input
  osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  osc.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) return clockError();
  hseHz = AQ_HSE_HZ;
  RCC_ClkInitTypeDef clk = {};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
  clk.AHBCLKDivider = RCC_HCLK_DIV2;
  clk.APB3CLKDivider = RCC_APB3_DIV2;
  clk.APB1CLKDivider = RCC_APB1_DIV2;
  clk.APB2CLKDivider = RCC_APB2_DIV2;
  clk.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) return clockError();
  RCC_PeriphCLKInitTypeDef pc = {};
  pc.PeriphClockSelection = RCC_PERIPHCLK_USB;
  pc.PLL3.PLL3M = AQ_HSE_HZ / 1000000u;  // 1 MHz
  pc.PLL3.PLL3N = 240;                   // 240 MHz
  pc.PLL3.PLL3P = 2;
  pc.PLL3.PLL3Q = 5;                     // 48 MHz
  pc.PLL3.PLL3R = 2;
  pc.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  pc.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  pc.PLL3.PLL3FRACN = 0;
  pc.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  HAL_RCCEx_PeriphCLKConfig(&pc);
  HAL_PWREx_EnableUSBVoltageDetector();
#else
  // F4 / F7: crystal measured (or HSI 16 MHz without one), VCO input 1-2 MHz, USB at 48 MHz
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  uint32_t src = measureHse();
  RCC_OscInitTypeDef osc = {};
  osc.PLL.PLLState = RCC_PLL_ON;
  if (src) {
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  } else {
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  }
  uint32_t in = src ? src : 16000000u;
  bool two = (in % 2000000u) == 0;  // 2 MHz into the VCO when the crystal allows it
  osc.PLL.PLLM = in / (two ? 2000000u : 1000000u);
#if defined(STM32F722xx)
  osc.PLL.PLLN = two ? 216 : 432;  // VCO 432 MHz
  osc.PLL.PLLP = RCC_PLLP_DIV2;    // 216 MHz
  osc.PLL.PLLQ = 9;                // 48 MHz
#if defined(RCC_PLLCFGR_PLLR)
  osc.PLL.PLLR = 2;
#endif
  const uint32_t latency = FLASH_LATENCY_7;
#elif defined(STM32F405xx)
  osc.PLL.PLLN = two ? 168 : 336;  // VCO 336 MHz
  osc.PLL.PLLP = RCC_PLLP_DIV2;    // 168 MHz
  osc.PLL.PLLQ = 7;                // 48 MHz
  const uint32_t latency = FLASH_LATENCY_5;
#else                              // F411
  osc.PLL.PLLN = two ? 96 : 192;   // VCO 192 MHz
  osc.PLL.PLLP = RCC_PLLP_DIV2;    // 96 MHz
  osc.PLL.PLLQ = 4;                // 48 MHz
  const uint32_t latency = FLASH_LATENCY_3;
#endif
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) return clockError();
  hseHz = src;
#if defined(STM32F722xx)
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) return clockError();
#endif
  RCC_ClkInitTypeDef clk = {};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
#if defined(STM32F411xE)
  clk.APB1CLKDivider = RCC_HCLK_DIV2;  // 48 MHz
  clk.APB2CLKDivider = RCC_HCLK_DIV1;  // 96 MHz
#else
  clk.APB1CLKDivider = RCC_HCLK_DIV4;  // 42 / 54 MHz
  clk.APB2CLKDivider = RCC_HCLK_DIV2;  // 84 / 108 MHz
#endif
  if (HAL_RCC_ClockConfig(&clk, latency) != HAL_OK) return clockError();
  // HAL computed SystemCoreClock from the compile-time HSE_VALUE: set the real one
  SystemCoreClock = (in / osc.PLL.PLLM) * osc.PLL.PLLN / 2;
#endif
}

// ============================================================ 1 MHz time base (32-bit timer + overflow count)
static uint32_t apbTimerClock(TIM_TypeDef* t) {
  bool apb2 = t == TIM1
#if defined(TIM8)
              || t == TIM8
#endif
#if defined(TIM9)
              || t == TIM9 || t == TIM10 || t == TIM11
#endif
      ;
  // timers run at twice the APB clock when its prescaler divides (field code 1xx)
#if defined(STM32H7xx)
  uint32_t pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  bool div = apb2 ? (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2_2) : (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1_2);
#else
  uint32_t pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  bool div = apb2 ? (RCC->CFGR & RCC_CFGR_PPRE2_2) : (RCC->CFGR & RCC_CFGR_PPRE1_2);
#endif
  return div ? pclk * 2 : pclk;
}

namespace hal {
namespace stm32 {
uint32_t timerClock(TIM_TypeDef* t) { return apbTimerClock(t); }
void timerClockEnable(TIM_TypeDef* t) {
  if (t == TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
  else if (t == TIM2) __HAL_RCC_TIM2_CLK_ENABLE();
  else if (t == TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
  else if (t == TIM4) __HAL_RCC_TIM4_CLK_ENABLE();
  else if (t == TIM5) __HAL_RCC_TIM5_CLK_ENABLE();
#if defined(TIM6)
  else if (t == TIM6) __HAL_RCC_TIM6_CLK_ENABLE();
#endif
#if defined(TIM7)
  else if (t == TIM7) __HAL_RCC_TIM7_CLK_ENABLE();
#endif
#if defined(TIM8)
  else if (t == TIM8) __HAL_RCC_TIM8_CLK_ENABLE();
#endif
}

void usTimerStart() {
  if (usTimerUp) return;
  TIM_TypeDef* t = AQ_US_TIM;
  timerClockEnable(t);
  t->CR1 = 0;
  t->PSC = timerClock(t) / 1000000u - 1;
  t->ARR = 0xFFFFFFFFu;
  t->CNT = 0;
  t->EGR = TIM_EGR_UG;  // load PSC
  t->SR = 0;
  t->DIER = TIM_DIER_UIE;
  HAL_NVIC_SetPriority(AQ_US_TIM_IRQn, AQ_PRIO_US_TIM, 0);
  HAL_NVIC_EnableIRQ(AQ_US_TIM_IRQn);
  t->CR1 = TIM_CR1_CEN;
  usTimerUp = true;
}

uint64_t timeUs64() {
  TIM_TypeDef* t = AQ_US_TIM;
  for (;;) {
    uint32_t h = usHigh;
    uint32_t c = t->CNT;
    uint32_t sr = t->SR;
    if (h != usHigh) continue;
    // overflow happened but its interrupt has not run yet (we are in a more urgent context)
    if ((sr & TIM_SR_UIF) && c < 0x80000000u) h++;
    return ((uint64_t)h << 32) | c;
  }
}
uint32_t hseFrequency() { return hseHz; }
}  // namespace stm32
}  // namespace hal

extern "C" void AQ_US_TIM_IRQHandler(void) {
  if (AQ_US_TIM->SR & TIM_SR_UIF) {
    AQ_US_TIM->SR = ~TIM_SR_UIF;
    usHigh = usHigh + 1;
  }
}

// HAL time: SysTick belongs to FreeRTOS. Until the 1 MHz timer runs, milliseconds come from the cycle
// counter (accumulated step by step so a clock switch never makes the count jump backwards).
extern "C" HAL_StatusTypeDef HAL_InitTick(uint32_t) { return HAL_OK; }
extern "C" uint32_t HAL_GetTick(void) {
  static uint32_t lastCyc = 0, accCyc = 0, ms = 0, offset = 0;
  static bool switched = false;
  if (usTimerUp) {
    uint32_t t = (uint32_t)(hal::stm32::timeUs64() / 1000u);
    if (!switched) { switched = true; offset = ms - t; }
    return t + offset;
  }
  uint32_t now = DWT->CYCCNT;
  accCyc += now - lastCyc;
  lastCyc = now;
  uint32_t per = SystemCoreClock / 1000u;
  if (!per) per = 16000;
  while (accCyc >= per) { accCyc -= per; ms++; }
  return ms;
}
extern "C" void HAL_Delay(uint32_t ms) {
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < ms) {}
}
extern "C" void HAL_IncTick(void) {}

// ============================================================ ROM bootloader (USB DFU)
// A reboot request is kept in an RTC backup register (survives a reset) and acted on first thing at
// start: clocks at reset state, system memory mapped, jump into the ROM.
static constexpr uint32_t BOOTLOADER_MAGIC = 0xB007B007u;
static void backupAccess() {
#if defined(STM32H7xx)
  __HAL_RCC_RTC_CLK_ENABLE();
#else
  __HAL_RCC_PWR_CLK_ENABLE();
#endif
  HAL_PWR_EnableBkUpAccess();
}
namespace hal {
namespace stm32 {
void requestBootloader() {
  backupAccess();
  RTC->BKP19R = BOOTLOADER_MAGIC;
}
}  // namespace stm32
}  // namespace hal
static void checkBootloaderRequest() {
#if defined(STM32H7xx)
  __HAL_RCC_RTC_CLK_ENABLE();
#endif
  if (RTC->BKP19R != BOOTLOADER_MAGIC) return;
  backupAccess();
  RTC->BKP19R = 0;
#if defined(STM32H7xx)
  const uint32_t sys = 0x1FF09800u;
#elif defined(STM32F7xx)
  const uint32_t sys = 0x1FF00000u;
#else
  const uint32_t sys = 0x1FFF0000u;
#endif
  __disable_irq();
  for (int i = 0; i < 8; i++) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
#if !defined(STM32F7xx) && !defined(STM32H7xx)
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
#endif
  __enable_irq();
  __set_MSP(*(volatile uint32_t*)sys);
  ((void (*)(void))(*(volatile uint32_t*)(sys + 4)))();
  for (;;) {}
}

// ============================================================ entry
#if defined(STM32H7xx)
#define IWDG_INST IWDG1
#else
#define IWDG_INST IWDG
#endif
// ============================================================ crash record + watchdog
// .noinit RAM is neither zeroed nor loaded by the startup code: what a crash writes there is still
// readable after the reset that follows (not after a power cycle). The record carries a magic and a
// checksum; on the H7 it is written back from the data cache before the reset.
extern "C" uint32_t _sdata, _estack;
namespace {
constexpr uint32_t CRASH_MAGIC = 0xC7A5B0D1u, CRUMB_MAGIC = 0xB4EADC4Bu;
struct alignas(32) CrashRec {
  uint32_t magic;
  hal::CrashInfo info;
  uint32_t check;
};
struct alignas(32) Crumb {  // one cache line
  uint32_t magic, uptimeMs;
  char task[24];
};
__attribute__((section(".noinit"))) CrashRec crashRec;
__attribute__((section(".noinit"))) Crumb crumb;
hal::CrashInfo bootCrash;
bool bootCrashValid = false;
int bootReason = 0;
bool wdtOn = false;

uint32_t crashSum(const hal::CrashInfo& i) {
  const uint8_t* b = (const uint8_t*)&i;
  uint32_t s = 0x5A17C0DEu;
  for (size_t k = 0; k < sizeof(i); k++) s = ((s << 5) | (s >> 27)) ^ b[k];
  return s;
}
bool crashValid() { return crashRec.magic == CRASH_MAGIC && crashRec.check == crashSum(crashRec.info); }
void cacheWriteBack(const void* p, size_t n) {
#if defined(STM32H7xx)
  if (SCB->CCR & SCB_CCR_DC_Msk) SCB_CleanDCache_by_Addr((uint32_t*)p, (int32_t)((n + 31) & ~31u));
#else
  (void)p; (void)n;
#endif
}
bool inRam(const void* p, size_t n) {
  uintptr_t a = (uintptr_t)p;
  return a >= (uintptr_t)&_sdata && a + n <= (uintptr_t)&_estack && a + n > a;
}
// printable ASCII only (the name may come from a corrupted task control block; it ends up in JSON)
void copyName(char* dst, size_t cap, const char* src) {
  size_t n = 0;
  bool ram = src && inRam(src, 1);
  while (src && n < cap - 1 && (!ram || (uintptr_t)(src + n) < (uintptr_t)&_estack) && src[n]) {
    char c = src[n];
    dst[n++] = (c >= 0x20 && c < 0x7F && c != '"' && c != '\\') ? c : '?';
  }
  dst[n] = 0;
}
const char* currentTaskName() {
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return "boot";
  TaskHandle_t t = xTaskGetCurrentTaskHandle();
  if (!t || !inRam(t, 64)) return nullptr;
  return pcTaskGetName(t);
}
}  // namespace

// called with interrupts off from the fault handlers / RTOS hooks: no allocation, no locks
static void crashRecord(uint8_t kind, const uint32_t* frame, const char* where) {
  uint16_t count = crashValid() ? crashRec.info.count : 0;
  hal::CrashInfo& i = crashRec.info;
  memset(&i, 0, sizeof(i));
  i.kind = kind;
  i.count = count < 0xFFFF ? count + 1 : count;
  i.bootsAgo = 0;
  i.uptimeMs = HAL_GetTick();
  if (frame && inRam(frame, 32)) {  // r0-r3, r12, lr, pc, xpsr
    i.lr = frame[5];
    i.pc = frame[6];
  }
  if (kind >= hal::CRASH_HARDFAULT && kind <= hal::CRASH_USAGEFAULT) {
    i.cfsr = SCB->CFSR;
    i.hfsr = SCB->HFSR;
    if (i.cfsr & SCB_CFSR_MMARVALID_Msk) i.addr = SCB->MMFAR;
    else if (i.cfsr & SCB_CFSR_BFARVALID_Msk) i.addr = SCB->BFAR;
  }
  copyName(i.where, sizeof(i.where), where ? where : currentTaskName());
  crashRec.check = crashSum(i);
  crashRec.magic = CRASH_MAGIC;
  cacheWriteBack(&crashRec, sizeof(crashRec));
}
static void crumbUpdate() {
  crumb.uptimeMs = HAL_GetTick();
  copyName(crumb.task, sizeof(crumb.task), currentTaskName());
  crumb.magic = CRUMB_MAGIC;
  cacheWriteBack(&crumb, sizeof(crumb));
}

// reset flags (read once, then cleared) and the record the previous run left
static void crashBoot() {
#if defined(STM32H7xx)
  uint32_t f = RCC->RSR;
  bool bor = f & RCC_RSR_BORRSTF, por = f & RCC_RSR_PORRSTF, wdg = f & (RCC_RSR_IWDG1RSTF | RCC_RSR_WWDG1RSTF),
       sft = f & RCC_RSR_SFTRSTF, pin = f & RCC_RSR_PINRSTF;
  RCC->RSR |= RCC_RSR_RMVF;
#else
  uint32_t f = RCC->CSR;
  bool bor = f & RCC_CSR_BORRSTF, por = f & RCC_CSR_PORRSTF, wdg = f & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF),
       sft = f & RCC_CSR_SFTRSTF, pin = f & RCC_CSR_PINRSTF;
  RCC->CSR |= RCC_CSR_RMVF;
#endif
  // a power-on also sets BOR on most parts: power-on wins, brownout means the supply dipped while running
  bootReason = por ? 1 : bor ? 9 : wdg ? 7 : sft ? 3 : pin ? 2 : 0;
  if (por || bor) {  // RAM content is undefined after the supply went away
    crashRec.magic = 0;
    crumb.magic = 0;
  }
  if (wdg) {  // nothing ran at the reset itself: the breadcrumb says what had the CPU
    uint16_t count = crashValid() ? crashRec.info.count : 0;
    hal::CrashInfo& i = crashRec.info;
    memset(&i, 0, sizeof(i));
    i.kind = hal::CRASH_WATCHDOG;
    i.count = count < 0xFFFF ? count + 1 : count;
    if (crumb.magic == CRUMB_MAGIC) {
      i.uptimeMs = crumb.uptimeMs;
      memcpy(i.where, crumb.task, sizeof(i.where));
      i.where[sizeof(i.where) - 1] = 0;
    }
    crashRec.check = crashSum(i);
    crashRec.magic = CRASH_MAGIC;
  }
  crumb.magic = 0;
  if (crashValid()) {
    bootCrash = crashRec.info;
    bootCrashValid = true;
    if (bootCrash.bootsAgo == 0 && sft && bootCrash.kind != hal::CRASH_WATCHDOG) bootReason = 4;  // panic reset
    if (crashRec.info.bootsAgo < 0xFFFF) crashRec.info.bootsAgo++;  // older at the next boot
    crashRec.check = crashSum(crashRec.info);
  }
  // (data cache not enabled yet: these writes go straight to RAM)
}

namespace hal {
namespace stm32 {
int bootResetReason() { return bootReason; }
void watchdogFlashErase() { if (wdtOn) IWDG_INST->KR = 0xAAAAu; }
}  // namespace stm32
bool lastCrash(CrashInfo& out) {
  if (!bootCrashValid) return false;
  out = bootCrash;
  return true;
}
void crashClear() {
  bootCrashValid = false;
  crashRec.magic = 0;
  cacheWriteBack(&crashRec, sizeof(crashRec));
}
// IWDG from the ~32 kHz LSI: /64, reload 3400 = 6.8 s nominal; >= 4.6 s even with the fastest LSI the
// F4 datasheet allows (47 kHz), so a 128 kB sector erase (up to 4 s, kicked right before) never trips it.
void watchdogStart() {
#if !defined(AQ_QEMU)
  if (wdtOn) return;
#if defined(STM32H7xx)
  DBGMCU->APB4FZ1 |= DBGMCU_APB4FZ1_DBG_IWDG1;  // stops while a debugger halts the core
#else
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#endif
  IWDG_INST->KR = 0xCCCCu;  // start (LSI switches on by itself)
  IWDG_INST->KR = 0x5555u;  // unlock PR / RLR
  IWDG_INST->PR = 4;        // /64
  IWDG_INST->RLR = 3400;
  for (uint32_t g = 0; (IWDG_INST->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) && g < 2000000u; g++) {}
  IWDG_INST->KR = 0xAAAAu;
  wdtOn = true;
#endif
}
void watchdogKick() { if (wdtOn) IWDG_INST->KR = 0xAAAAu; }
bool watchdogRunning() { return wdtOn; }
}  // namespace hal

int main(void) {
  crashBoot();  // reset flags + the record a crash left in RAM, before anything else runs
#if defined(AQ_QEMU)
  // no DWT, no RCC in QEMU: the time base runs from the start
  clockInit();
  hal::stm32::usTimerStart();
  HAL_Init();
  app::firmwareSetup();
  for (;;) {}
#endif
  checkBootloaderRequest();
  // cycle counter: profiler (perf) and the HAL tick before the time base runs
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(__CORTEX_M) && (__CORTEX_M == 7U)
  DWT->LAR = 0xC5ACCE55u;
#endif
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#if defined(STM32H7xx)
  // both caches on; the DMA buffers (DShot, LED strip) are cleaned / invalidated around each transfer
  SCB_EnableICache();
  SCB_EnableDCache();
#elif defined(STM32F7xx)
  SCB_EnableICache();  // data cache off: the DMA buffers stay coherent without maintenance
#endif
  HAL_Init();
  clockInit();
  app::firmwareSetup();  // starts the scheduler: never returns
  for (;;) {}
}

// ============================================================ heap: malloc / new -> FreeRTOS heap_4
extern "C" {
void* malloc(size_t n) { return pvPortMalloc(n); }
void free(void* p) { vPortFree(p); }
void* calloc(size_t n, size_t m) {
  void* p = pvPortMalloc(n * m);
  if (p) memset(p, 0, n * m);
  return p;
}
void* realloc(void* p, size_t n) {
  // heap_4 keeps the block size in front of the block (BlockLink_t: next pointer, size with the top bit
  // marking "allocated"); copy what fits
  if (!p) return pvPortMalloc(n);
  if (!n) { vPortFree(p); return nullptr; }
  size_t old = (((size_t*)p)[-1] & ~((size_t)1 << (sizeof(size_t) * 8 - 1))) - 2 * sizeof(size_t);
  void* q = pvPortMalloc(n);
  if (q) { memcpy(q, p, old < n ? old : n); vPortFree(p); }
  return q;
}
void* _malloc_r(struct _reent*, size_t n) { return malloc(n); }
void _free_r(struct _reent*, void* p) { free(p); }
void* _calloc_r(struct _reent*, size_t n, size_t m) { return calloc(n, m); }
void* _realloc_r(struct _reent*, void* p, size_t n) { return realloc(p, n); }

// newlib stubs (no file system behind stdio; text goes through hal::portWrite)
int _write(int, const char*, int len) { return len; }
int _read(int, char*, int) { return 0; }
int _close(int) { return -1; }
int _lseek(int, int, int) { return 0; }
int _fstat(int, struct stat* st) { st->st_mode = S_IFCHR; return 0; }
int _isatty(int) { return 1; }
int _getpid(void) { return 1; }
int _kill(int, int) { errno = EINVAL; return -1; }
void _exit(int) { NVIC_SystemReset(); for (;;) {} }
void* _sbrk(ptrdiff_t) { errno = ENOMEM; return (void*)-1; }  // everything goes through heap_4

// ============================================================ faults & RTOS hooks
// A crash writes a record into RAM that survives the reset (crashRecord) and restarts the controller
// (USB and the configurator come back). Motors stop with the reset: the outputs return to idle.
// The fault handlers find the stacked frame (MSP or PSP) for the faulting PC / LR.
__attribute__((noreturn)) static void panic(uint8_t kind, const uint32_t* frame, const char* where) {
  __disable_irq();
  crashRecord(kind, frame, where);
  NVIC_SystemReset();
  for (;;) {}
}
void aqAssertFailed(const char* file, int line) {
  char w[24];
  const char* b = file ? file : "?";
  for (const char* p = b; *p; p++) if (*p == '/' || *p == '\\') b = p + 1;
  size_t n = 0;
  while (b[n] && n < 15) { w[n] = b[n]; n++; }
  w[n++] = ':';
  char d[8];
  int k = 0;
  unsigned v = line > 0 ? (unsigned)line : 0;
  do { d[k++] = (char)('0' + v % 10); v /= 10; } while (v && k < 7);
  while (k && n < sizeof(w) - 1) w[n++] = d[--k];
  w[n] = 0;
  panic(hal::CRASH_ASSERT, nullptr, w);
}
void vApplicationStackOverflowHook(TaskHandle_t, char* name) { panic(hal::CRASH_STACK, nullptr, name); }
void vApplicationMallocFailedHook(void) { panic(hal::CRASH_NOMEM, nullptr, nullptr); }
__attribute__((used)) void faultEntry(const uint32_t* frame, uint32_t kind) { panic((uint8_t)kind, frame, nullptr); }
#define AQ_FAULT_HANDLER(name, kind)                                              \
  __attribute__((naked)) void name(void) {                                        \
    __asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n"        \
                   "movs r1, %0\n b faultEntry\n" ::"i"(kind));                  \
  }
AQ_FAULT_HANDLER(HardFault_Handler, hal::CRASH_HARDFAULT)
AQ_FAULT_HANDLER(MemManage_Handler, hal::CRASH_MEMMANAGE)
AQ_FAULT_HANDLER(BusFault_Handler, hal::CRASH_BUSFAULT)
AQ_FAULT_HANDLER(UsageFault_Handler, hal::CRASH_USAGEFAULT)
// every 64 ms: which task runs and since when (tells what hogged the CPU after a watchdog reset)
void vApplicationTickHook(void) {
  static uint8_t n = 0;
  if (++n & 63) return;
  crumbUpdate();
}
void NMI_Handler(void) {}
void DebugMon_Handler(void) {}
}  // extern "C"

void* operator new(size_t n) { return pvPortMalloc(n); }
void* operator new[](size_t n) { return pvPortMalloc(n); }
void operator delete(void* p) noexcept { vPortFree(p); }
void operator delete[](void* p) noexcept { vPortFree(p); }
void operator delete(void* p, size_t) noexcept { vPortFree(p); }
void operator delete[](void* p, size_t) noexcept { vPortFree(p); }
#endif
