// AllQuad - STM32F4 / STM32F7 / STM32H7 HAL on STM32Cube (ST HAL drivers + FreeRTOS, no Arduino)
// Boards: F411 Black Pill, F405 / F722 flight-controller layouts, H743 WeAct / DevEBox (src/hal/board.h,
// peripheral routing in src/hal/stm32/board_stm32.h).
//  * serial ports: own interrupt drivers with 1 kB receive rings per UART (no byte lost while a task is
//    busy), overrun / framing flags cleared in the interrupt (a stuck overrun flag can never block RX)
//  * I2C: ST HAL in polling mode with short timeouts, bus recovery + re-init after any bus error
//  * SPI: register-level transfers (F4 / F7), ST HAL (H7), chip select by GPIO
//  * USB: CDC virtual COM port (src/hal/stm32/usb_cdc.c)
//  * motors: DShot300/600 (+ bidirectional) via TIM1 update -> DMA2 Stream5 -> GPIOx->BSRR bit-banging,
//    PWM / OneShot125 on the motor pins' timer
//  * storage: the last 8 kB of the last flash sector (same layout as the former STM32duino EEPROM
//    emulation: settings stored by older firmware are kept)
#if defined(AQ_STM32) && !defined(ALLQUAD_HOST)
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "../common/cycles.h"
#include "../common/math3d.h"
#include "ee_ab.h"
#include "board.h"
#include "dshot_codec.h"
#include "hal.h"
#include "stm32/board_stm32.h"
#include "stm32/stm32_hal.h"

#ifndef LED_ACTIVE_LOW
#define LED_ACTIVE_LOW 0
#endif

// register / bit names that differ between the USART generations
#if defined(STM32F4xx)
#define AQ_USART_V1 1
#else
#if defined(USART_ISR_RXNE_RXFNE)
#define AQ_ISR_RXNE USART_ISR_RXNE_RXFNE
#define AQ_ISR_TXE USART_ISR_TXE_TXFNF
#define AQ_CR1_RXNEIE USART_CR1_RXNEIE_RXFNEIE
#define AQ_CR1_TXEIE USART_CR1_TXEIE_TXFNFIE
#else
#define AQ_ISR_RXNE USART_ISR_RXNE
#define AQ_ISR_TXE USART_ISR_TXE
#define AQ_CR1_RXNEIE USART_CR1_RXNEIE
#define AQ_CR1_TXEIE USART_CR1_TXEIE
#endif
#endif
#if defined(USART_CR1_M0)
#define AQ_CR1_M9 USART_CR1_M0
#else
#define AQ_CR1_M9 USART_CR1_M
#endif

namespace hal {

static inline bool rtosRunning() { return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED; }

// ============================================================ time
uint32_t micros() { return (uint32_t)stm32::timeUs64(); }
uint32_t millis() { return (uint32_t)(stm32::timeUs64() / 1000u); }
static void delayUs(uint32_t us) {
  uint32_t t0 = fc::cycles(), n = (uint32_t)(us * fc::cyclesPerUs());
  while (fc::cycles() - t0 < n) {}
}
void delayMs(uint32_t ms) {
  if (rtosRunning() && __get_IPSR() == 0) {
    vTaskDelay(pdMS_TO_TICKS(ms) ? pdMS_TO_TICKS(ms) : 1);
  } else {
    uint64_t end = stm32::timeUs64() + (uint64_t)ms * 1000u;
    while (stm32::timeUs64() < end) {}
  }
}

static inline uint32_t irqSave() {
  uint32_t p = __get_PRIMASK();
  __disable_irq();
  return p;
}
static inline void irqRestore(uint32_t p) { __set_PRIMASK(p); }

// ============================================================ platform
const char* platformName() {
  static char name[80];
  if (!name[0]) {
    uint32_t hse = stm32::hseFrequency();
    const char* rev = "";
#if defined(STM32H7xx)
    // silicon revision (DBGMCU_IDCODE REV_ID): only rev V (and X) run 480 MHz (VOS0); rev Y stops at 400 MHz
    uint32_t r = HAL_GetREVID();
    rev = r == 0x1003 ? ", rev Y" : r == 0x2003 ? ", rev V" : r == 0x2001 ? ", rev X" : ", rev ?";
#endif
    snprintf(name, sizeof(name), "%s (%lu MHz%s, %s %lu MHz)", AQ_BOARD_NAME, (unsigned long)(SystemCoreClock / 1000000u),
             rev, hse ? "crystal" : "no crystal, HSI", (unsigned long)(hse ? hse / 1000000u : 16u));
  }
  return name;
}
uint32_t consoleBaud() { return 0; }  // USB CDC
int imuIntPin() {
#if defined(PIN_IMU_INT)
  return PIN_IMU_INT;
#else
  return -1;
#endif
}
int resetReason() { return stm32::bootResetReason(); }  // flags read (and cleared) once in main()
static const char* pinName(int p, char* b) {
  if (p < 0) { strcpy(b, "-"); return b; }
  snprintf(b, 6, "P%c%d", 'A' + (p >> 4), p & 15);
  return b;
}
const char* pinSummary() {
  static char s[200];
  if (!s[0]) {
    char a[6], b[6], c[6], d[6], e[6], f[6], g[6];
    snprintf(s, sizeof(s), "SPI SCK %s MISO %s MOSI %s | IMU CS %s INT %s | I2C SDA %s SCL %s", pinName(PIN_SPI_SCK, a),
             pinName(PIN_SPI_MISO, b), pinName(PIN_SPI_MOSI, c), pinName(PIN_IMU_CS, d), pinName(imuIntPin(), e),
             pinName(PIN_I2C_SDA, f), pinName(PIN_I2C_SCL, g));
  }
  return s;
}
uint32_t deviceId() {
#if defined(AQ_QEMU)
  return 0x0A110AD5u;  // QEMU maps no unique-ID area
#else
  return HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
#endif
}
uint32_t random32() {
  static uint32_t x = 0;
  x ^= deviceId() ^ DWT->CYCCNT ^ (uint32_t)stm32::timeUs64();
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return x;
}

// ============================================================ serial ports (interrupt driven)
namespace {
#if defined(STM32H7xx)
constexpr uint32_t URX = 1024, UTX = 512;
#else
constexpr uint32_t URX = 1024, UTX = 256;  // 128 kB RAM parts
#endif
struct Uart {
  USART_TypeDef* reg = nullptr;
  int rx = -1, tx = -1;
  uint8_t af = 0;
  bool open = false;
  uint8_t rxb[URX];
  uint8_t txb[UTX];
  volatile uint32_t rh = 0, rt = 0, th = 0, tt = 0;
  volatile uint32_t overruns = 0, framing = 0, noise = 0, dropped = 0, rxBytes = 0, txBytes = 0;
};
// one slot per serial port the board has (buffers are 1.3-1.6 kB each: the F405 / F411 have 128 kB RAM)
#if defined(AQ_SERIAL7_UART)
constexpr int UART_SLOTS = 7;
#elif defined(AQ_SERIAL6_UART)
constexpr int UART_SLOTS = 6;
#elif defined(AQ_SERIAL5_UART)
constexpr int UART_SLOTS = 5;
#elif defined(AQ_SERIAL4_UART)
constexpr int UART_SLOTS = 4;
#else
constexpr int UART_SLOTS = 3;
#endif
Uart uarts[UART_SLOTS];

int uartSlot(Port p) {
  switch (p) {
    case Port::SERIAL1: return 0;
    case Port::SERIAL2: return 1;
    case Port::SERIAL3: return 2;
    case Port::SERIAL4: return 3;
    case Port::SERIAL5: return 4;
    case Port::SERIAL6: return 5;
    case Port::SERIAL7: return 6;
    default: return -1;
  }
}
void uartSetup() {
  static bool done = false;
  if (done) return;
  done = true;
#if defined(AQ_SERIAL1_UART)
  uarts[0].reg = AQ_SERIAL1_UART; uarts[0].af = AQ_SERIAL1_AF; uarts[0].rx = PIN_SERIAL1_RX; uarts[0].tx = PIN_SERIAL1_TX;
#endif
#if defined(AQ_SERIAL2_UART)
  uarts[1].reg = AQ_SERIAL2_UART; uarts[1].af = AQ_SERIAL2_AF; uarts[1].rx = PIN_SERIAL2_RX; uarts[1].tx = PIN_SERIAL2_TX;
#endif
#if defined(AQ_SERIAL3_UART)
  uarts[2].reg = AQ_SERIAL3_UART; uarts[2].af = AQ_SERIAL3_AF; uarts[2].rx = PIN_SERIAL3_RX; uarts[2].tx = PIN_SERIAL3_TX;
#endif
#if defined(AQ_SERIAL4_UART) && defined(PIN_SERIAL4_RX)
  uarts[3].reg = AQ_SERIAL4_UART; uarts[3].af = AQ_SERIAL4_AF; uarts[3].rx = PIN_SERIAL4_RX; uarts[3].tx = PIN_SERIAL4_TX;
#endif
#if defined(AQ_SERIAL5_UART) && defined(PIN_SERIAL5_RX)
  uarts[4].reg = AQ_SERIAL5_UART; uarts[4].af = AQ_SERIAL5_AF; uarts[4].rx = PIN_SERIAL5_RX; uarts[4].tx = PIN_SERIAL5_TX;
#endif
#if defined(AQ_SERIAL6_UART) && defined(PIN_SERIAL6_RX)
  uarts[5].reg = AQ_SERIAL6_UART; uarts[5].af = AQ_SERIAL6_AF; uarts[5].rx = PIN_SERIAL6_RX; uarts[5].tx = PIN_SERIAL6_TX;
#endif
#if defined(AQ_SERIAL7_UART) && defined(PIN_SERIAL7_RX)
  uarts[6].reg = AQ_SERIAL7_UART; uarts[6].af = AQ_SERIAL7_AF; uarts[6].rx = PIN_SERIAL7_RX; uarts[6].tx = PIN_SERIAL7_TX;
#endif
  for (Uart& u : uarts)
    if (u.rx < 0 || u.tx < 0) u.reg = nullptr;
}
#if defined(AQ_QEMU)
constexpr int QEMU_CONSOLE_SLOT = 1;  // USART1 = QEMU serial0: the console, not a SERIALx port
#endif
Uart* uartOf(Port p) {
  uartSetup();
  int i = uartSlot(p);
#if defined(AQ_QEMU)
  if (i == QEMU_CONSOLE_SLOT) return nullptr;
#endif
  return (i >= 0 && i < UART_SLOTS && uarts[i].reg) ? &uarts[i] : nullptr;
}

// clock enable, interrupt number, kernel clock (APB2 for USART1/6, APB1 for the rest)
bool uartHw(USART_TypeDef* r, IRQn_Type& irq, bool& apb2) {
  apb2 = false;
  if (r == USART1) { __HAL_RCC_USART1_CLK_ENABLE(); irq = USART1_IRQn; apb2 = true; return true; }
  if (r == USART2) { __HAL_RCC_USART2_CLK_ENABLE(); irq = USART2_IRQn; return true; }
#if defined(USART3)
  if (r == USART3) { __HAL_RCC_USART3_CLK_ENABLE(); irq = USART3_IRQn; return true; }
#endif
#if defined(UART4)
  if (r == UART4) { __HAL_RCC_UART4_CLK_ENABLE(); irq = UART4_IRQn; return true; }
#endif
#if defined(UART5)
  if (r == UART5) { __HAL_RCC_UART5_CLK_ENABLE(); irq = UART5_IRQn; return true; }
#endif
#if defined(USART6)
  if (r == USART6) { __HAL_RCC_USART6_CLK_ENABLE(); irq = USART6_IRQn; apb2 = true; return true; }
#endif
#if defined(UART7)
  if (r == UART7) { __HAL_RCC_UART7_CLK_ENABLE(); irq = UART7_IRQn; return true; }
#endif
#if defined(UART8)
  if (r == UART8) { __HAL_RCC_UART8_CLK_ENABLE(); irq = UART8_IRQn; return true; }
#endif
  return false;
}

inline void uartIrq(USART_TypeDef* r) {
  Uart* u = nullptr;
  for (Uart& x : uarts)
    if (x.reg == r) { u = &x; break; }
  if (!u) { r->CR1 &= ~(uint32_t)0x1F0; return; }  // not ours: all interrupt enables off
#if defined(AQ_USART_V1)
  uint32_t sr = r->SR;
  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    uint8_t c = (uint8_t)r->DR;  // SR then DR read clears every error flag
    if (sr & USART_SR_ORE) u->overruns = u->overruns + 1;
    if (sr & USART_SR_FE) u->framing = u->framing + 1;
    if (sr & (USART_SR_NE | USART_SR_PE)) u->noise = u->noise + 1;
    if (sr & USART_SR_RXNE) {
      u->rxBytes = u->rxBytes + 1;
      uint32_t n = (u->rh + 1) % URX;
      if (n != u->rt) { u->rxb[u->rh] = c; u->rh = n; }
      else u->dropped = u->dropped + 1;
    }
  }
  if ((r->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
    if (u->tt != u->th) {
      u->txBytes = u->txBytes + 1;
      r->DR = u->txb[u->tt];
      u->tt = (u->tt + 1) % UTX;
    } else {
      r->CR1 &= ~USART_CR1_TXEIE;
    }
  }
#else
  uint32_t isr = r->ISR;
  if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
    #if defined(USART_ICR_NECF)
    r->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
#else
    r->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_PECF;
#endif
    if (isr & USART_ISR_ORE) u->overruns = u->overruns + 1;
    if (isr & USART_ISR_FE) u->framing = u->framing + 1;
    if (isr & (USART_ISR_NE | USART_ISR_PE)) u->noise = u->noise + 1;
  }
  if (isr & AQ_ISR_RXNE) {
    uint8_t c = (uint8_t)r->RDR;
    u->rxBytes = u->rxBytes + 1;
    uint32_t n = (u->rh + 1) % URX;
    if (n != u->rt) { u->rxb[u->rh] = c; u->rh = n; }
    else u->dropped = u->dropped + 1;
  }
  if ((r->CR1 & AQ_CR1_TXEIE) && (isr & AQ_ISR_TXE)) {
    if (u->tt != u->th) {
      u->txBytes = u->txBytes + 1;
      r->TDR = u->txb[u->tt];
      u->tt = (u->tt + 1) % UTX;
    } else {
      r->CR1 &= ~AQ_CR1_TXEIE;
    }
  }
#endif
}
}  // namespace

bool portExists(Port p) {
  if (p == Port::CONSOLE) return true;
  if (p == Port::UDP) return false;
  return uartOf(p) != nullptr;
}
const char* portLabel(Port p) {
  static char buf[(int)Port::COUNT][32];
  Uart* u = uartOf(p);
  if (!u) return "";
  const char* nm = u->reg == USART1 ? "USART1" : u->reg == USART2 ? "USART2" : "UART";
#if defined(USART3)
  if (u->reg == USART3) nm = "USART3";
#endif
#if defined(UART4)
  if (u->reg == UART4) nm = "UART4";
#endif
#if defined(UART5)
  if (u->reg == UART5) nm = "UART5";
#endif
#if defined(USART6)
  if (u->reg == USART6) nm = "USART6";
#endif
#if defined(UART7)
  if (u->reg == UART7) nm = "UART7";
#endif
#if defined(UART8)
  if (u->reg == UART8) nm = "UART8";
#endif
  // pins are (port << 4) | n
  snprintf(buf[(int)p], sizeof(buf[0]), "%s RX P%c%d TX P%c%d", nm, 'A' + (u->rx >> 4), u->rx & 15, 'A' + (u->tx >> 4), u->tx & 15);
  return buf[(int)p];
}

static void uartOpen(Uart* u, uint32_t baud, UartFormat fmt, bool inverted);
void portOpen(Port p, uint32_t baud, UartFormat fmt, bool inverted) {
  Uart* u = uartOf(p);
  if (!u || !baud) return;
  uartOpen(u, baud, fmt, inverted);
}
static void uartOpen(Uart* u, uint32_t baud, UartFormat fmt, bool inverted) {
  USART_TypeDef* r = u->reg;
  IRQn_Type irq;
  bool apb2;
  if (!uartHw(r, irq, apb2)) return;
  NVIC_DisableIRQ(irq);
  r->CR1 = 0;
  u->rh = u->rt = 0;
  u->th = u->tt = 0;
  u->overruns = u->framing = u->noise = u->dropped = u->rxBytes = u->txBytes = 0;
  // RX idles high (pull-up keeps an unplugged receiver quiet); inverted (SBUS): idles low
  gpio::config(u->rx, gpio::AF, inverted ? gpio::DOWN : gpio::UP, false, 2, u->af);
  gpio::config(u->tx, gpio::AF, gpio::NOPULL, false, 2, u->af);
  uint32_t clk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  r->BRR = (clk + baud / 2) / baud;  // oversampling x16
  uint32_t cr1 = USART_CR1_TE | USART_CR1_RE;
  uint32_t cr2 = 0;
  if (fmt == UartFormat::F8E2) { cr1 |= AQ_CR1_M9 | USART_CR1_PCE; cr2 |= USART_CR2_STOP_1; }
  if (fmt == UartFormat::F8N2) cr2 |= USART_CR2_STOP_1;
#if defined(USART_CR2_RXINV)
  if (inverted) cr2 |= USART_CR2_RXINV | USART_CR2_TXINV;  // F7 / H7: SBUS without an external inverter
#endif
  r->CR2 = cr2;
#if defined(USART_CR3_OVRDIS)
  r->CR3 = USART_CR3_OVRDIS;  // an overrun never stops reception
  r->ICR = 0xFFFFFFFFu;
#else
  r->CR3 = 0;
  (void)r->SR;
  (void)r->DR;
#endif
#if defined(AQ_USART_V1)
  r->CR1 = cr1 | USART_CR1_RXNEIE;
  r->CR1 |= USART_CR1_UE;
#else
  r->CR1 = cr1 | AQ_CR1_RXNEIE;
  r->CR1 |= USART_CR1_UE;
#endif
  HAL_NVIC_SetPriority(irq, AQ_PRIO_UART, 0);
  NVIC_ClearPendingIRQ(irq);
  NVIC_EnableIRQ(irq);
  u->open = true;
}

int portRead(Port p) {
#if defined(AQ_QEMU)
  Uart* u = p == Port::CONSOLE ? &uarts[QEMU_CONSOLE_SLOT] : uartOf(p);
#else
  if (p == Port::CONSOLE) return aqUsbRead();
  Uart* u = uartOf(p);
#endif
  if (!u || !u->open || u->rt == u->rh) return -1;
  uint8_t c = u->rxb[u->rt];
  u->rt = (u->rt + 1) % URX;
  return c;
}

size_t portWrite(Port p, const uint8_t* d, size_t n) {
#if defined(AQ_QEMU)
  if (p == Port::CONSOLE) {  // blocking: QEMU sends each byte at once
    USART_TypeDef* r = uarts[QEMU_CONSOLE_SLOT].reg;
    for (size_t i = 0; i < n; i++) {
      while (!(r->SR & USART_SR_TXE)) {}
      r->DR = d[i];
    }
    return n;
  }
#endif
  if (p == Port::CONSOLE) return aqUsbWrite(d, n);
  Uart* u = uartOf(p);
  if (!u || !u->open) return 0;
  size_t done = 0;
  uint32_t h = u->th;
  while (done < n) {
    uint32_t next = (h + 1) % UTX;
    if (next == u->tt) break;
    u->txb[h] = d[done++];
    h = next;
  }
  u->th = h;
  if (done) {
    uint32_t s = irqSave();
#if defined(AQ_USART_V1)
    u->reg->CR1 |= USART_CR1_TXEIE;
#else
    u->reg->CR1 |= AQ_CR1_TXEIE;
#endif
    irqRestore(s);
  }
  return done;
}

bool portStats(Port p, PortStats& o) {
  Uart* u = uartOf(p);
  if (!u || !u->open) return false;
  o.rxBytes = u->rxBytes; o.txBytes = u->txBytes; o.overruns = u->overruns;
  o.framing = u->framing; o.noise = u->noise; o.dropped = u->dropped;
  return true;
}

size_t portWriteSpace(Port p) {
#if defined(AQ_QEMU)
  if (p == Port::CONSOLE) return 4096;
#endif
  if (p == Port::CONSOLE) return aqUsbWriteSpace();
  Uart* u = uartOf(p);
  if (!u || !u->open) return 0;
  return (u->tt - u->th - 1 + UTX) % UTX;
}

}  // namespace hal

extern "C" {
void USART1_IRQHandler(void) { hal::uartIrq(USART1); }
void USART2_IRQHandler(void) { hal::uartIrq(USART2); }
#if defined(USART3)
void USART3_IRQHandler(void) { hal::uartIrq(USART3); }
#endif
#if defined(UART4)
void UART4_IRQHandler(void) { hal::uartIrq(UART4); }
#endif
#if defined(UART5)
void UART5_IRQHandler(void) { hal::uartIrq(UART5); }
#endif
#if defined(USART6)
void USART6_IRQHandler(void) { hal::uartIrq(USART6); }
#endif
#if defined(UART7)
void UART7_IRQHandler(void) { hal::uartIrq(UART7); }
#endif
#if defined(UART8)
void UART8_IRQHandler(void) { hal::uartIrq(UART8); }
#endif
}

namespace hal {

// ============================================================ I2C
static I2C_HandleTypeDef hi2c;
static uint32_t i2cHz = 400000;
static bool i2cUp = false;
static char i2cDiagText[96] = "";

static void i2cClockEnable() {
  if (AQ_I2C == I2C1) __HAL_RCC_I2C1_CLK_ENABLE();
  else if (AQ_I2C == I2C2) __HAL_RCC_I2C2_CLK_ENABLE();
#if defined(I2C3)
  else if (AQ_I2C == I2C3) __HAL_RCC_I2C3_CLK_ENABLE();
#endif
}
static void i2cResetPeripheral() {
  if (AQ_I2C == I2C1) { __HAL_RCC_I2C1_FORCE_RESET(); __HAL_RCC_I2C1_RELEASE_RESET(); }
  else if (AQ_I2C == I2C2) { __HAL_RCC_I2C2_FORCE_RESET(); __HAL_RCC_I2C2_RELEASE_RESET(); }
}

#if !defined(STM32F4xx)
// TIMINGR for the I2C kernel clock (F7: PCLK1, H7: PCLK1 = rcc_pclk1): standard / fast mode timing
// with margin for slow rising edges (module pull-ups 4.7-10 k, internal ~40 k as a backup)
static uint32_t i2cTiming(uint32_t clk, uint32_t hz) {
  bool fast = hz > 100000;
  uint32_t tickHz = fast ? 16000000u : 8000000u;
  uint32_t presc = (clk + tickHz - 1) / tickHz;
  if (presc < 1) presc = 1;
  if (presc > 16) presc = 16;
  float tick = presc * 1e9f / clk;  // ns
  float period = 1e9f / hz;
  float overhead = fast ? 300.0f : 600.0f;  // rise + fall + synchronisation
  float lo = (period - overhead) * 0.58f, hi = (period - overhead) * 0.42f;
  if (lo < (fast ? 1300.0f : 4700.0f)) lo = fast ? 1300.0f : 4700.0f;
  if (hi < (fast ? 600.0f : 4000.0f)) hi = fast ? 600.0f : 4000.0f;
  uint32_t scll = (uint32_t)(lo / tick + 0.5f), sclh = (uint32_t)(hi / tick + 0.5f);
  uint32_t scldel = (uint32_t)((fast ? 400.0f : 1250.0f) / tick + 0.5f);
  uint32_t sdadel = (uint32_t)((fast ? 200.0f : 500.0f) / tick + 0.5f);
  if (scll < 2) scll = 2;
  if (sclh < 2) sclh = 2;
  if (scll > 256) scll = 256;
  if (sclh > 256) sclh = 256;
  if (scldel < 1) scldel = 1;
  if (scldel > 16) scldel = 16;
  if (sdadel > 15) sdadel = 15;
  return ((presc - 1) << 28) | ((scldel - 1) << 20) | (sdadel << 16) | ((sclh - 1) << 8) | (scll - 1);
}
#endif

static void i2cPinsAf() {
  gpio::config(PIN_I2C_SDA, gpio::AF, gpio::UP, true, 1, AQ_I2C_AF);
  gpio::config(PIN_I2C_SCL, gpio::AF, gpio::UP, true, 1, AQ_I2C_AF);
}

// up to 9 clocks + STOP free a slave that holds SDA low (reset in the middle of a read)
static int i2cBusRecover() {
  gpio::config(PIN_I2C_SDA, gpio::IN, gpio::UP);
  gpio::write(PIN_I2C_SCL, true);
  gpio::config(PIN_I2C_SCL, gpio::OUT, gpio::UP, true, 1);
  delayUs(10);
  int clocks = 0;
  for (; clocks < 9 && !gpio::read(PIN_I2C_SDA); clocks++) {
    gpio::write(PIN_I2C_SCL, false);
    delayUs(5);
    gpio::write(PIN_I2C_SCL, true);
    delayUs(5);
  }
  gpio::write(PIN_I2C_SDA, false);  // STOP: SDA low -> high while SCL is high
  gpio::config(PIN_I2C_SDA, gpio::OUT, gpio::UP, true, 1);
  delayUs(5);
  gpio::write(PIN_I2C_SDA, true);
  delayUs(5);
  return clocks;
}

static void i2cStart() {
  i2cClockEnable();
  i2cResetPeripheral();
  i2cPinsAf();
  memset(&hi2c, 0, sizeof(hi2c));
  hi2c.Instance = AQ_I2C;
#if defined(STM32F4xx)
  hi2c.Init.ClockSpeed = i2cHz;
  hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
#else
  hi2c.Init.Timing = i2cTiming(HAL_RCC_GetPCLK1Freq(), i2cHz);
#endif
  hi2c.Init.OwnAddress1 = 0;
  hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  i2cUp = HAL_I2C_Init(&hi2c) == HAL_OK;
#if !defined(STM32F4xx)
  if (i2cUp) {
    HAL_I2CEx_ConfigAnalogFilter(&hi2c, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c, 0);
  }
#endif
}

// after a timeout / bus error: the peripheral (F4 errata: BUSY flag stuck) and the bus are reset
static void i2cRecover() {
  HAL_I2C_DeInit(&hi2c);
  i2cBusRecover();
  i2cStart();
}
static bool i2cResult(HAL_StatusTypeDef s) {
  if (s == HAL_OK) return true;
  uint32_t e = HAL_I2C_GetError(&hi2c);
  // a missing device only NACKs; anything else (timeout, bus error, arbitration lost) resets the bus
  if (s != HAL_ERROR || (e & ~(uint32_t)HAL_I2C_ERROR_AF)) i2cRecover();
  return false;
}
static uint32_t i2cTimeout(size_t len) {
  uint32_t perByteUs = 9000000u / i2cHz + 10;
  return 2 + (uint32_t)((len + 3) * perByteUs / 1000u);
}

// a probe of an absent address only NACKs: the bus is reset only when the peripheral stays busy
// (SDA / SCL held low, F4 BUSY-flag errata)
static bool i2cBusBusy() {
#if defined(STM32F4xx)
  return (AQ_I2C->SR2 & I2C_SR2_BUSY) != 0;
#else
  return (AQ_I2C->ISR & I2C_ISR_BUSY) != 0;
#endif
}
bool i2cProbe(uint8_t addr) {
  if (!i2cUp) return false;
  if (HAL_I2C_IsDeviceReady(&hi2c, (uint16_t)(addr << 1), 1, 3) == HAL_OK) return true;
  if (i2cBusBusy() || hi2c.State != HAL_I2C_STATE_READY) i2cRecover();
  return false;
}
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  if (!i2cUp) return false;
  return i2cResult(HAL_I2C_Mem_Read(&hi2c, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, i2cTimeout(len)));
}
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  if (!i2cUp) return false;
  return i2cResult(HAL_I2C_Mem_Write(&hi2c, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, i2cTimeout(1)));
}
bool i2cCommand(uint8_t addr, uint8_t cmd) {
  if (!i2cUp) return false;
  return i2cResult(HAL_I2C_Master_Transmit(&hi2c, (uint16_t)(addr << 1), &cmd, 1, i2cTimeout(1)));
}
bool i2cReadRaw(uint8_t addr, uint8_t* buf, uint8_t len) {
  if (!i2cUp) return false;
  return i2cResult(HAL_I2C_Master_Receive(&hi2c, (uint16_t)(addr << 1), buf, len, i2cTimeout(len)));
}
void i2cSetClock(uint32_t hz) {
  if (hz < 10000) hz = 10000;
  if (hz > 1000000) hz = 1000000;
  i2cHz = hz;
  HAL_I2C_DeInit(&hi2c);
  i2cStart();
}
uint32_t i2cClock() { return i2cHz; }

// SDA / SCL before the I2C peripheral takes the pins: internal pull-downs on, so a line reads high only
// if an external pull-up holds it
static void i2cPrepare() {
  gpio::config(PIN_I2C_SDA, gpio::IN, gpio::DOWN);
  gpio::config(PIN_I2C_SCL, gpio::IN, gpio::DOWN);
  delayUs(200);
  bool sdaHi = gpio::read(PIN_I2C_SDA), sclHi = gpio::read(PIN_I2C_SCL);
  int clocks = 0;
  if (sclHi && !sdaHi) clocks = i2cBusRecover();
  char a[6], b[6];
  snprintf(i2cDiagText, sizeof(i2cDiagText), "SDA %s %s, SCL %s %s%s%s", pinName(PIN_I2C_SDA, a), sdaHi ? "high" : "LOW",
           pinName(PIN_I2C_SCL, b), sclHi ? "high" : "LOW",
           !sdaHi || !sclHi ? " (no pull-up / short to GND / module unpowered)" : " (pull-ups ok)",
           clocks ? " - bus was stuck, freed" : "");
}
const char* i2cDiag() { return i2cDiagText; }

void i2cLineLevels(bool& sda, bool& scl) {
  sda = gpio::read(PIN_I2C_SDA);
  scl = gpio::read(PIN_I2C_SCL);
}

// ~10 kHz bit-banged scan with the internal pull-ups (open drain: drive low, release high)
static void odSet(int p, bool high) {
  if (high) gpio::config(p, gpio::IN, gpio::UP);
  else { gpio::write(p, false); gpio::config(p, gpio::OUT, gpio::UP, true, 1); }
  delayUs(50);
}
int i2cSoftScan(bool swapped, uint8_t* out, int max) {
  int sda = swapped ? PIN_I2C_SCL : PIN_I2C_SDA, scl = swapped ? PIN_I2C_SDA : PIN_I2C_SCL;
  HAL_I2C_DeInit(&hi2c);
  i2cUp = false;
  odSet(sda, true);
  odSet(scl, true);
  int n = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    odSet(sda, false);  // START
    odSet(scl, false);
    uint8_t byte = (uint8_t)(a << 1);
    for (int i = 7; i >= 0; i--) {
      odSet(sda, (byte >> i) & 1);
      odSet(scl, true);
      odSet(scl, false);
    }
    odSet(sda, true);  // ACK slot
    odSet(scl, true);
    bool ack = !gpio::read(sda);
    odSet(scl, false);
    odSet(sda, false);  // STOP
    odSet(scl, true);
    odSet(sda, true);
    if (ack && n < max) out[n++] = a;
  }
  i2cStart();
  return n;
}

// ============================================================ SPI
struct SpiBus {
  SPI_TypeDef* reg = nullptr;
  SPI_HandleTypeDef h;
  uint32_t kernelHz = 0;
  uint32_t curHz = 0;
  bool up = false;
};
static SpiBus spiImu, spiOsd;
static uint32_t spiHz[3] = {1000000, 1000000, 1000000};

static int csPin(SpiDev d) {
  switch (d) {
    case SpiDev::IMU: return PIN_IMU_CS;
    case SpiDev::IMU2: return PIN_IMU2_CS;
    case SpiDev::OSD: return PIN_OSD_CS;
  }
  return -1;
}
static SpiBus* spiBus(SpiDev d) {
  SpiBus* b = d == SpiDev::OSD ? &spiOsd : &spiImu;
  return b->up ? b : nullptr;
}

static uint32_t spiPrescBits(SpiBus& b, uint32_t hz, uint32_t& actual) {
  uint32_t div = 2, code = 0;
  while (div < 256 && b.kernelHz / div > hz) { div <<= 1; code++; }
  actual = b.kernelHz / div;
  return code;
}

static void spiBusInit(SpiBus& b, SPI_TypeDef* reg, uint8_t af, int sck, int miso, int mosi) {
  if (sck < 0 || miso < 0 || mosi < 0) return;
  b.reg = reg;
  if (reg == SPI1) {
    __HAL_RCC_SPI1_CLK_ENABLE();
#if defined(STM32H7xx)
    b.kernelHz = stm32::spiKernelHz();  // SPI123: PLL1Q
#else
    b.kernelHz = HAL_RCC_GetPCLK2Freq();
#endif
  } else if (reg == SPI2) {
    __HAL_RCC_SPI2_CLK_ENABLE();
#if defined(STM32H7xx)
    b.kernelHz = stm32::spiKernelHz();
#else
    b.kernelHz = HAL_RCC_GetPCLK1Freq();
#endif
  }
#if defined(SPI3)
  else if (reg == SPI3) {
    __HAL_RCC_SPI3_CLK_ENABLE();
#if defined(STM32H7xx)
    b.kernelHz = stm32::spiKernelHz();
#else
    b.kernelHz = HAL_RCC_GetPCLK1Freq();
#endif
  }
#endif
  gpio::config(sck, gpio::AF, gpio::NOPULL, false, 3, af);
  gpio::config(miso, gpio::AF, gpio::UP, false, 3, af);
  gpio::config(mosi, gpio::AF, gpio::NOPULL, false, 3, af);
  memset(&b.h, 0, sizeof(b.h));
  b.h.Instance = reg;
  b.h.Init.Mode = SPI_MODE_MASTER;
  b.h.Init.Direction = SPI_DIRECTION_2LINES;
  b.h.Init.DataSize = SPI_DATASIZE_8BIT;
  b.h.Init.CLKPolarity = SPI_POLARITY_LOW;
  b.h.Init.CLKPhase = SPI_PHASE_1EDGE;
  b.h.Init.NSS = SPI_NSS_SOFT;
  b.h.Init.FirstBit = SPI_FIRSTBIT_MSB;
  b.h.Init.TIMode = SPI_TIMODE_DISABLE;
  b.h.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  b.h.Init.CRCPolynomial = 7;
  uint32_t act;
  uint32_t code = spiPrescBits(b, 1000000, act);
#if defined(STM32H7xx)
  b.h.Init.BaudRatePrescaler = code << SPI_CFG1_MBR_Pos;
#else
  b.h.Init.BaudRatePrescaler = code << SPI_CR1_BR_Pos;
#endif
#if defined(STM32H7xx)
  b.h.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  b.h.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  b.h.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  b.h.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  b.h.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  b.h.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  b.h.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  // SCK stays low between transfers
  b.h.Init.IOSwap = SPI_IO_SWAP_DISABLE;
#elif defined(STM32F7xx)
  b.h.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
#endif
  b.up = HAL_SPI_Init(&b.h) == HAL_OK;
  b.curHz = act;
#if !defined(STM32H7xx)
  if (b.up) reg->CR1 |= SPI_CR1_SPE;
#endif
}

static void spiApplySpeed(SpiBus& b, uint32_t hz) {
  uint32_t act;
  uint32_t code = spiPrescBits(b, hz, act);
  if (act == b.curHz) return;
  b.curHz = act;
#if defined(STM32H7xx)
  b.reg->CR1 &= ~SPI_CR1_SPE;
  b.reg->CFG1 = (b.reg->CFG1 & ~SPI_CFG1_MBR) | (code << SPI_CFG1_MBR_Pos);
  b.h.Init.BaudRatePrescaler = code << SPI_CFG1_MBR_Pos;
#else
  while (b.reg->SR & SPI_SR_BSY) {}
  b.reg->CR1 &= ~SPI_CR1_SPE;
  b.reg->CR1 = (b.reg->CR1 & ~SPI_CR1_BR) | (code << SPI_CR1_BR_Pos);
  b.reg->CR1 |= SPI_CR1_SPE;
  b.h.Init.BaudRatePrescaler = code << SPI_CR1_BR_Pos;
#endif
}

// full-duplex transfer of n bytes (rx may be nullptr, tx nullptr = zeros)
static bool spiXfer(SpiBus& b, const uint8_t* tx, uint8_t* rx, size_t n) {
#if defined(STM32H7xx)
  static uint8_t zeros[64];
  static uint8_t sink[64];
  while (n) {
    size_t k = n < 64 ? n : 64;
    if (HAL_SPI_TransmitReceive(&b.h, (uint8_t*)(tx ? tx : zeros), rx ? rx : sink, (uint16_t)k, 5) != HAL_OK) {
      HAL_SPI_Abort(&b.h);
      return false;
    }
    if (tx) tx += k;
    if (rx) rx += k;
    n -= k;
  }
  return true;
#else
  SPI_TypeDef* r = b.reg;
  volatile uint8_t* dr = (volatile uint8_t*)&r->DR;  // 8-bit access (F7: no data packing)
  while (r->SR & SPI_SR_RXNE) (void)*dr;
  for (size_t i = 0; i < n; i++) {
    uint32_t guard = 0;
    while (!(r->SR & SPI_SR_TXE))
      if (++guard > 200000) return false;
    *dr = tx ? tx[i] : 0;
    guard = 0;
    while (!(r->SR & SPI_SR_RXNE))
      if (++guard > 200000) return false;
    uint8_t v = *dr;
    if (rx) rx[i] = v;
  }
  while (r->SR & SPI_SR_BSY) {}
  return true;
#endif
}

bool spiHasDevice(SpiDev d) { return csPin(d) >= 0 && spiBus(d) != nullptr; }
void spiSetSpeed(SpiDev d, uint32_t hz) { spiHz[(int)d] = hz; }

bool spiRead(SpiDev d, uint8_t reg, uint8_t* buf, size_t len, bool dummy) {
  SpiBus* b = spiBus(d);
  int cs = csPin(d);
  if (!b || cs < 0) return false;
  spiApplySpeed(*b, spiHz[(int)d]);
  uint8_t head[2] = {(uint8_t)(reg | 0x80), 0};
  gpio::write(cs, false);
  bool ok = spiXfer(*b, head, nullptr, dummy ? 2 : 1);
  if (ok) ok = spiXfer(*b, nullptr, buf, len);
  gpio::write(cs, true);
  if (!ok) memset(buf, 0, len);
  return ok;
}
bool spiWrite(SpiDev d, uint8_t reg, uint8_t val) {
  SpiBus* b = spiBus(d);
  int cs = csPin(d);
  if (!b || cs < 0) return false;
  uint32_t hz = spiHz[(int)d] < 1000000 ? spiHz[(int)d] : 1000000;  // registers: slow & safe
  if (d == SpiDev::OSD) hz = spiHz[(int)d];
  spiApplySpeed(*b, hz);
  uint8_t t[2] = {(uint8_t)(reg & 0x7F), val};
  gpio::write(cs, false);
  bool ok = spiXfer(*b, t, nullptr, 2);
  gpio::write(cs, true);
  return ok;
}
bool spiWriteRaw(SpiDev d, const uint8_t* data, size_t len) {
  SpiBus* b = spiBus(d);
  int cs = csPin(d);
  if (!b || cs < 0) return false;
  spiApplySpeed(*b, spiHz[(int)d]);
  gpio::write(cs, false);
  bool ok = spiXfer(*b, data, nullptr, len);
  gpio::write(cs, true);
  return ok;
}

// ============================================================ analog
static ADC_HandleTypeDef hadc;
static bool adcUp = false;
static void adcInit() {
  gpio::config(PIN_VBAT, gpio::ANALOG);
  if (PIN_CURRENT >= 0) gpio::config(PIN_CURRENT, gpio::ANALOG);
#if defined(STM32H7xx)
  RCC_PeriphCLKInitTypeDef pc = {};
  pc.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  pc.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;  // per_ck = HSI 64 MHz
  HAL_RCCEx_PeriphCLKConfig(&pc);
  __HAL_RCC_ADC12_CLK_ENABLE();
#else
  __HAL_RCC_ADC1_CLK_ENABLE();
#endif
  memset(&hadc, 0, sizeof(hadc));
  hadc.Instance = ADC1;
#if defined(STM32H7xx)
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;  // 16 MHz
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc.Init.OversamplingMode = DISABLE;
  adcUp = HAL_ADC_Init(&hadc) == HAL_OK;
  if (adcUp) HAL_ADCEx_Calibration_Start(&hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
#else
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.ScanConvMode = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  adcUp = HAL_ADC_Init(&hadc) == HAL_OK;
#endif
}
static float adcRead(uint32_t channel) {
  if (!adcUp) return 0;
  ADC_ChannelConfTypeDef c = {};
  c.Channel = channel;
#if defined(STM32H7xx)
  c.Rank = ADC_REGULAR_RANK_1;
  c.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  c.SingleDiff = ADC_SINGLE_ENDED;
  c.OffsetNumber = ADC_OFFSET_NONE;
#else
  c.Rank = 1;
  c.SamplingTime = ADC_SAMPLETIME_84CYCLES;
#endif
  if (HAL_ADC_ConfigChannel(&hadc, &c) != HAL_OK) return 0;
  HAL_ADC_Start(&hadc);
  float v = 0;
  if (HAL_ADC_PollForConversion(&hadc, 2) == HAL_OK) v = HAL_ADC_GetValue(&hadc) * (3.3f / 4095.0f);
  HAL_ADC_Stop(&hadc);
  return v;
}
float adcVbatPin() { return adcRead(AQ_ADC_VBAT_CH); }
float adcCurrentPin() { return PIN_CURRENT >= 0 ? adcRead(AQ_ADC_CUR_CH) : -1.0f; }

// ============================================================ indicators
void ledSet(bool on) {
  if (PIN_LED >= 0) gpio::write(PIN_LED, on ^ (LED_ACTIVE_LOW != 0));
}
void buzzerSet(bool on) {
  if (PIN_BUZZER >= 0) gpio::write(PIN_BUZZER, on);
}

// ============================================================ init
void init() {
  stm32::usTimerStart();
#if defined(AQ_QEMU)
  uartSetup();
  uartOpen(&uarts[QEMU_CONSOLE_SLOT], 115200, UartFormat::F8N1, false);
#else
  aqUsbInit();
#endif
  if (PIN_LED >= 0) gpio::config(PIN_LED, gpio::OUT, gpio::NOPULL, false, 0);
  if (PIN_BUZZER >= 0) gpio::config(PIN_BUZZER, gpio::OUT, gpio::NOPULL, false, 0);
  ledSet(false);
  buzzerSet(false);
  const SpiDev devs[] = {SpiDev::IMU, SpiDev::IMU2, SpiDev::OSD};
  for (SpiDev d : devs) {
    int cs = csPin(d);
    if (cs >= 0) { gpio::write(cs, true); gpio::config(cs, gpio::OUT, gpio::NOPULL, false, 2); }
  }
  i2cPrepare();
  i2cStart();
  spiBusInit(spiImu, AQ_SPI_IMU, AQ_SPI_IMU_AF, PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
#if defined(AQ_SPI_OSD)
  if (PIN_OSD_CS >= 0) spiBusInit(spiOsd, AQ_SPI_OSD, AQ_SPI_OSD_AF, PIN_OSD_SCK, PIN_OSD_MISO, PIN_OSD_MOSI);
#endif
  adcInit();
  resetReason();
}

// ============================================================ wireless / firmware slots (none)
bool wifiStart(const char*, const char*) { return false; }
void wifiSetEnabled(bool) {}
void netReport(char* out, size_t n) { snprintf(out, n, "no WiFi on this board\r\n"); }
FirmwareInfo firmwareInfo() { return FirmwareInfo{"flash", false, false}; }
void firmwareConfirm() {}
void firmwareReject() {}
void reboot() {
  delayMs(20);  // let the last USB packet go out
  NVIC_SystemReset();
}
bool bootloaderSupported() { return true; }
void rebootToBootloader() {
  stm32::requestBootloader();
  delayMs(20);
  NVIC_SystemReset();
}
void markFastLoopDone() {}

// ============================================================ storage (8 kB images at the end of a sector)
// Image layout: settings 0-2047, rally 2048 (328 B), fence 2560 (520 B), mission 3584 (120 waypoints,
// 3856 B), WiFi 7600 (172 B), footer 8176-8191 {magic, sequence, crc32 of 0-8175, ~magic}.
// H743 and F405 keep two copies in two sectors (A/B): a save erases and writes the copy that is NOT in
// use, so a power loss during the 1-2 s erase / program leaves the previous settings intact; the valid
// copy with the newest sequence is used. F722 / F411 have no spare sector (one copy, as before).
// An image written before the footer existed is read as it is (copy A) and the first save goes to B.
static constexpr uint32_t EE_SIZE = ee::SIZE, EE_FOOT = ee::FOOT;
struct EeCopy { uint32_t addr, sector; };
#if defined(STM32H7xx)
static const EeCopy EE[] = {{0x081FE000u, 7}, {0x081DE000u, 6}};  // bank 2 sectors 7, 6 (blackbox: 0-5)
#elif defined(STM32F405xx)
static const EeCopy EE[] = {{0x080FE000u, FLASH_SECTOR_11}, {0x080DE000u, FLASH_SECTOR_10}};  // firmware <= 768 kB
#else  // F722 / F411: 512 kB, sector 7
static const EeCopy EE[] = {{0x0807E000u, FLASH_SECTOR_7}};
#endif
static constexpr int EE_COPIES = (int)(sizeof(EE) / sizeof(EE[0]));
static int eeActive = -1;         // copy in use (-1: not chosen yet)
static uint32_t eeSeq = 0;        // its sequence number (0: image without footer)
static volatile bool eeWriting = false;  // a save is erasing / programming (blackbox waits)

static size_t slotOffset(Slot s) {
  return s == Slot::CONFIG ? 0 : s == Slot::RALLY ? 2048 : s == Slot::FENCE ? 2560 : s == Slot::MISSION ? 3584 : 7600;
}
static void eeInvalidate() {
#if defined(STM32H7xx)
  for (const EeCopy& c : EE) SCB_InvalidateDCache_by_Addr((uint32_t*)c.addr, (int32_t)EE_SIZE);
#endif
}
static void eeChoose() {
  if (eeActive >= 0) return;
  eeInvalidate();
  const uint8_t* c[EE_COPIES];
  for (int i = 0; i < EE_COPIES; i++) c[i] = (const uint8_t*)EE[i].addr;
  eeActive = ee::pick(c, EE_COPIES, eeSeq);
}
static bool eeFlush(const uint8_t* img, int copy) {
  const EeCopy& c = EE[copy];
  HAL_FLASH_Unlock();
#if defined(STM32H7xx)
  __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);
#elif defined(FLASH_FLAG_PGSERR)
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                         FLASH_FLAG_PGSERR);
#else
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                         FLASH_FLAG_ERSERR);
#endif
  FLASH_EraseInitTypeDef er = {};
  uint32_t bad = 0;
  er.TypeErase = FLASH_TYPEERASE_SECTORS;
#if defined(STM32H7xx)
  er.Banks = FLASH_BANK_2;
#endif
  er.Sector = c.sector;
  er.NbSectors = 1;
  er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  stm32::watchdogFlashErase();  // F4 / F7: the CPU stalls for the whole erase (1-2 s, up to 4 s)
  bool ok = HAL_FLASHEx_Erase(&er, &bad) == HAL_OK;
#if defined(STM32H7xx)
  for (uint32_t o = 0; ok && o < EE_SIZE; o += 32)
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, c.addr + o, (uint32_t)(img + o)) == HAL_OK;
#else
  for (uint32_t o = 0; ok && o < EE_SIZE; o += 4) {
    uint32_t w;
    memcpy(&w, img + o, 4);
    if (w != 0xFFFFFFFFu) ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, c.addr + o, w) == HAL_OK;
  }
#endif
  HAL_FLASH_Lock();
  eeInvalidate();
  return ok;
}
bool storageLoad(Slot s, void* data, size_t len) {
  size_t off = slotOffset(s);
  if (off + len > EE_FOOT) return false;
  eeChoose();
  eeInvalidate();
  memcpy(data, (const void*)(EE[eeActive].addr + off), len);
  return true;
}
bool storageSave(Slot s, const void* data, size_t len) {
  size_t off = slotOffset(s);
  if (off + len > EE_FOOT) return false;
  // one save at a time (CLI, web, deferred save from the control task)
  taskENTER_CRITICAL();
  bool busy = eeWriting;
  eeWriting = true;
  taskEXIT_CRITICAL();
  if (busy) return false;
  eeChoose();
  eeInvalidate();
  const uint8_t* cur = (const uint8_t*)EE[eeActive].addr;
  bool ok = true;
  if (memcmp(cur + off, data, len)) {  // unchanged: no flash wear
    uint8_t* img = (uint8_t*)pvPortMalloc(EE_SIZE + 32);
    if (!img) ok = false;
    else {
      uint8_t* a = (uint8_t*)(((uintptr_t)img + 31) & ~(uintptr_t)31);  // H7 flash words: 32-byte aligned source
      memcpy(a, cur, EE_SIZE);
      memcpy(a + off, data, len);
      uint32_t seq;
      int target = ee::seal(a, eeActive, eeSeq, EE_COPIES, seq);
      ok = eeFlush(a, target);
      uint32_t back = 0;  // read back: the copy must check out before it is used
      ok = ok && ee::valid((const uint8_t*)EE[target].addr, back) && back == seq && !memcmp((const void*)EE[target].addr, a, EE_SIZE);
      if (ok) { eeActive = target; eeSeq = seq; }
      else if (EE_COPIES == 1) eeActive = -1;  // choose again (the only copy may be half written)
      vPortFree(img);
    }
  }
  eeWriting = false;
  return ok;
}
// which copy is in use (CLI "status" / selftest): copy index, its sequence, number of copies
void storageInfo(int& copy, uint32_t& seq, int& copies) {
  eeChoose();
  copy = eeActive;
  seq = eeSeq;
  copies = EE_COPIES;
}

// ============================================================ motors
static const int motorPins[4] = {PIN_MOTOR1, PIN_MOTOR2, PIN_MOTOR3, PIN_MOTOR4};
static fc::MotorProtocol motorProto = fc::MotorProtocol::DSHOT300;
static bool motorsReady = false;

// ---- PWM / OneShot125 on the motor pins' timer (8 ticks per microsecond)
static const uint8_t pwmCh[4] = AQ_PWM_CHANNELS;
static uint32_t pwmMinTicks = 8000, pwmRangeTicks = 8000;
static bool pwmActive = false;
static volatile uint32_t* pwmCcr(int ch) {
  TIM_TypeDef* t = AQ_PWM_TIM;
  switch (ch) {
    case 1: return &t->CCR1;
    case 2: return &t->CCR2;
    case 3: return &t->CCR3;
    default: return &t->CCR4;
  }
}
static bool pwmInit(bool oneshot) {
  TIM_TypeDef* t = AQ_PWM_TIM;
  stm32::timerClockEnable(t);
  t->CR1 = 0;
  t->PSC = stm32::timerClock(t) / 8000000u - 1;
  uint32_t periodUs = oneshot ? 500 : 2500;
  pwmMinTicks = (oneshot ? 125 : 1000) * 8;
  pwmRangeTicks = (oneshot ? 125 : 1000) * 8;
  t->ARR = periodUs * 8 - 1;
  t->CCMR1 = (6u << 4) | TIM_CCMR1_OC1PE | (6u << 12) | TIM_CCMR1_OC2PE;  // PWM1 + preload
  t->CCMR2 = (6u << 4) | TIM_CCMR2_OC3PE | (6u << 12) | TIM_CCMR2_OC4PE;
  uint32_t ccer = 0;
  for (int i = 0; i < 4; i++) {
    *pwmCcr(pwmCh[i]) = pwmMinTicks;
    ccer |= TIM_CCER_CC1E << ((pwmCh[i] - 1) * 4);
    gpio::config(motorPins[i], gpio::AF, gpio::NOPULL, false, 2, AQ_PWM_AF);
  }
  t->CCER = ccer;
  if (t == TIM1
#if defined(TIM8)
      || t == TIM8
#endif
  )
    t->BDTR = TIM_BDTR_MOE;
  t->EGR = TIM_EGR_UG;
  t->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
  pwmActive = true;
  return true;
}
static void pwmStop() {
  if (!pwmActive) return;
  AQ_PWM_TIM->CR1 = 0;
  AQ_PWM_TIM->CCER = 0;
  pwmActive = false;
}

// ---- DShot: 3 DMA slots per bit -> "0" = 33 % high, "1" = 67 % high (Betaflight bit-bang scheme).
// Bidirectional DShot: the signal is inverted (idle high); after the frame the same DMA stream samples
// the GPIO input register at 3x the telemetry bit rate while the ESC answers, the next motorsWrite()
// decodes the samples of every pin.
static GPIO_TypeDef* dshotPort = nullptr;
static uint32_t dshotMask[4];
static uint32_t dshotAllMask = 0;
alignas(32) static uint32_t dshotBuf[56];
static bool bidir = false, bidirWanted = false, bdSampling = false;
static uint32_t bitrateHz = 300000;
static constexpr int BD_SAMPLES = 150;
alignas(32) static uint16_t bdSamples[160];
static float bdErpm[4] = {-1, -1, -1, -1};
static uint32_t bdOkMs[4], bdFrames, bdErrors;
static dshot::Edt bdEdt[4];

static inline void cacheClean(const void* p, size_t n) {
#if defined(STM32H7xx)
  SCB_CleanDCache_by_Addr((uint32_t*)((uintptr_t)p & ~31u), (int32_t)((n + 63) & ~31u));
#else
  (void)p; (void)n;
#endif
}
static inline void cacheInvalidate(const void* p, size_t n) {
#if defined(STM32H7xx)
  SCB_InvalidateDCache_by_Addr((uint32_t*)p, (int32_t)((n + 31) & ~31u));
#else
  (void)p; (void)n;
#endif
}
static void pinsMode(uint32_t moder) {
  for (int i = 0; i < 4; i++) {
    uint32_t n = 31 - __builtin_clz(dshotMask[i]);
    dshotPort->MODER = (dshotPort->MODER & ~(3u << (n * 2))) | (moder << (n * 2));
  }
}

static bool dshotInit(bool d600) {
  pwmStop();
  dshotAllMask = 0;
  for (int i = 0; i < 4; i++) {
    GPIO_TypeDef* port = gpio::port(motorPins[i]);
    if (i == 0) dshotPort = port;
    else if (port != dshotPort) return false;  // the bit-bang needs all four on one port
    dshotMask[i] = gpio::mask(motorPins[i]);
    dshotAllMask |= dshotMask[i];
    gpio::write(motorPins[i], bidir);
    gpio::config(motorPins[i], gpio::OUT, bidir ? gpio::UP : gpio::NOPULL, false, 3);
  }
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  bitrateHz = d600 ? 600000 : 300000;
  TIM1->CR1 = 0;
  TIM1->PSC = 0;
  TIM1->ARR = stm32::timerClock(TIM1) / (bitrateHz * 3) - 1;
  TIM1->DIER = TIM_DIER_UDE;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 = TIM_CR1_CEN;
  DMA_Stream_TypeDef* st = DMA2_Stream5;
  st->CR = 0;
  while (st->CR & DMA_SxCR_EN) {}
  st->PAR = (uint32_t)&dshotPort->BSRR;
  st->FCR = 0;
#if defined(STM32H7xx)
  DMAMUX1_Channel13->CCR = DMA_REQUEST_TIM1_UP;  // DMA2 stream 5 = DMAMUX1 channel 8 + 5
  st->CR = DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
#else
  st->CR = (6u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
#endif
  return true;
}
static void dmaClearFlags() {
  DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
}
static void bidirCollect() {
  DMA_Stream_TypeDef* st = DMA2_Stream5;
  if (!bdSampling) return;
  bool complete = !(st->CR & DMA_SxCR_EN);
  if (!complete) { st->CR &= ~DMA_SxCR_EN; while (st->CR & DMA_SxCR_EN) {} }
  bdSampling = false;
  uint32_t now = millis();
  int n = BD_SAMPLES - (int)st->NDTR;
  cacheInvalidate(bdSamples, sizeof(bdSamples));
  for (int m = 0; m < 4; m++) {
    float e;
    bdFrames++;
    uint8_t kind = n > 40 ? dshot::decodeSamplesEx(bdSamples, n, (uint16_t)dshotMask[m], 3.0f, e, &bdEdt[m]) : dshot::BAD;
    if (kind == dshot::ERPM) { bdErpm[m] = e; bdOkMs[m] = now; }
    else if (kind == dshot::BAD) bdErrors++;
    if (now - bdOkMs[m] > 100) bdErpm[m] = -1;
  }
}
static void dshotSend(const uint16_t values[4], int telemetryMotor, bool allTelemetry) {
  DMA_Stream_TypeDef* st = DMA2_Stream5;
  if (bidir) bidirCollect();
  else if (st->CR & DMA_SxCR_EN) return;  // previous frame still running
  uint16_t pkt[4];
  for (int m = 0; m < 4; m++) pkt[m] = dshot::frame(values[m], allTelemetry || m == telemetryMotor, bidir);
  for (int b = 0; b < 16; b++) {
    uint32_t zeros = 0;
    for (int m = 0; m < 4; m++)
      if (!(pkt[m] & (0x8000 >> b))) zeros |= dshotMask[m];
    if (bidir) {
      dshotBuf[b * 3 + 0] = dshotAllMask << 16;
      dshotBuf[b * 3 + 1] = zeros;
      dshotBuf[b * 3 + 2] = dshotAllMask;
    } else {
      dshotBuf[b * 3 + 0] = dshotAllMask;
      dshotBuf[b * 3 + 1] = zeros << 16;
      dshotBuf[b * 3 + 2] = dshotAllMask << 16;
    }
  }
  dshotBuf[48] = bidir ? dshotAllMask : dshotAllMask << 16;
  if (bidir) {
    pinsMode(1);
    TIM1->ARR = stm32::timerClock(TIM1) / (bitrateHz * 3) - 1;
    TIM1->EGR = TIM_EGR_UG;
    st->PAR = (uint32_t)&dshotPort->BSRR;
    st->CR = (st->CR & ~(DMA_SxCR_DIR | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE)) | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_DIR_0;
  }
  cacheClean(dshotBuf, 49 * 4);
  dmaClearFlags();
  st->M0AR = (uint32_t)dshotBuf;
  st->NDTR = 49;
  st->CR |= DMA_SxCR_EN;
  if (!bidir) return;
  uint32_t guard = 0;
  while ((st->CR & DMA_SxCR_EN) && ++guard < 1000000) {}
  pinsMode(0);
  TIM1->ARR = stm32::timerClock(TIM1) * 4 / (bitrateHz * 15) - 1;  // 3 samples per telemetry bit
  TIM1->EGR = TIM_EGR_UG;
  st->PAR = (uint32_t)&dshotPort->IDR;
  st->CR = (st->CR & ~(DMA_SxCR_DIR | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE)) | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0;
  cacheInvalidate(bdSamples, sizeof(bdSamples));
  dmaClearFlags();
  st->M0AR = (uint32_t)bdSamples;
  st->NDTR = BD_SAMPLES;
  st->CR |= DMA_SxCR_EN;
  bdSampling = true;
}

bool motorsAreDshot() { return motorProto == fc::MotorProtocol::DSHOT300 || motorProto == fc::MotorProtocol::DSHOT600; }
bool motorsBidirActive() { return bidir && motorsReady; }
void motorsErpm(float erpm[4]) {
  for (int m = 0; m < 4; m++) erpm[m] = bidir ? bdErpm[m] : -1;
}
void motorsBidirStats(uint32_t& frames, uint32_t& errors) { frames = bdFrames; errors = bdErrors; }
bool motorsEdt(int o, dshot::Edt& out) {
  if (o < 0 || o > 3 || !bdEdt[o].frames) return false;
  out = bdEdt[o];
  return true;
}
bool motorsInit(fc::MotorProtocol p, uint16_t, bool wantBidir) {
  motorProto = p;
  bidirWanted = wantBidir;
  bidir = wantBidir && motorsAreDshot();
  motorsReady = motorsAreDshot() ? dshotInit(p == fc::MotorProtocol::DSHOT600) : pwmInit(p == fc::MotorProtocol::ONESHOT125);
  float stop[4] = {0, 0, 0, 0};
  motorsWrite(stop, false);
  return motorsReady;
}
void motorsWrite(const float out[4], bool armed, int telemetryMotor) {
  if (!motorsReady) return;
  if (motorsAreDshot()) {
    uint16_t v[4];
    for (int m = 0; m < 4; m++) v[m] = (armed && out[m] >= 0) ? (uint16_t)(48 + fc::constrainf(out[m], 0, 1) * 1999.0f) : 0;
    dshotSend(v, telemetryMotor, false);
  } else {
    for (int m = 0; m < 4; m++) {
      float o = (armed && out[m] >= 0) ? fc::constrainf(out[m], 0, 1) : 0.0f;
      *pwmCcr(pwmCh[m]) = pwmMinTicks + (uint32_t)(o * pwmRangeTicks);
    }
  }
}
void motorsCommandMask(uint8_t cmd, uint8_t mask) {
  if (!motorsReady || !motorsAreDshot() || cmd > 47) return;
  uint16_t v[4];
  for (int m = 0; m < 4; m++) v[m] = (mask >> m) & 1 ? cmd : 0;
  for (int i = 0; i < 10; i++) {
    dshotSend(v, -1, true);
    delayUs(1000);
  }
}
void motorsCommand(uint8_t cmd) { motorsCommandMask(cmd, 0x0F); }

// ============================================================ ESC passthrough (one-wire 19200 baud)
static bool escActive = false;
bool escSerialBegin() {
  if (escActive) return true;
  if (motorsAreDshot()) {
    if (bdSampling) { DMA2_Stream5->CR &= ~DMA_SxCR_EN; bdSampling = false; }
    while (DMA2_Stream5->CR & DMA_SxCR_EN) {}
    TIM1->CR1 = 0;
  } else {
    pwmStop();
  }
  for (int i = 0; i < 4; i++) gpio::config(motorPins[i], gpio::IN, gpio::UP);
  motorsReady = false;
  escActive = true;
  return true;
}
void escSerialEnd() {
  if (!escActive) return;
  escActive = false;
  motorsInit(motorProto, 0, bidirWanted);
}
static inline void waitCycles(uint32_t until) {
  while ((int32_t)(fc::cycles() - until) < 0) {}
}
void escSerialWrite(int esc, const uint8_t* d, size_t n) {
  if (!escActive || esc < 0 || esc > 3) return;
  int pin = motorPins[esc];
  const uint32_t bit = (uint32_t)(fc::cyclesPerUs() * 52.083f);
  gpio::write(pin, true);
  gpio::config(pin, gpio::OUT, gpio::UP, false, 2);
  for (size_t i = 0; i < n; i++) {
    uint16_t frame = (uint16_t)((d[i] << 1) | (1u << 9));
    uint32_t s = irqSave();
    uint32_t t = fc::cycles();
    for (int k = 0; k < 10; k++) {
      gpio::write(pin, (frame >> k) & 1);
      t += bit;
      waitCycles(t);
    }
    irqRestore(s);
  }
  gpio::config(pin, gpio::IN, gpio::UP);
}
int escSerialRead(int esc, uint32_t timeoutMs) {
  if (!escActive || esc < 0 || esc > 3) return -1;
  int pin = motorPins[esc];
  const uint32_t bit = (uint32_t)(fc::cyclesPerUs() * 52.083f);
  uint32_t t0 = millis();
  for (;;) {
    uint32_t s = irqSave();
    uint32_t sliceEnd = fc::cycles() + (uint32_t)(fc::cyclesPerUs() * 1000);
    while (gpio::read(pin) && (int32_t)(fc::cycles() - sliceEnd) < 0) {}
    if (!gpio::read(pin)) {
      uint32_t t = fc::cycles() + bit / 2;
      waitCycles(t);
      int v = -1;
      if (!gpio::read(pin)) {
        uint8_t b = 0;
        for (int k = 0; k < 8; k++) {
          t += bit;
          waitCycles(t);
          if (gpio::read(pin)) b |= (uint8_t)(1u << k);
        }
        t += bit;
        waitCycles(t);
        if (gpio::read(pin)) v = b;
      }
      irqRestore(s);
      return v;
    }
    irqRestore(s);
    if (millis() - t0 >= timeoutMs) return -1;
  }
}

// ============================================================ LED strip (WS2812 / SK6812)
// TIM4 channel 3 PWM on PIN_LED_STRIP (PB8) with DMA1 Stream6 (TIM4_UP) reloading the compare register
// every bit: 800 kHz, no CPU time.
#if defined(PIN_LED_STRIP)
static constexpr int LED_MAX = 32;
alignas(32) static uint16_t ledBuf[LED_MAX * 24 + 48];
static bool ledReady = false;
static uint16_t ledT0 = 0, ledT1 = 0;

bool ledStripBegin(uint8_t pin, uint8_t count) {
  if (!pin || !count) return false;
  gpio::write(PIN_LED_STRIP, false);
  gpio::config(PIN_LED_STRIP, gpio::AF, gpio::NOPULL, false, 3, 2);  // AF2 = TIM4
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  uint32_t arr = stm32::timerClock(TIM4) / 800000;
  ledT0 = (uint16_t)(arr * 32 / 100);
  ledT1 = (uint16_t)(arr * 64 / 100);
  TIM4->CR1 = 0;
  TIM4->PSC = 0;
  TIM4->ARR = arr - 1;
  TIM4->CCR3 = 0;
  TIM4->CCMR2 = (TIM4->CCMR2 & ~0xFFu) | (6u << 4) | (1u << 3);
  TIM4->CCER |= TIM_CCER_CC3E;
  TIM4->DIER = TIM_DIER_UDE;
  TIM4->EGR = TIM_EGR_UG;
  TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
  DMA_Stream_TypeDef* st = DMA1_Stream6;
  st->CR = 0;
  while (st->CR & DMA_SxCR_EN) {}
  st->PAR = (uint32_t)&TIM4->CCR3;
  st->FCR = 0;
#if defined(STM32H7xx)
  DMAMUX1_Channel6->CCR = DMA_REQUEST_TIM4_UP;
  st->CR = DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
#else
  st->CR = (2u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
#endif
  ledReady = true;
  return true;
}
void ledStripShow(const uint8_t* grb, uint8_t count) {
  DMA_Stream_TypeDef* st = DMA1_Stream6;
  if (!ledReady || (st->CR & DMA_SxCR_EN)) return;
  if (count > LED_MAX) count = LED_MAX;
  int k = 0;
  for (int i = 0; i < count * 3; i++)
    for (int b = 7; b >= 0; b--) ledBuf[k++] = (grb[i] >> b) & 1 ? ledT1 : ledT0;
  for (int i = 0; i < 48; i++) ledBuf[k++] = 0;
  cacheClean(ledBuf, sizeof(ledBuf));
  DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
  st->M0AR = (uint32_t)ledBuf;
  st->NDTR = (uint32_t)k;
  st->CR |= DMA_SxCR_EN;
}
#else
bool ledStripBegin(uint8_t, uint8_t) { return false; }
void ledStripShow(const uint8_t*, uint8_t) {}
#endif

// ============================================================ blackbox in the internal flash (H7 bank 2)
#if defined(AQ_HAS_FLASH_LOG) && defined(STM32H7xx) && defined(FLASH_BANK_2)
// Sectors 0-5 of bank 2 (768 kB); the settings own sectors 6 and 7 (two copies). Programming / erasing bank 2 does not
// stall code fetches from bank 1. 32-byte flash words, each programmed once (ECC); a tail < 32 bytes
// waits in RAM. Erase: one 128 kB sector at a time, started by register and polled (never blocks).
static constexpr uint32_t BB_BASE = 0x08100000u;
static constexpr uint32_t BB_SECTOR = 128u * 1024u;
static constexpr uint32_t BB_SECTORS = 6;
static constexpr uint32_t BB_SIZE = BB_SECTOR * BB_SECTORS;
alignas(32) static uint8_t bbWord[32];
static size_t bbFill = 0;
static uint32_t bbEnd = 0;
static uint32_t bbErased = BB_SIZE;
static uint32_t bbEraseNext = 0, bbEraseTo = 0;
static bool bbErasing = false, bbScanned = false;

static bool bbErasedAt(uint32_t off) {
  const uint32_t* w = (const uint32_t*)(BB_BASE + off);
  SCB_InvalidateDCache_by_Addr((uint32_t*)w, 32);
  for (int i = 0; i < 8; i++)
    if (w[i] != 0xFFFFFFFFu) return false;
  return true;
}
static void bbScan() {
  if (bbScanned) return;
  bbScanned = true;
  uint32_t lo = 0, hi = BB_SIZE / 32;
  while (lo < hi) {
    uint32_t mid = (lo + hi) / 2;
    if (bbErasedAt(mid * 32)) hi = mid;
    else lo = mid + 1;
  }
  bbEnd = lo * 32;
  uint32_t s = bbEnd;
  while (s < BB_SIZE && bbErasedAt(s) && bbErasedAt(((s / BB_SECTOR) + 1) * BB_SECTOR - 32)) s = (s / BB_SECTOR + 1) * BB_SECTOR;
  bbErased = s;
  if (bbErased < BB_SIZE) {
    bbEraseNext = bbErased / BB_SECTOR;
    bbEraseTo = BB_SECTORS;
    if (bbEnd > bbEraseNext * BB_SECTOR) bbEnd = bbEraseNext * BB_SECTOR;
  }
}
static bool bbBusy() { return (FLASH->SR2 & FLASH_SR_QW) != 0; }
static void bbUnlock() {
  if (FLASH->CR2 & FLASH_CR_LOCK) { FLASH->KEYR2 = 0x45670123u; FLASH->KEYR2 = 0xCDEF89ABu; }
}
bool logOpen() {
  bbScan();
  return true;
}
size_t logWrite(const uint8_t* d, size_t n) {
  size_t done = 0;
  while (done < n) {
    if (bbEnd + 32 > bbErased || bbEraseNext < bbEraseTo) return done;
    size_t k = n - done < 32 - bbFill ? n - done : 32 - bbFill;
    memcpy(bbWord + bbFill, d + done, k);
    bbFill += k;
    done += k;
    if (bbFill == 32) {
      if (eeWriting) { bbFill -= k; return done - k; }  // a settings save owns bank 2 for now
      HAL_FLASH_Unlock();
      bool ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, BB_BASE + bbEnd, (uint32_t)bbWord) == HAL_OK;
      HAL_FLASH_Lock();
      if (!ok) return done;
      bbEnd += 32;
      bbFill = 0;
    }
  }
  return done;
}
void logFlush() {}
void logErase() {
  bbScan();
  uint32_t used = bbEnd + (uint32_t)bbFill;
  bbEraseNext = 0;
  bbEraseTo = (used + BB_SECTOR - 1) / BB_SECTOR;
  if (bbErased < BB_SIZE) bbEraseTo = BB_SECTORS;
  bbEnd = 0;
  bbFill = 0;
  bbErased = bbEraseTo ? 0 : BB_SIZE;
}
void logService(bool armed) {
  if (bbErasing) {
    if (bbBusy()) return;
    FLASH->CCR2 = FLASH_CCR_CLR_EOP;
    FLASH->CR2 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    HAL_FLASH_Lock();
    bbErasing = false;
    bbEraseNext++;
    SCB_InvalidateDCache_by_Addr((uint32_t*)BB_BASE, (int32_t)BB_SIZE);
    if (bbEraseNext >= bbEraseTo) bbErased = BB_SIZE;
    else bbErased = bbEraseNext * BB_SECTOR;
    return;
  }
  if (armed || bbEraseNext >= bbEraseTo || bbBusy() || eeWriting) return;
  bbUnlock();
  FLASH->CR2 = (FLASH->CR2 & ~(FLASH_CR_SNB | FLASH_CR_PSIZE)) | FLASH_CR_SER | FLASH_CR_PSIZE_1 | (bbEraseNext << FLASH_CR_SNB_Pos);
  FLASH->CR2 |= FLASH_CR_START;
  bbErasing = true;
}
bool logBusy() { return bbErasing || bbEraseNext < bbEraseTo; }
uint32_t logSize() { return bbEnd + (uint32_t)bbFill; }
uint32_t logCapacity() { return BB_SIZE; }
bool logRead(uint32_t off, uint8_t* buf, size_t len) {
  if (off + len > BB_SIZE) return false;
  for (size_t i = 0; i < len; i++) {
    uint32_t o = off + (uint32_t)i;
    if (o < bbEnd) continue;
    buf[i] = (o < bbEnd + bbFill) ? bbWord[o - bbEnd] : 0xFF;
  }
  if (off < bbEnd) {
    size_t n = bbEnd - off < len ? bbEnd - off : len;
    SCB_InvalidateDCache_by_Addr((uint32_t*)((BB_BASE + off) & ~31u), (int32_t)(n + 64));
    memcpy(buf, (const void*)(BB_BASE + off), n);
  }
  return true;
}
#else
bool logOpen() { return false; }
size_t logWrite(const uint8_t*, size_t) { return 0; }
void logFlush() {}
void logErase() {}
uint32_t logSize() { return 0; }
uint32_t logCapacity() { return 0; }
bool logRead(uint32_t, uint8_t*, size_t) { return false; }
void logService(bool) {}
bool logBusy() { return false; }
#endif

}  // namespace hal
#endif
