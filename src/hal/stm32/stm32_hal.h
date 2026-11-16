// AllQuad - internal interface between the STM32Cube system files (src/hal/stm32/*) and the HAL
// implementation (hal_stm32cube.cpp). Not used by code above the HAL.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "stm32_mcu.h"

#ifdef __cplusplus
namespace hal {
namespace stm32 {
// ---- system_stm32.cpp ----
void usTimerStart();                   // 1 MHz free-running time base (hal::init)
uint64_t timeUs64();                   // microseconds since the time base started, never wraps
uint32_t timerClock(TIM_TypeDef* t);   // input clock of a timer (APB x1 / x2)
void timerClockEnable(TIM_TypeDef* t);
void requestBootloader();              // ROM DFU on the next reset
uint32_t hseFrequency();               // crystal in Hz, 0 = internal oscillator
}  // namespace stm32
}  // namespace hal
extern "C" {
#endif

// ---- usb_cdc.c: USB virtual COM port (console / configurator) ----
void aqUsbInit(void);
int aqUsbRead(void);                                  // -1 if empty
size_t aqUsbWrite(const uint8_t* data, size_t len);   // queued, sent from the USB interrupt
size_t aqUsbWriteSpace(void);
int aqUsbConnected(void);                             // host opened the port (DTR) / configured

#ifdef __cplusplus
}
#endif
