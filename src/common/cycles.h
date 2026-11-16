// AllQuad - CPU cycle counter for profiling the flight-critical path (cheap: one register read).
//   ESP32 / ESP32-S3 : Xtensa CCOUNT        STM32 : DWT->CYCCNT (enabled at start, src/hal/stm32/system_stm32.cpp)
//   host             : nanoseconds (cyclesPerUs = 1000)
#pragma once
#include <stdint.h>

#if defined(ALLQUAD_HOST)
#include <chrono>
namespace fc {
inline uint32_t cycles() {
  return (uint32_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline float cyclesPerUs() { return 1000.0f; }
}  // namespace fc
#elif defined(ESP32)
#include <Arduino.h>
#include <esp_cpu.h>
namespace fc {
inline uint32_t cycles() { return (uint32_t)esp_cpu_get_cycle_count(); }
inline float cyclesPerUs() { return (float)getCpuFrequencyMhz(); }
}  // namespace fc
#elif defined(AQ_STM32) && defined(AQ_QEMU)
// QEMU models no DWT: cycles from the 1 MHz time base (TIM2 on the F405 layout)
#include "../hal/stm32/stm32_mcu.h"
namespace fc {
inline uint32_t cycles() { return TIM2->CNT * (SystemCoreClock / 1000000u); }
inline float cyclesPerUs() { return SystemCoreClock * 1e-6f; }
}  // namespace fc
#elif defined(AQ_STM32)
#include "../hal/stm32/stm32_mcu.h"
namespace fc {
inline uint32_t cycles() { return DWT->CYCCNT; }
inline float cyclesPerUs() { return SystemCoreClock * 1e-6f; }
}  // namespace fc
#else
namespace fc {
inline uint32_t cycles() { return 0; }
inline float cyclesPerUs() { return 1.0f; }
}  // namespace fc
#endif

// Flight-critical code that must not wait for the flash cache (ESP32: IRAM). Empty elsewhere:
// STM32 executes from flash through the ART accelerator at zero wait states.
#if defined(ESP32) && !defined(ALLQUAD_HOST)
#include <esp_attr.h>
#define FAST_CODE IRAM_ATTR
#else
#define FAST_CODE
#endif
