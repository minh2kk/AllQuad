// AllQuad - quadcopter flight controller firmware for ESP32 and STM32.
// All work runs in RTOS tasks created by app::firmwareSetup() (see src/app/app.h).
// ESP32: Arduino setup() / loop(). STM32 (STM32Cube, no Arduino): main() is in src/hal/stm32/system_stm32.cpp.
#if !defined(ALLQUAD_HOST) && defined(ARDUINO)
#include <Arduino.h>

#include "app/app.h"

void setup() { app::firmwareSetup(); }
void loop() { app::firmwareLoop(); }
#endif
