// AllQuad - STM32 peripheral routing per board (the pins themselves are in ../board.h).
// SERIALn: USART instance + alternate function; I2C / SPI buses; PWM timer for the motors; ADC channels
// of the battery pins; the two timers the system uses (1 MHz time base, flight-loop wake-up).
#pragma once
#include "stm32_mcu.h"

#if defined(BOARD_STM32H743_WEACT) || defined(BOARD_STM32H743_DEVEBOX)
#define AQ_SERIAL1_UART USART3   // PD9 / PD8
#define AQ_SERIAL1_AF 7
#define AQ_SERIAL2_UART UART7    // PE7 / PE8
#define AQ_SERIAL2_AF 7
#define AQ_SERIAL3_UART UART8    // PE0 / PE1
#define AQ_SERIAL3_AF 8
#define AQ_SERIAL4_UART USART1   // PA10 / PA9
#define AQ_SERIAL4_AF 7
#define AQ_SERIAL5_UART USART2   // PD6 / PD5
#define AQ_SERIAL5_AF 7
#define AQ_SERIAL6_UART UART4    // PD0 / PD1
#define AQ_SERIAL6_AF 8
#define AQ_SERIAL7_UART USART6   // PC7 / PC6
#define AQ_SERIAL7_AF 7
#define AQ_I2C I2C2              // PB11 / PB10
#define AQ_I2C_AF 4
#define AQ_SPI_IMU SPI2          // PB13-PB15
#define AQ_SPI_IMU_AF 5
#define AQ_PWM_TIM TIM2          // PA0-PA3 = CH1-CH4
#define AQ_PWM_AF 1
#define AQ_PWM_CHANNELS {1, 2, 3, 4}
#define AQ_ADC_VBAT_CH ADC_CHANNEL_10  // PC0
#define AQ_ADC_CUR_CH ADC_CHANNEL_11   // PC1
#define AQ_US_TIM TIM5           // 32-bit, 1 MHz free running
#define AQ_US_TIM_IRQn TIM5_IRQn
#define AQ_US_TIM_IRQHandler TIM5_IRQHandler
#define AQ_LOOP_TIM TIM7
#define AQ_LOOP_TIM_IRQn TIM7_IRQn
#define AQ_LOOP_TIM_IRQHandler TIM7_IRQHandler
#define AQ_HSE_HZ 25000000u      // both boards carry a 25 MHz crystal

#elif defined(BOARD_STM32F405) || defined(BOARD_STM32F722)
#define AQ_SERIAL1_UART USART2   // PA3 / PA2
#define AQ_SERIAL1_AF 7
#define AQ_SERIAL2_UART USART1   // PA10 / PA9
#define AQ_SERIAL2_AF 7
#define AQ_SERIAL3_UART UART4    // PC11 / PC10
#define AQ_SERIAL3_AF 8
#define AQ_SERIAL4_UART UART5    // PD2 / PC12
#define AQ_SERIAL4_AF 8
#define AQ_SERIAL5_UART USART6   // PC7 / PC6
#define AQ_SERIAL5_AF 8
#define AQ_SERIAL6_UART USART3   // PB11 / PB10
#define AQ_SERIAL6_AF 7
#define AQ_I2C I2C1              // PB7 / PB6
#define AQ_I2C_AF 4
#define AQ_SPI_IMU SPI1          // PA5-PA7
#define AQ_SPI_IMU_AF 5
#define AQ_SPI_OSD SPI2          // PB13-PB15
#define AQ_SPI_OSD_AF 5
#define AQ_PWM_TIM TIM3          // PB4, PB5, PB0, PB1
#define AQ_PWM_AF 2
#define AQ_PWM_CHANNELS {1, 2, 3, 4}
#define AQ_ADC_VBAT_CH ADC_CHANNEL_11  // PC1
#define AQ_ADC_CUR_CH ADC_CHANNEL_12   // PC2
#define AQ_US_TIM TIM2
#define AQ_US_TIM_IRQn TIM2_IRQn
#define AQ_US_TIM_IRQHandler TIM2_IRQHandler
#define AQ_LOOP_TIM TIM5
#define AQ_LOOP_TIM_IRQn TIM5_IRQn
#define AQ_LOOP_TIM_IRQHandler TIM5_IRQHandler
#define AQ_HSE_HZ 0u             // measured at start (8 MHz on most flight controllers)

#elif defined(BOARD_STM32F411_BLACKPILL)
#define AQ_SERIAL1_UART USART2   // PA3 / PA2
#define AQ_SERIAL1_AF 7
#define AQ_SERIAL2_UART USART1   // PA10 / PA9
#define AQ_SERIAL2_AF 7
#define AQ_I2C I2C1              // PB7 / PB6
#define AQ_I2C_AF 4
#define AQ_SPI_IMU SPI1          // PA5-PA7
#define AQ_SPI_IMU_AF 5
#define AQ_SPI_OSD SPI2          // PB13-PB15
#define AQ_SPI_OSD_AF 5
#define AQ_PWM_TIM TIM3          // PB4, PB5, PB0, PB1
#define AQ_PWM_AF 2
#define AQ_PWM_CHANNELS {1, 2, 3, 4}
#define AQ_ADC_VBAT_CH ADC_CHANNEL_0   // PA0
#define AQ_ADC_CUR_CH ADC_CHANNEL_1    // PA1
#define AQ_US_TIM TIM2
#define AQ_US_TIM_IRQn TIM2_IRQn
#define AQ_US_TIM_IRQHandler TIM2_IRQHandler
#define AQ_LOOP_TIM TIM5
#define AQ_LOOP_TIM_IRQn TIM5_IRQn
#define AQ_LOOP_TIM_IRQHandler TIM5_IRQHandler
#define AQ_HSE_HZ 0u             // 25 MHz (WeAct v2/v3.0) or 8 MHz (some v3.1): measured at start

#else
#error "no STM32 peripheral map for this board (src/hal/stm32/board_stm32.h)"
#endif

// interrupt priorities (0 = most urgent). <= 4: never calls FreeRTOS; >= 5: may (configMAX_SYSCALL 5)
#define AQ_PRIO_UART 1
#define AQ_PRIO_US_TIM 2
#define AQ_PRIO_LOOP_TIM 5
#define AQ_PRIO_EXTI 5
#define AQ_PRIO_USB 6
#define AQ_PRIO_SD 7     // SDMMC1 (DMA completion only)
