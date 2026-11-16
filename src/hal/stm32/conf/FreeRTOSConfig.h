// AllQuad - FreeRTOS configuration for the STM32 builds (STM32Cube HAL, no Arduino)
// Cortex-M4F / M7 with 4 NVIC priority bits. Interrupts that call FreeRTOS (TIM loop timer, IMU data-ready
// EXTI) run at priority >= 5; UARTs (1), USB (6) and DMA never call it.
#pragma once
#include <stdint.h>

extern uint32_t SystemCoreClock;

#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE 0
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ 1000
#define configMAX_PRIORITIES 10
#define configMINIMAL_STACK_SIZE 160
#define configMAX_TASK_NAME_LEN 12
#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD 1
#define configUSE_TASK_NOTIFICATIONS 1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 1
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 0
#define configUSE_COUNTING_SEMAPHORES 1
#define configQUEUE_REGISTRY_SIZE 0
#define configUSE_QUEUE_SETS 0
#define configENABLE_BACKWARD_COMPATIBILITY 1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0
#define configUSE_NEWLIB_REENTRANT 1  // snprintf / float formatting from several tasks
#define configSTACK_DEPTH_TYPE uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE size_t

// memory: heap_4 (lib/freertos) also serves malloc / new (src/hal/stm32/system_stm32.cpp)
#define configSUPPORT_STATIC_ALLOCATION 0
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#if defined(STM32H7xx)
#define configTOTAL_HEAP_SIZE (192 * 1024)
#elif defined(STM32F722xx)
#define configTOTAL_HEAP_SIZE (112 * 1024)
#elif defined(STM32F405xx)
#define configTOTAL_HEAP_SIZE (66 * 1024)
#else  // F411: 128 kB RAM
#define configTOTAL_HEAP_SIZE (66 * 1024)
#endif
#define configAPPLICATION_ALLOCATED_HEAP 0

// hooks
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK 1
#define configUSE_DAEMON_TASK_STARTUP_HOOK 0
#define configGENERATE_RUN_TIME_STATS 0
#define configUSE_TRACE_FACILITY 0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0
#define configUSE_CO_ROUTINES 0

// software timers (not used by the firmware, kept small)
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY 2
#define configTIMER_QUEUE_LENGTH 4
#define configTIMER_TASK_STACK_DEPTH 256

// interrupt priorities (4 bits on STM32)
#define configPRIO_BITS 4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

// API
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_xResumeFromISR 1
#define INCLUDE_xTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetIdleTaskHandle 0
#define INCLUDE_eTaskGetState 0
#define INCLUDE_xTimerPendFunctionCall 0

#ifdef __cplusplus
extern "C" {
#endif
void aqAssertFailed(const char* file, int line);
#ifdef __cplusplus
}
#endif
#if defined(AQ_QEMU)
// QEMU 8 models 8 NVIC priority bits instead of 4: the port's interrupt-priority check cannot pass there
#define configASSERT(x) ((void)0)
#else
#define configASSERT(x) \
  if (!(x)) aqAssertFailed(__FILE__, __LINE__)
#endif

// the port's handlers are the vector table entries
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler
