/* AllQuad - ST USB device library configuration: one CDC (virtual COM port) device, full speed,
   static class data (no malloc inside the USB interrupt). */
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if defined(STM32H7xx)
#include "stm32h7xx_hal.h"
#elif defined(STM32F7xx)
#include "stm32f7xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif

#define USBD_MAX_NUM_INTERFACES 2U
#define USBD_MAX_NUM_CONFIGURATION 1U
#define USBD_MAX_STR_DESC_SIZ 64U
#define USBD_DEBUG_LEVEL 0U
#define USBD_LPM_ENABLED 0U
#define USBD_SELF_POWERED 1U
#define USBD_SUPPORT_USER_STRING_DESC 0U
#define USBD_CLASS_USER_STRING_DESC 0U
#define USBD_CLASS_BOS_ENABLED 0U
#define USBD_CDC_INTERVAL 1000U
#define CDC_FS_BINTERVAL 0x10U
#define DEVICE_FS 0

void* USBD_static_malloc(uint32_t size);
void USBD_static_free(void* p);
#define USBD_malloc (void*)USBD_static_malloc
#define USBD_free USBD_static_free
#define USBD_memset memset
#define USBD_memcpy memcpy
#define USBD_Delay HAL_Delay

#define USBD_UsrLog(...) do {} while (0)
#define USBD_ErrLog(...) do {} while (0)
#define USBD_DbgLog(...) do {} while (0)
