/* AllQuad - USB virtual COM port (CDC ACM) on the STM32 OTG FS peripheral (ST USB device library).
   Console / CLI / configurator / MAVLink. Ring buffers in both directions:
     * received packets are copied into a 2 kB ring in the USB interrupt; when less than one packet of
       space is left the endpoint is not re-armed (the host waits: no byte is ever dropped) until
       aqUsbRead() made room again
     * aqUsbWrite() queues into a 4 kB ring; the interrupt keeps sending while data is queued, a write
       starts the transfer when the endpoint is idle
   The USB interrupt (priority 6) never calls FreeRTOS. Tasks synchronise with it by masking that one
   interrupt for a few instructions. */
#if defined(AQ_STM32) && !defined(ALLQUAD_HOST)
#include <string.h>

#include "stm32_hal.h"
#include "board_stm32.h"
#include "usbd_cdc.h"
#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_def.h"

#define USB_VID 0x0483u
#define USB_PID 0x5740u

static PCD_HandleTypeDef hpcd;
static USBD_HandleTypeDef hdev;

/* ------------------------------------------------------------------ rings */
#if defined(STM32H7xx)
#define RX_SIZE 2048u
#define TX_SIZE 4096u
#else
#define RX_SIZE 1024u
#define TX_SIZE 2048u
#endif
static uint8_t rxRing[RX_SIZE];
static volatile uint32_t rxHead, rxTail;  /* head: ISR writes, tail: reader */
static uint8_t txRing[TX_SIZE];
static volatile uint32_t txHead, txTail;  /* head: writers, tail: ISR (after a packet went out) */
static volatile uint32_t txInFlight;      /* bytes of the running transfer */
static volatile uint8_t rxPaused, configured, dtr;
static uint8_t rxPacket[CDC_DATA_FS_MAX_PACKET_SIZE] __attribute__((aligned(4)));
static uint8_t txPacket[256] __attribute__((aligned(4)));

static inline void usbMask(void) { NVIC_DisableIRQ(OTG_FS_IRQn); __DSB(); __ISB(); }
static inline void usbUnmask(void) { NVIC_EnableIRQ(OTG_FS_IRQn); }

/* static class data for the CDC handle */
void* USBD_static_malloc(uint32_t size) {
  static uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef) + 3) / 4];
  return size <= sizeof(mem) ? mem : NULL;
}
void USBD_static_free(void* p) { (void)p; }

/* start the next transfer if the endpoint is idle and data is queued (USB interrupt context or masked) */
static void txKick(void) {
  if (txInFlight || !configured) return;
  uint32_t h = txHead, t = txTail;
  uint32_t n = (h - t) % TX_SIZE;
  if (!n) return;
  if (n > sizeof(txPacket)) n = sizeof(txPacket);
  for (uint32_t i = 0; i < n; i++) txPacket[i] = txRing[(t + i) % TX_SIZE];
  txInFlight = n;
  USBD_CDC_SetTxBuffer(&hdev, txPacket, (uint32_t)n);
  if (USBD_CDC_TransmitPacket(&hdev) != USBD_OK) txInFlight = 0;
}

/* ------------------------------------------------------------------ CDC interface */
static USBD_CDC_LineCodingTypeDef lineCoding = {115200, 0, 0, 8};

static int8_t cdcInit(void) {
  rxHead = rxTail = 0;
  txTail = txHead;
  txInFlight = 0;
  rxPaused = 0;
  configured = 1;
  USBD_CDC_SetRxBuffer(&hdev, rxPacket);
  USBD_CDC_ReceivePacket(&hdev);
  return USBD_OK;
}
static int8_t cdcDeInit(void) {
  configured = 0;
  dtr = 0;
  txInFlight = 0;
  return USBD_OK;
}
static int8_t cdcControl(uint8_t cmd, uint8_t* buf, uint16_t len) {
  (void)len;
  switch (cmd) {
    case CDC_SET_LINE_CODING:
      lineCoding.bitrate = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
      lineCoding.format = buf[4];
      lineCoding.paritytype = buf[5];
      lineCoding.datatype = buf[6];
      break;
    case CDC_GET_LINE_CODING:
      buf[0] = (uint8_t)lineCoding.bitrate;
      buf[1] = (uint8_t)(lineCoding.bitrate >> 8);
      buf[2] = (uint8_t)(lineCoding.bitrate >> 16);
      buf[3] = (uint8_t)(lineCoding.bitrate >> 24);
      buf[4] = lineCoding.format;
      buf[5] = lineCoding.paritytype;
      buf[6] = lineCoding.datatype;
      break;
    case CDC_SET_CONTROL_LINE_STATE: {
      USBD_SetupReqTypedef* req = (USBD_SetupReqTypedef*)buf;
      dtr = (req->wValue & 1u) ? 1 : 0;
      break;
    }
    default:
      break;
  }
  return USBD_OK;
}
static int8_t cdcReceive(uint8_t* buf, uint32_t* len) {
  uint32_t n = *len, h = rxHead;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t next = (h + 1) % RX_SIZE;
    if (next == rxTail) break; /* cannot happen: re-armed only with a packet of space */
    rxRing[h] = buf[i];
    h = next;
  }
  rxHead = h;
  uint32_t space = (rxTail - h - 1) % RX_SIZE;
  if (space >= CDC_DATA_FS_MAX_PACKET_SIZE) {
    USBD_CDC_SetRxBuffer(&hdev, rxPacket);
    USBD_CDC_ReceivePacket(&hdev);
  } else {
    rxPaused = 1;
  }
  return USBD_OK;
}
static int8_t cdcTxDone(uint8_t* buf, uint32_t* len, uint8_t ep) {
  (void)buf; (void)len; (void)ep;
  txTail = (txTail + txInFlight) % TX_SIZE;
  txInFlight = 0;
  txKick();
  return USBD_OK;
}
static USBD_CDC_ItfTypeDef cdcItf = {cdcInit, cdcDeInit, cdcControl, cdcReceive, cdcTxDone};

/* ------------------------------------------------------------------ public */
int aqUsbRead(void) {
  uint32_t t = rxTail;
  if (t == rxHead) return -1;
  int c = rxRing[t];
  rxTail = (t + 1) % RX_SIZE;
  if (rxPaused) {
    uint32_t space = (rxTail - rxHead - 1) % RX_SIZE;
    if (space >= CDC_DATA_FS_MAX_PACKET_SIZE) {
      usbMask();
      if (rxPaused && configured) {
        rxPaused = 0;
        USBD_CDC_SetRxBuffer(&hdev, rxPacket);
        USBD_CDC_ReceivePacket(&hdev);
      }
      usbUnmask();
    }
  }
  return c;
}
size_t aqUsbWriteSpace(void) {
  if (!configured) return TX_SIZE - 1;
  return (txTail - txHead - 1) % TX_SIZE;
}
size_t aqUsbWrite(const uint8_t* d, size_t n) {
  if (!configured) return n; /* nobody listening: drop, never block the caller */
  /* several tasks may write (CLI, MAVLink, log messages): the ring is filled with the interrupt masked */
  usbMask();
  uint32_t h = txHead;
  uint32_t space = (txTail - h - 1) % TX_SIZE;
  if (n > space) n = space;
  for (size_t i = 0; i < n; i++) { txRing[h] = d[i]; h = (h + 1) % TX_SIZE; }
  txHead = h;
  txKick();
  usbUnmask();
  return n;
}
int aqUsbConnected(void) { return configured && dtr; }

/* ------------------------------------------------------------------ descriptors */
static uint8_t devDesc[USB_LEN_DEV_DESC] __attribute__((aligned(4))) = {
    0x12, USB_DESC_TYPE_DEVICE, 0x00, 0x02, 0x02, 0x02, 0x00, USB_MAX_EP0_SIZE,
    (uint8_t)USB_VID, (uint8_t)(USB_VID >> 8), (uint8_t)USB_PID, (uint8_t)(USB_PID >> 8),
    0x00, 0x02, USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR, USBD_IDX_SERIAL_STR, USBD_MAX_NUM_CONFIGURATION};
static uint8_t langDesc[USB_LEN_LANGID_STR_DESC] __attribute__((aligned(4))) = {
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING, 0x09, 0x04};
static uint8_t strDesc[USBD_MAX_STR_DESC_SIZ] __attribute__((aligned(4)));

static uint8_t* descDevice(USBD_SpeedTypeDef s, uint16_t* len) { (void)s; *len = sizeof(devDesc); return devDesc; }
static uint8_t* descLang(USBD_SpeedTypeDef s, uint16_t* len) { (void)s; *len = sizeof(langDesc); return langDesc; }
static uint8_t* descString(const char* str, uint16_t* len) {
  USBD_GetString((uint8_t*)str, strDesc, len);
  return strDesc;
}
static uint8_t* descMfc(USBD_SpeedTypeDef s, uint16_t* len) { (void)s; return descString("AllQuad", len); }
static uint8_t* descProduct(USBD_SpeedTypeDef s, uint16_t* len) { (void)s; return descString("AllQuad Flight Controller", len); }
static uint8_t* descSerial(USBD_SpeedTypeDef s, uint16_t* len) {
  (void)s;
  static const char hex[] = "0123456789ABCDEF";
  char sn[13];
  uint32_t a = HAL_GetUIDw0() + HAL_GetUIDw2(), b = HAL_GetUIDw1();
  for (int i = 0; i < 8; i++) sn[i] = hex[(a >> (28 - 4 * i)) & 15];
  for (int i = 0; i < 4; i++) sn[8 + i] = hex[(b >> (28 - 4 * i)) & 15];
  sn[12] = 0;
  return descString(sn, len);
}
static uint8_t* descConfig(USBD_SpeedTypeDef s, uint16_t* len) { (void)s; return descString("CDC Config", len); }
static uint8_t* descItf(USBD_SpeedTypeDef s, uint16_t* len) { (void)s; return descString("CDC Interface", len); }
static USBD_DescriptorsTypeDef descriptors = {descDevice, descLang, descMfc, descProduct, descSerial, descConfig, descItf};

/* ------------------------------------------------------------------ start */
void aqUsbInit(void) {
  /* PA11 (DM) / PA12 (DP), AF10 on every family */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = 10;
  HAL_GPIO_Init(GPIOA, &g);
  USBD_Init(&hdev, &descriptors, DEVICE_FS);
  USBD_RegisterClass(&hdev, &USBD_CDC);
  USBD_CDC_RegisterInterface(&hdev, &cdcItf);
  USBD_Start(&hdev);
}

/* ------------------------------------------------------------------ low level (PCD <-> USB device library) */
void OTG_FS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd); }

void HAL_PCD_MspInit(PCD_HandleTypeDef* h) {
  (void)h;
#if defined(STM32H7xx)
  __HAL_RCC_USB2_OTG_FS_CLK_ENABLE();
#else
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
#endif
  HAL_NVIC_SetPriority(OTG_FS_IRQn, AQ_PRIO_USB, 0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef* pdev) {
  memset(&hpcd, 0, sizeof(hpcd));
  hpcd.pData = pdev;
  pdev->pData = &hpcd;
  hpcd.Instance = USB_OTG_FS;
  hpcd.Init.dev_endpoints = 4;
  hpcd.Init.speed = PCD_SPEED_FULL;
  hpcd.Init.dma_enable = DISABLE;
  hpcd.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd.Init.Sof_enable = DISABLE;
  hpcd.Init.low_power_enable = DISABLE;
  hpcd.Init.lpm_enable = DISABLE;
  hpcd.Init.vbus_sensing_enable = DISABLE; /* the boards do not wire VBUS to PA9 (it is a UART pin) */
  hpcd.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd) != HAL_OK) return USBD_FAIL;
  /* 320 words of FIFO RAM (1.25 kB): RX 128, EP0 IN 64, EP1 IN (data) 64, EP2 IN (notifications) 16 */
  HAL_PCDEx_SetRxFiFo(&hpcd, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd, 1, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd, 2, 0x10);
  return USBD_OK;
}

static USBD_StatusTypeDef st(HAL_StatusTypeDef s) {
  return s == HAL_OK ? USBD_OK : s == HAL_BUSY ? USBD_BUSY : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef* p) { return st(HAL_PCD_DeInit((PCD_HandleTypeDef*)p->pData)); }
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef* p) { return st(HAL_PCD_Start((PCD_HandleTypeDef*)p->pData)); }
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef* p) { return st(HAL_PCD_Stop((PCD_HandleTypeDef*)p->pData)); }
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef* p, uint8_t ep, uint8_t type, uint16_t mps) {
  return st(HAL_PCD_EP_Open((PCD_HandleTypeDef*)p->pData, ep, mps, type));
}
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef* p, uint8_t ep) { return st(HAL_PCD_EP_Close((PCD_HandleTypeDef*)p->pData, ep)); }
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef* p, uint8_t ep) { return st(HAL_PCD_EP_Flush((PCD_HandleTypeDef*)p->pData, ep)); }
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef* p, uint8_t ep) { return st(HAL_PCD_EP_SetStall((PCD_HandleTypeDef*)p->pData, ep)); }
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef* p, uint8_t ep) { return st(HAL_PCD_EP_ClrStall((PCD_HandleTypeDef*)p->pData, ep)); }
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef* p, uint8_t ep) {
  PCD_HandleTypeDef* h = (PCD_HandleTypeDef*)p->pData;
  return (ep & 0x80) ? h->IN_ep[ep & 0x7F].is_stall : h->OUT_ep[ep & 0x7F].is_stall;
}
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef* p, uint8_t a) { return st(HAL_PCD_SetAddress((PCD_HandleTypeDef*)p->pData, a)); }
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef* p, uint8_t ep, uint8_t* buf, uint32_t n) {
  return st(HAL_PCD_EP_Transmit((PCD_HandleTypeDef*)p->pData, ep, buf, n));
}
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef* p, uint8_t ep, uint8_t* buf, uint32_t n) {
  return st(HAL_PCD_EP_Receive((PCD_HandleTypeDef*)p->pData, ep, buf, n));
}
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef* p, uint8_t ep) { return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef*)p->pData, ep); }
void USBD_LL_Delay(uint32_t ms) { HAL_Delay(ms); }

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef* h) { USBD_LL_SetupStage((USBD_HandleTypeDef*)h->pData, (uint8_t*)h->Setup); }
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef* h, uint8_t ep) {
  USBD_LL_DataOutStage((USBD_HandleTypeDef*)h->pData, ep, h->OUT_ep[ep].xfer_buff);
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef* h, uint8_t ep) {
  USBD_LL_DataInStage((USBD_HandleTypeDef*)h->pData, ep, h->IN_ep[ep].xfer_buff);
}
void HAL_PCD_SOFCallback(PCD_HandleTypeDef* h) { USBD_LL_SOF((USBD_HandleTypeDef*)h->pData); }
void HAL_PCD_ResetCallback(PCD_HandleTypeDef* h) {
  USBD_LL_SetSpeed((USBD_HandleTypeDef*)h->pData, USBD_SPEED_FULL);
  USBD_LL_Reset((USBD_HandleTypeDef*)h->pData);
  configured = 0;
  dtr = 0;
  txInFlight = 0;
}
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef* h) { USBD_LL_Suspend((USBD_HandleTypeDef*)h->pData); }
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef* h) { USBD_LL_Resume((USBD_HandleTypeDef*)h->pData); }
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef* h, uint8_t ep) { USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef*)h->pData, ep); }
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef* h, uint8_t ep) { USBD_LL_IsoINIncomplete((USBD_HandleTypeDef*)h->pData, ep); }
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef* h) { USBD_LL_DevConnected((USBD_HandleTypeDef*)h->pData); }
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef* h) {
  USBD_LL_DevDisconnected((USBD_HandleTypeDef*)h->pData);
  configured = 0;
  dtr = 0;
}
#endif
