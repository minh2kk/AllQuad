// AllQuad - microSD card on SDMMC1 (STM32H7 boards with the slot on the PCB) with FatFs.
// 4-bit bus. Default: polled transfers - the card's hardware flow control stops the bus clock whenever the
// FIFO is full / empty, so a flight task preempting the low-priority "sdcard" task never causes an
// over/underrun. sd_dma 1: the SDMMC's own DMA (IDMA) moves the blocks through a 32-byte aligned bounce
// buffer (cache cleaned / invalidated around it) while the task sleeps; any DMA error or timeout aborts
// the transfer, repeats it polled and keeps polling until the next mount. Other STM32 boards: stubs.
#if defined(AQ_STM32) && !defined(ALLQUAD_HOST)
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "../board.h"
#include "../hal.h"
#include "board_stm32.h"
#include "stm32_hal.h"

#if defined(AQ_HAS_SDMMC) && defined(STM32H7xx)
#include "diskio.h"
#include "ff.h"

namespace {
SD_HandleTypeDef hsd;
bool cardUp = false;   // card initialised (disk layer)
bool mounted = false;  // FAT volume mounted
FATFS fs;
FIL fw, fr;
bool fwOpen = false, frOpen = false;
char frPath[40] = "";

bool rtos() { return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED; }
void yield1() {
  if (rtos() && __get_IPSR() == 0) vTaskDelay(1);
}
// the card is busy programming after a write: up to ~250 ms
bool waitTransfer(uint32_t timeoutMs) {
  uint32_t t0 = HAL_GetTick();
  while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
    if (HAL_GetTick() - t0 > timeoutMs) return false;
    yield1();
  }
  return true;
}

uint8_t clockDiv = 2;  // SDMMC_CK = 100 MHz / (2 x div): 25 MHz

// ---- DMA (IDMA) transfers
bool dmaWanted = false, dmaOn = false;
volatile int dmaState = 0;  // 1 running, 2 done, -1 error
constexpr UINT DMA_BLOCKS = 8;
alignas(32) uint8_t dmaBuf[DMA_BLOCKS * 512];

bool dmaWait(uint32_t timeoutMs) {
  uint32_t t0 = HAL_GetTick();
  while (dmaState == 1) {
    if (HAL_GetTick() - t0 > timeoutMs) return false;
    yield1();
  }
  return dmaState == 2;
}
void dmaFail() {
  HAL_SD_Abort(&hsd);
  dmaOn = false;  // polled from now on (until the next mount)
}
// one chunk (<= DMA_BLOCKS) by DMA; false = do it polled
bool dmaRead(BYTE* buf, uint32_t sector, UINT n) {
  dmaState = 1;
  if (HAL_SD_ReadBlocks_DMA(&hsd, dmaBuf, sector, n) != HAL_OK) { dmaState = 0; dmaFail(); return false; }
  if (!dmaWait(1000)) { dmaFail(); return false; }
  SCB_InvalidateDCache_by_Addr((uint32_t*)dmaBuf, (int32_t)(n * 512));
  memcpy(buf, dmaBuf, n * 512);
  return true;
}
bool dmaWrite(const BYTE* buf, uint32_t sector, UINT n) {
  memcpy(dmaBuf, buf, n * 512);
  SCB_CleanDCache_by_Addr((uint32_t*)dmaBuf, (int32_t)(n * 512));
  dmaState = 1;
  if (HAL_SD_WriteBlocks_DMA(&hsd, dmaBuf, sector, n) != HAL_OK) { dmaState = 0; dmaFail(); return false; }
  if (!dmaWait(1500)) { dmaFail(); return false; }
  return true;
}

bool cardInit() {
  // PC8-PC11 = D0-D3, PC12 = CK, PD2 = CMD (AF12); pull-ups on CMD / data
  static const int dataPins[] = {PC8, PC9, PC10, PC11};
  for (int p : dataPins) gpio::config(p, gpio::AF, gpio::UP, false, 3, 12);
  gpio::config(PC12, gpio::AF, gpio::NOPULL, false, 3, 12);
  gpio::config(PD2, gpio::AF, gpio::UP, false, 3, 12);
  __HAL_RCC_SDMMC1_CLK_ENABLE();
  __HAL_RCC_SDMMC1_FORCE_RESET();
  __HAL_RCC_SDMMC1_RELEASE_RESET();
  memset(&hsd, 0, sizeof(hsd));
  hsd.Instance = SDMMC1;
  hsd.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd.Init.ClockDiv = clockDiv;
  if (HAL_SD_Init(&hsd) != HAL_OK) {
    // some cards / sockets do not manage 4-bit at once: retry 1-bit
    HAL_SD_DeInit(&hsd);
    hsd.Init.BusWide = SDMMC_BUS_WIDE_1B;
    if (HAL_SD_Init(&hsd) != HAL_OK) return false;
  }
  return true;
}
}  // namespace

// ---------------- FatFs disk layer ----------------
extern "C" {
DSTATUS disk_initialize(BYTE pdrv) {
  if (pdrv) return STA_NOINIT;
  if (!cardUp) cardUp = cardInit();
  return cardUp ? 0 : STA_NOINIT;
}
DSTATUS disk_status(BYTE pdrv) { return (pdrv || !cardUp) ? STA_NOINIT : 0; }
DRESULT disk_read(BYTE pdrv, BYTE* buf, LBA_t sector, UINT count) {
  if (pdrv || !cardUp) return RES_NOTRDY;
  while (dmaOn && count) {
    UINT n = count < DMA_BLOCKS ? count : DMA_BLOCKS;
    if (!waitTransfer(500)) return RES_ERROR;
    if (!dmaRead(buf, (uint32_t)sector, n)) break;  // the rest goes polled
    buf += n * 512; sector += n; count -= n;
  }
  if (!count) return waitTransfer(500) ? RES_OK : RES_ERROR;
  if (!waitTransfer(500)) return RES_ERROR;
  if (HAL_SD_ReadBlocks(&hsd, buf, (uint32_t)sector, count, 1000) != HAL_OK) return RES_ERROR;
  return waitTransfer(500) ? RES_OK : RES_ERROR;
}
DRESULT disk_write(BYTE pdrv, const BYTE* buf, LBA_t sector, UINT count) {
  if (pdrv || !cardUp) return RES_NOTRDY;
  while (dmaOn && count) {
    UINT n = count < DMA_BLOCKS ? count : DMA_BLOCKS;
    if (!waitTransfer(1000)) return RES_ERROR;
    if (!dmaWrite(buf, (uint32_t)sector, n)) break;
    buf += n * 512; sector += n; count -= n;
  }
  if (!count) return waitTransfer(1000) ? RES_OK : RES_ERROR;
  if (!waitTransfer(1000)) return RES_ERROR;
  if (HAL_SD_WriteBlocks(&hsd, (uint8_t*)buf, (uint32_t)sector, count, 1000) != HAL_OK) return RES_ERROR;
  return waitTransfer(1000) ? RES_OK : RES_ERROR;
}
// SDMMC1 interrupt (DMA transfers only)
void SDMMC1_IRQHandler(void) { HAL_SD_IRQHandler(&hsd); }
void HAL_SD_TxCpltCallback(SD_HandleTypeDef*) { dmaState = 2; }
void HAL_SD_RxCpltCallback(SD_HandleTypeDef*) { dmaState = 2; }
void HAL_SD_ErrorCallback(SD_HandleTypeDef*) { dmaState = -1; }
void HAL_SD_AbortCallback(SD_HandleTypeDef*) { if (dmaState == 1) dmaState = -1; }
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buf) {
  if (pdrv || !cardUp) return RES_NOTRDY;
  HAL_SD_CardInfoTypeDef ci;
  switch (cmd) {
    case CTRL_SYNC: return waitTransfer(1000) ? RES_OK : RES_ERROR;
    case GET_SECTOR_COUNT: HAL_SD_GetCardInfo(&hsd, &ci); *(LBA_t*)buf = ci.LogBlockNbr; return RES_OK;
    case GET_SECTOR_SIZE: *(WORD*)buf = 512; return RES_OK;
    case GET_BLOCK_SIZE: *(DWORD*)buf = 1; return RES_OK;
    default: return RES_PARERR;
  }
}
}

namespace hal {
void sdSetDma(bool on) { dmaWanted = on; }
bool sdDmaActive() { return dmaOn; }
uint8_t sdModes() { return 1 << 3; }  // SD bus 4-bit only (fixed wiring)
bool sdPinsFixed() { return true; }
void sdDefaultPins(uint8_t p[6]) {
  for (int i = 0; i < 6; i++) p[i] = 255;
}
bool sdMount(uint8_t, uint8_t mhz, const uint8_t*, char* err, size_t n) {
  sdUnmount();
  uint32_t m = mhz ? mhz : 25;
  if (m > 50) m = 50;
  clockDiv = (uint8_t)((100 + 2 * m - 1) / (2 * m));
  if (!clockDiv) clockDiv = 1;
  cardUp = false;
  if (disk_initialize(0)) { snprintf(err, n, "no card (SDMMC1)"); return false; }
  // DMA from the start (the FAT mount reads go through it too); polled again after any DMA error
  dmaOn = dmaWanted;
  if (dmaOn) {
    HAL_NVIC_SetPriority(SDMMC1_IRQn, AQ_PRIO_SD, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  } else {
    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
  }
  FRESULT r = f_mount(&fs, "", 1);
  if (r != FR_OK) {
    snprintf(err, n, r == FR_NO_FILESYSTEM ? "card not FAT formatted" : "mount failed (%d)", (int)r);
    HAL_SD_DeInit(&hsd);
    cardUp = false;
    return false;
  }
  mounted = true;
  return true;
}
void sdUnmount() {
  dmaOn = false;
  if (fwOpen) { f_close(&fw); fwOpen = false; }
  sdReadClose();
  if (mounted) f_mount(nullptr, "", 0);
  mounted = false;
  if (cardUp) HAL_SD_DeInit(&hsd);
  cardUp = false;
}
bool sdMounted() { return mounted; }
const char* sdCardType() {
  if (!mounted) return "";
  HAL_SD_CardInfoTypeDef ci;
  HAL_SD_GetCardInfo(&hsd, &ci);
  return ci.CardType == CARD_SDHC_SDXC ? "SDHC" : "SDSC";
}
bool sdSpace(uint64_t& total, uint64_t& used) {
  if (!mounted) return false;
  DWORD fre = 0;
  FATFS* f = nullptr;
  if (f_getfree("", &fre, &f) != FR_OK) return false;
  uint64_t cl = (uint64_t)f->csize * 512u;
  total = (uint64_t)(f->n_fatent - 2) * cl;
  used = total - (uint64_t)fre * cl;
  return true;
}
uint8_t* sdQueueAlloc(size_t want, size_t& got) {
  static uint8_t q[64 * 1024] __attribute__((aligned(32)));
  got = want < sizeof(q) ? want : sizeof(q);
  return q;
}
bool sdCreate(const char* path) {
  if (!mounted) return false;
  char dir[40];
  snprintf(dir, sizeof(dir), "%s", path);
  char* slash = strrchr(dir, '/');
  if (slash && slash != dir) {
    *slash = 0;
    f_mkdir(dir);  // FR_EXIST when it is already there
  }
  fwOpen = f_open(&fw, path, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK;
  return fwOpen;
}
size_t sdAppend(const uint8_t* d, size_t n) {
  if (!fwOpen) return 0;
  UINT w = 0;
  if (f_write(&fw, d, (UINT)n, &w) != FR_OK) return 0;
  return w;
}
bool sdSync() { return fwOpen && f_sync(&fw) == FR_OK; }
void sdClose() {
  if (fwOpen) f_close(&fw);
  fwOpen = false;
}
void sdList(const char* dir, void (*cb)(const char*, uint32_t, void*), void* ctx) {
  if (!mounted) return;
  DIR d;
  FILINFO fi;
  if (f_opendir(&d, dir) != FR_OK) return;
  while (f_readdir(&d, &fi) == FR_OK && fi.fname[0]) {
    if (!(fi.fattrib & AM_DIR)) cb(fi.fname, (uint32_t)fi.fsize, ctx);
  }
  f_closedir(&d);
}
bool sdRemove(const char* path) { return mounted && f_unlink(path) == FR_OK; }
bool sdRead(const char* path, uint32_t off, uint8_t* buf, size_t len, size_t& got, uint32_t& size) {
  got = 0;
  if (!mounted) return false;
  if (!frOpen || strcmp(path, frPath)) {
    sdReadClose();
    if (f_open(&fr, path, FA_READ) != FR_OK) return false;
    frOpen = true;
    snprintf(frPath, sizeof(frPath), "%s", path);
  }
  size = (uint32_t)f_size(&fr);
  if (off >= size) return true;
  if (f_tell(&fr) != off && f_lseek(&fr, off) != FR_OK) return false;
  UINT r = 0;
  if (f_read(&fr, buf, (UINT)len, &r) != FR_OK) return false;
  got = r;
  return true;
}
void sdReadClose() {
  if (frOpen) f_close(&fr);
  frOpen = false;
  frPath[0] = 0;
}
}  // namespace hal

#else
namespace hal {
uint8_t sdModes() { return 0; }
bool sdPinsFixed() { return true; }
void sdDefaultPins(uint8_t p[6]) {
  for (int i = 0; i < 6; i++) p[i] = 255;
}
bool sdMount(uint8_t, uint8_t, const uint8_t*, char* err, size_t n) {
  snprintf(err, n, "no SD card support on this board");
  return false;
}
void sdSetDma(bool) {}
bool sdDmaActive() { return false; }
void sdUnmount() {}
bool sdMounted() { return false; }
const char* sdCardType() { return ""; }
bool sdSpace(uint64_t&, uint64_t&) { return false; }
uint8_t* sdQueueAlloc(size_t, size_t& got) {
  got = 0;
  return nullptr;
}
bool sdCreate(const char*) { return false; }
size_t sdAppend(const uint8_t*, size_t) { return 0; }
bool sdSync() { return false; }
void sdClose() {}
void sdList(const char*, void (*)(const char*, uint32_t, void*), void*) {}
bool sdRemove(const char*) { return false; }
bool sdRead(const char*, uint32_t, uint8_t*, size_t, size_t& got, uint32_t& size) {
  got = 0;
  size = 0;
  return false;
}
void sdReadClose() {}
}  // namespace hal
#endif
#endif
