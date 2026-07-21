#include <stdint.h>
#include <string.h>

uint32_t Cy_SysLib_EnterCriticalSection(void) {
  return 0;
}

void Cy_SysLib_ExitCriticalSection(uint32_t savedIntrStatus) {
  (void)savedIntrStatus;
}

__attribute__((weak)) void Cy_SysLib_DelayCycles(uint32_t cycles) {
  for (volatile uint32_t i = cycles / 12; i; i--)
    continue;
}

#include "../sdl/common/src/drivers/gpio/cy_gpio.c"
#include "../sdl/common/src/drivers/ipc/cy_ipc_drv.c"
#include "../sdl/common/src/drivers/srom/cy_srom.c"
#include "../sdl/common/src/drivers/sysint/cy_sysint.c"
#include "../sdl/common/src/drivers/syslib/cy_syslib.c"
#include "../sdl/common/src/drivers/systick/cy_systick.c"
#include "../sdl/common/src/drivers/syswdt/cy_syswdt.c"
#include "../sdl/tviic2d6mddr/src/drivers/sd_host/cy_sd_host.c"
#include "../sdl/tviic2d6mddr/src/drivers/sysclk/cy_sysclk.c"
#include "../sdl/tviic2d6mddr/src/drivers/syspmic/cy_syspmic.c"
#include "../sdl/tviic2d6mddr/src/mw/power/cy_power.c"
#include "../sdl/tviic2d6mddr/src/system/rev_a/system_tviic2d6mddr_cm0plus.c"

#undef CY_ASSERT

#define ASSERT(x)           \
  if (!(x)) {               \
    __asm volatile(         \
        "mov    r0, %0  \n" \
        "bkpt   #0        " \
        : /* No outputs */  \
        : "r"(__LINE__)     \
        : "memory");        \
  }

void wait_complete(cy_stc_sd_host_context_t const *context) {
  while (1) {
    uint32_t cardStatus = Cy_SD_Host_GetCardStatus((stc_SDHC_t *)CY_SDHC_TYPE, context);
    if (((CY_SD_HOST_CARD_TRAN << CY_SD_HOST_CMD13_CURRENT_STATE) | (1u << CY_SD_HOST_CMD13_READY_FOR_DATA)) ==
        cardStatus)
      break;
  }
}

static cy_stc_gpio_pin_config_t sdhc_port_pin_cfg = {
    .outVal   = 0x00,
    .intEdge  = 0,
    .intMask  = 0,
    .vtrip    = 0,
    .slewRate = 0,
    .driveSel = 0,
    .vregEn   = 0,
    .ibufMode = 0,
    .vtripSel = 0,
    .vrefSel  = 0,
    .vohSel   = 0,
};

static uint32_t rca = 3u;
static cy_en_sd_host_card_capacity_t cardCapacity;
static cy_stc_sd_host_context_t sd_host_context;
static cy_en_sd_host_status_t enRet = CY_SD_HOST_SUCCESS;

cy_stc_sd_host_init_config_t host_cfg = {
    .dmaType          = CY_SD_HOST_DMA_SDMA,
    .enableLedControl = false,
    .emmc             = true,
};
cy_en_sd_host_card_type_t card_type = CY_SD_HOST_EMMC;

cy_stc_sd_host_sd_card_config_t stc_card_cfg = {
    .lowVoltageSignaling = false,
    .busWidth            = CY_SD_HOST_BUS_WIDTH_4_BIT,
    .cardType            = &card_type,
    .rca                 = &rca,
    .cardCapacity        = &cardCapacity,
};

__attribute__((used)) int Init(unsigned long adr, unsigned long clk, unsigned long fnc) {
  (void)adr;
  (void)clk;

  stc_card_cfg.cardType     = &card_type;
  stc_card_cfg.rca          = &rca;
  stc_card_cfg.cardCapacity = &cardCapacity;

  SystemInit();

  Cy_SystemHsioPowerConfig();
  Cy_SystemHsioEnhPowerUp();

  Cy_SysClk_HfClkEnable(CY_SYSCLK_HFCLK_13);  // CLK_HF13 enable
  Cy_SD_Host_Enable((volatile stc_SDHC_t *)CY_SDHC_TYPE);

  sdhc_port_pin_cfg.driveMode = CY_GPIO_DM_STRONG;
  sdhc_port_pin_cfg.hsiom     = CY_SDHC_CARD_CMD_PIN_MUX;
  Cy_GPIO_Pin_Init(CY_SDHC_CARD_CMD_PORT, CY_SDHC_CARD_CMD_PIN, &sdhc_port_pin_cfg);
  sdhc_port_pin_cfg.hsiom = CY_SDHC_CLK_CARD_PIN_MUX;
  Cy_GPIO_Pin_Init(CY_SDHC_CLK_CARD_PORT, CY_SDHC_CLK_CARD_PIN, &sdhc_port_pin_cfg);
  sdhc_port_pin_cfg.hsiom = CY_SDHC_CARD_DAT_3TO00_PIN_MUX;
  Cy_GPIO_Pin_Init(CY_SDHC_CARD_DAT_3TO00_PORT, CY_SDHC_CARD_DAT_3TO00_PIN, &sdhc_port_pin_cfg);
  sdhc_port_pin_cfg.hsiom = CY_SDHC_CARD_DAT_3TO01_PIN_MUX;
  Cy_GPIO_Pin_Init(CY_SDHC_CARD_DAT_3TO01_PORT, CY_SDHC_CARD_DAT_3TO01_PIN, &sdhc_port_pin_cfg);
  sdhc_port_pin_cfg.hsiom = CY_SDHC_CARD_DAT_3TO02_PIN_MUX;
  Cy_GPIO_Pin_Init(CY_SDHC_CARD_DAT_3TO02_PORT, CY_SDHC_CARD_DAT_3TO02_PIN, &sdhc_port_pin_cfg);
  sdhc_port_pin_cfg.hsiom = CY_SDHC_CARD_DAT_3TO03_PIN_MUX;
  Cy_GPIO_Pin_Init(CY_SDHC_CARD_DAT_3TO03_PORT, CY_SDHC_CARD_DAT_3TO03_PIN, &sdhc_port_pin_cfg);

  Cy_SD_Host_Init((volatile stc_SDHC_t *)CY_SDHC_TYPE, &host_cfg, &sd_host_context);

  enRet = Cy_SD_Host_InitCard((volatile stc_SDHC_t *)CY_SDHC_TYPE, &stc_card_cfg, &sd_host_context);
  ASSERT(enRet == CY_SD_HOST_SUCCESS);
  wait_complete(&sd_host_context);

  return 0;
}

__attribute__((used)) int UnInit(unsigned long fnc) {
  (void)fnc;

  return 0;
}

__attribute__((used)) int EraseSector(unsigned long adr) {
  cy_en_sd_host_status_t enRet;

  adr   = (adr - 0x60000000) / ROW_SIZE;
  enRet = Cy_SD_Host_Erase((volatile stc_SDHC_t *)CY_SDHC_TYPE, adr, adr, CY_SD_HOST_ERASE_ERASE, &sd_host_context);
  ASSERT(enRet == CY_SD_HOST_SUCCESS);

  enRet = Cy_SD_Host_PollCmdComplete((volatile stc_SDHC_t *)CY_SDHC_TYPE);
  ASSERT(enRet == CY_SD_HOST_SUCCESS);
  wait_complete(&sd_host_context);

  return 0;
}

__attribute__((used)) int ProgramPage(unsigned long adr, unsigned long sz, const unsigned char *buf) {
  ASSERT(sz == ROW_SIZE);
  ASSERT(adr % ROW_SIZE == 0);

  cy_stc_sd_host_write_read_config_t c = {
      .data            = (uint32_t *)buf,
      .address         = (adr - 0x60000000) / ROW_SIZE,
      .numberOfBlocks  = 1,
      .autoCommand     = CY_SD_HOST_AUTO_CMD_AUTO,
      .dataTimeout     = 0x0E,
      .enReliableWrite = 0,
      .enableDma       = 0,
  };

  enRet = Cy_SD_Host_Write((volatile stc_SDHC_t *)CY_SDHC_TYPE, &c, &sd_host_context);
  ASSERT(enRet == CY_SD_HOST_SUCCESS);
  wait_complete(&sd_host_context);
  return 0;
}

static uint8_t page_buf[ROW_SIZE];

void aligned_read(uint32_t addr, uint32_t size, uint8_t *dest) {
  while (addr % ROW_SIZE)
    continue;

  while (size % ROW_SIZE)
    continue;

  cy_stc_sd_host_write_read_config_t c = {
      .data            = (uint32_t *)dest,
      .address         = (addr - 0x60000000) / ROW_SIZE,
      .numberOfBlocks  = size / ROW_SIZE,
      .autoCommand     = CY_SD_HOST_AUTO_CMD_AUTO,
      .dataTimeout     = 0x0E,
      .enReliableWrite = 0,
      .enableDma       = 0,
  };

  enRet = Cy_SD_Host_Read((volatile stc_SDHC_t *)CY_SDHC_TYPE, &c, &sd_host_context);
  ASSERT(enRet == CY_SD_HOST_SUCCESS);
  wait_complete(&sd_host_context);
}

__attribute__((used)) int SEGGER_FL_Read(uint32_t addr, uint32_t size, uint8_t *dest) {
  memset(dest, 0xAA, size);

  uint32_t s = 0;
  if (addr % ROW_SIZE) {
    uint32_t aligned_addr = addr & ~(ROW_SIZE - 1);
    uint32_t bytes_read   = ROW_SIZE - (addr - aligned_addr);

    aligned_read(aligned_addr, ROW_SIZE, page_buf);
    memcpy(dest, &page_buf[addr - aligned_addr], bytes_read);
    addr = aligned_addr + ROW_SIZE;
    size -= bytes_read;
    dest += bytes_read;
    s += bytes_read;
  }

  uint32_t bytes_read = size & ~(ROW_SIZE - 1);
  aligned_read(addr, bytes_read, dest);
  addr += bytes_read;
  dest += bytes_read;
  size -= bytes_read;
  s += bytes_read;

  if (size) {
    aligned_read(addr, ROW_SIZE, page_buf);
    memcpy(dest, page_buf, size);
    s += size;
  }

  return s;
}

__attribute__((used)) int BlankCheck(unsigned long adr, unsigned long sz, unsigned char pat) {
  ASSERT(sz % ROW_SIZE == 0);
  ASSERT(adr % ROW_SIZE == 0);

  for (uint32_t i = 0; i < sz / ROW_SIZE; i++) {
    SEGGER_FL_Read(adr, ROW_SIZE, page_buf);
    for (size_t i = 0; i < sizeof(page_buf); i++)
      if (page_buf[i])
        return 1;

    adr += ROW_SIZE;
  }

  return 0;
}
