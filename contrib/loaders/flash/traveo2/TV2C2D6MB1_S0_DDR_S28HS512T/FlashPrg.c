#include <string.h>

#include "../flm/glue_functions.c"
#include "../flm/sdl_helpers.c"
#include "../sdl/common/src/drivers/gpio/cy_gpio.c"
#include "../sdl/common/src/drivers/ipc/cy_ipc_drv.c"
#include "../sdl/common/src/drivers/smif/common/cy_smif.c"
#include "../sdl/common/src/drivers/smif/ver4/cy_smif_ver_specific.c"
#include "../sdl/common/src/drivers/srom/cy_srom.c"
#include "../sdl/common/src/drivers/syslib/cy_syslib.c"
#include "../sdl/common/src/drivers/systick/cy_systick.c"
#include "../sdl/common/src/drivers/syswdt/cy_syswdt.c"
#include "../sdl/tviic2d6m/src/drivers/sysclk/cy_sysclk.c"
#include "../sdl/tviic2d6m/src/mw/power/cy_power.c"
#include "../sdl/tviic2d6m/src/mw/smif_mem/cy_smif_device_common.c"
#include "../sdl/tviic2d6m/src/mw/smif_mem/cy_smif_semp.c"
#include "../sdl/tviic2d6m/src/system/rev_c/system_tviic2d6m_cm0plus.c"
#include "../sdl/tviic2d6mddr/src/drivers/syspmic/cy_syspmic.c"
//#include "../sdl/tviic2d6m/src/mw/smif_mem/cy_smif_hb_flash.c"
//#include "../sdl/tviic2d6m/src/mw/smif_mem/cy_smif_s25fl.c"
//#include "../sdl/tviic2d6m/src/mw/smif_mem/cy_smif_s25fs.c"
#include "tvii_series_smif_ex_adopter.h"

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

/******************************************************/
#define TGT_SMIF   SMIF_USED
#define TGT_DEVICE SMIF_DEVICE0
#define TGT_SLAVE  CY_SMIF_SLAVE_SELECT_0

#define TGT_PLL_FREQ    166000000u
#define TGT_TRX_TYPE    CY_SPI_TRANSACTION_8D8D8D
#define TGT_WRITE_LC    CY_SMIF_SEMP_WT_LATENCY0
#define TGT_READ_LC     CY_SMIF_SEMP_RD_LATENCY1
#define TGT_READ_REG_LC CY_SMIF_SEMP_REG_LATENCY1
/******************************************************/

typedef struct {
  volatile stc_GPIO_PRT_t* port;
  uint8_t pin;
  en_hsiom_sel_t hsiom;
  uint32_t driveMode;
} cy_stc_smif_port_t;

// clang-format off
static const cy_stc_smif_port_t smifPortCfg[] = {
    {CY_SMIF_CLK_PORT,     CY_SMIF_CLK_PIN,     CY_SMIF_CLK_PIN_MUX,     CY_GPIO_DM_STRONG},
    {CY_SMIF_CLK_INV_PORT, CY_SMIF_CLK_INV_PIN, HSIOM_SEL_GPIO,          CY_GPIO_DM_STRONG_IN_OFF},
    {CY_SMIF_RWDS_PORT,    CY_SMIF_RWDS_PIN,    CY_SMIF_RWDS_PIN_MUX,    CY_GPIO_DM_STRONG},
    {CY_SMIF_SELECT0_PORT, CY_SMIF_SELECT0_PIN, CY_SMIF_SELECT0_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_SELECT1_PORT, CY_SMIF_SELECT1_PIN, CY_SMIF_SELECT1_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA0_PORT,   CY_SMIF_DATA0_PIN,   CY_SMIF_DATA0_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA1_PORT,   CY_SMIF_DATA1_PIN,   CY_SMIF_DATA1_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA2_PORT,   CY_SMIF_DATA2_PIN,   CY_SMIF_DATA2_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA3_PORT,   CY_SMIF_DATA3_PIN,   CY_SMIF_DATA3_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA4_PORT,   CY_SMIF_DATA4_PIN,   CY_SMIF_DATA4_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA5_PORT,   CY_SMIF_DATA5_PIN,   CY_SMIF_DATA5_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA6_PORT,   CY_SMIF_DATA6_PIN,   CY_SMIF_DATA6_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA7_PORT,   CY_SMIF_DATA7_PIN,   CY_SMIF_DATA7_PIN_MUX,   CY_GPIO_DM_STRONG},
};
// clang-format on
#define SIZE_OF_PORT (sizeof(smifPortCfg) / sizeof(cy_stc_smif_port_t))

static void smifport_init(const cy_stc_smif_port_t cfg[], uint8_t size) {
  cy_stc_gpio_pin_config_t pinCfg = {0};
  for (uint32_t i = 0; i < size; i++) {
    pinCfg.driveMode = cfg[i].driveMode;
    pinCfg.hsiom     = cfg[i].hsiom;
    Cy_GPIO_SetDriveSelTrim(cfg[i].port, cfg[i].pin, CY_GPIO_DRIVE_STRENGTH_90OHM);
    Cy_GPIO_Pin_Init(cfg[i].port, cfg[i].pin, &pinCfg);
  }
}

static cy_stc_smif_context_t smif_ctxt;
static cy_stc_ex_dev_context_t smif_dev_ctxt;

static cy_stc_smif_config_t smif_cfg;
static cy_stc_device_config_t smif_dev_cfg;
static cy_stc_smif_dll_config_t dll_cfg;

static void init_variables() {
  memset(g_sempReadLatencyCodeToDummyCycle, 0, sizeof(g_sempReadLatencyCodeToDummyCycle));
  memset(g_sempWriteLatencyCodeToDummyCycle, 0, sizeof(g_sempWriteLatencyCodeToDummyCycle));
  memset(cmdLatTbls, 0, sizeof(cmdLatTbls));

  g_sempReadLatencyCodeToDummyCycle[0]  = g_sempLatTblOtherThanOctalRead;
  g_sempReadLatencyCodeToDummyCycle[1]  = g_sempLatTblOctalRead;
  g_sempWriteLatencyCodeToDummyCycle[0] = g_latancyTableOfWrite;

  cmdLatTbls[0] = NULL;
  cmdLatTbls[1] = g_sempLatTblRegType1;
  cmdLatTbls[2] = g_sempLatTblRegType2;
  cmdLatTbls[3] = g_sempLatTblRegType3;
  cmdLatTbls[4] = g_sempLatTblOtherThanOctalRead;
  cmdLatTbls[5] = g_sempLatTblOctalRead;

  static const cy_stc_smif_config_t smif_cfg_c = {
      .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
      .setupDelay    = CY_SMIF_SETUP_2_CLK_PULUS_MIN, /* BHDT was 0 */
      .holdDelay     = CY_SMIF_HOLD_2_CLK_PULUS_MIN,  /* BHDT was 0 */
      .mode          = CY_SMIF_NORMAL,                /* MMIO mode */
      .blockEvent    = CY_SMIF_BUS_ERROR,
      .pDllCfg       = NULL,  //&dll_cfg,
  };

  memcpy(&smif_cfg, &smif_cfg_c, sizeof(smif_cfg_c));
  smif_cfg.pDllCfg = &dll_cfg;

  static const cy_stc_device_config_t smif_dev_cfg_c = {
      .xipReadCmdId       = CY_SMIF_SEMP_RDID_8D8D8D_4ADR,
      .xipWriteCmdId      = CY_SMIF_SEMP_WTID_8D8D8D_4ADR,
      .writeEn            = false,
      .xipReadMode        = 0u,
      .xipWriteMode       = 0u,
      .mergeEnable        = true,
      .mergeTimeout       = CY_SMIF_MER_TIMEOUT_4096_CYCLE,
      .totalTimeoutEnable = false,
      .totalTimeout       = 1000u,
      .addr               = 0,  // will be updated in the application
      .size               = CY_SMIF_DEVICE_64M_BYTE,
      .rdLatencyCode      = TGT_READ_LC,
      .wtLatencyCode      = TGT_WRITE_LC,
  };

  memcpy(&smif_dev_cfg, &smif_dev_cfg_c, sizeof(smif_dev_cfg_c));

  static const cy_stc_smif_dll_config_t dll_cfg_c = {
      .pllFreq    = TGT_PLL_FREQ,
      .mdlOutDiv  = CY_SMIF_MDL_CLK_OUT_DIV16,
      .mdlTapSel  = CY_SMIF_DDL_7_TAP_DELAY,
      .rxCapMode  = CY_SMIF_CAP_MODE_SPI,
      .txSdrExtra = CY_SMIF_TX_ONE_PERIOD_AHEAD,
  };

  memcpy(&dll_cfg, &dll_cfg_c, sizeof(dll_cfg_c));
}

__attribute__((used)) int Init(unsigned long adr, unsigned long clk, unsigned long fnc) {
  (void)adr;
  (void)clk;

  __disable_irq();
  init_variables();

  cy_en_smif_status_t hr;
  SystemInit();

  /* Apply DLL trim */
  *(volatile uint32_t*)(((uint32_t)TGT_SMIF) + 0x7E0) = 0b00101000;

  Cy_SMIF_DeInit(TGT_SMIF);
  Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
  ChangePLLFrequency(TGT_PLL_FREQ);

  smifport_init(smifPortCfg, SIZE_OF_PORT);
  hr = Cy_SMIF_InitExMemContext(CY_SMIF_SEMP, &smif_dev_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);
  hr = Cy_SMIF_Init(TGT_SMIF, &smif_cfg, 1000, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  Cy_SMIF_DeviceSetDataSelect(TGT_DEVICE, CY_SMIF_DATA_SEL0);
  Cy_SMIF_DeviceWriteEnable(TGT_DEVICE);
  Cy_SMIF_DeviceSetRxCaptureSdr(TGT_DEVICE, CY_SMIF_RX_CAP_STYLE_SDR_POS);
  Cy_SMIF_DeviceSetRxCaptureDdr(TGT_DEVICE, CY_SMIF_RX_CAP_STYLE_DDR_OCTAL);  // only for mode detection
  hr = Cy_SMIF_Set_DelayTapSel(TGT_DEVICE, 15);
  ASSERT(hr == CY_SMIF_SUCCESS);

  Cy_SMIF_Enable(TGT_SMIF, &smif_ctxt);
  for (size_t i = 0; i < 100000 && !Cy_SMIF_IsDllLocked(TGT_SMIF); i++) {};
  ASSERT(Cy_SMIF_IsDllLocked(TGT_SMIF));

  cy_en_smif_trx_type_t crnt_trx_type            = 0;
  cy_en_smif_semp_reg_latency_code_t crnt_reg_lc = CY_SMIF_SEMP_REG_LATENCY1;

  hr = semp_detect_and_reset(TGT_SMIF, TGT_SLAVE, &s28hx512t_id, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = semp_detect(TGT_SMIF, TGT_SLAVE, TGT_READ_LC, &s28hx512t_id, &crnt_trx_type, &crnt_reg_lc, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = semp_setup_spi_regs(TGT_SMIF, TGT_SLAVE, crnt_reg_lc, TGT_READ_REG_LC, TGT_READ_LC, crnt_trx_type, SEMP_S28H,
                           SECTOR_ARCH, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  bool use_ddr = (TGT_TRX_TYPE == CY_SPI_TRANSACTION_8D8D8D);
  hr = semp_s28h_enter_ospi_mode(TGT_SMIF, TGT_SLAVE, TGT_READ_REG_LC, crnt_trx_type, use_ddr, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  while (Cy_SMIF_IsBusy(TGT_SMIF)) {};
  Cy_SMIF_Disable(TGT_SMIF);
  Cy_SMIF_Disable(TGT_SMIF);

  dll_cfg.rxCapMode = CY_SMIF_CAP_MODE_RWDS;

  Cy_SMIF_Init(TGT_SMIF, &smif_cfg, 1000, &smif_ctxt);
  Cy_SMIF_Enable(TGT_SMIF, &smif_ctxt);

  while (Cy_SMIF_IsDllLocked(TGT_SMIF) == false) {};

  smif_dev_cfg.addr = CY_SMIF_GetXIP_Address(TGT_SMIF);

  hr = Cy_SMIF_InitXIPModeForExMem(TGT_DEVICE, CY_SMIF_DEV_OSPI, &smif_dev_cfg, &smif_dev_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  smif_set_mode(TGT_SMIF, CY_SMIF_MEMORY);

  Cy_SMIF_CacheInvalidate(TGT_SMIF, CY_SMIF_CACHE_BOTH);
  Cy_SMIF_CacheDisable(TGT_SMIF, CY_SMIF_CACHE_BOTH);

  if (fnc == 3) /* Re-enable cache for Read/Verify op */
    Cy_SMIF_CacheEnable(TGT_SMIF, CY_SMIF_CACHE_BOTH);

  return 0;
}

__attribute__((used)) int UnInit(unsigned long fnc) {
  (void)fnc;

  Cy_SMIF_CacheInvalidate(TGT_SMIF, CY_SMIF_CACHE_BOTH);
  Cy_SMIF_CacheDisable(TGT_SMIF, CY_SMIF_CACHE_BOTH);
  Cy_SMIF_CacheEnable(TGT_SMIF, CY_SMIF_CACHE_BOTH);

  return 0;
}

__attribute__((used)) int EraseSector(unsigned long adr) {
  ASSERT(adr >= CY_SMIF_GetXIP_Address(TGT_SMIF))
  adr -= CY_SMIF_GetXIP_Address(TGT_SMIF);

  Cy_SMIF_SEMP_WriteEnable(TGT_SMIF, TGT_SLAVE, TGT_TRX_TYPE, &smif_ctxt);
  Cy_SMIF_SEMP_Erase_256KB_Sector(TGT_SMIF, TGT_SLAVE, adr, TGT_TRX_TYPE, &smif_ctxt);
  ASSERT(semp_wait_ready(TGT_SMIF, TGT_SLAVE, TGT_READ_REG_LC, TGT_TRX_TYPE, &smif_ctxt) == 0);

  return 0;
}

__attribute__((used)) int EraseChip() {
  ASSERT(Cy_SMIF_SEMP_WriteEnable(TGT_SMIF, TGT_SLAVE, TGT_TRX_TYPE, &smif_ctxt) == CY_SMIF_SUCCESS);
  ASSERT(Cy_SMIF_SEMP_EraseChip(TGT_SMIF, TGT_SLAVE, TGT_TRX_TYPE, &smif_ctxt) == CY_SMIF_SUCCESS);
  ASSERT(semp_wait_ready(TGT_SMIF, TGT_SLAVE, TGT_READ_REG_LC, TGT_TRX_TYPE, &smif_ctxt) == 0);

  return 0;
}

__attribute__((used)) int ProgramPage(unsigned long adr, unsigned long sz, const unsigned char* buf) {
  ASSERT(adr >= CY_SMIF_GetXIP_Address(TGT_SMIF))
  adr -= CY_SMIF_GetXIP_Address(TGT_SMIF);

  Cy_SMIF_SEMP_WriteEnable(TGT_SMIF, TGT_SLAVE, TGT_TRX_TYPE, &smif_ctxt);
  Cy_SMIF_MMIO_Program_ExMem(TGT_SMIF, TGT_SLAVE, CY_SMIF_SEMP_WTID_8D8D8D_4ADR, adr, sz, buf, CY_SMIF_BLOCKING,
                             TGT_WRITE_LC, Cy_SMIF_SEMP_MODE_NOT_CONTINUOUS, &smif_dev_ctxt, &smif_ctxt);

  ASSERT(semp_wait_ready(TGT_SMIF, TGT_SLAVE, TGT_READ_REG_LC, TGT_TRX_TYPE, &smif_ctxt) == 0);

  return 0;
}
