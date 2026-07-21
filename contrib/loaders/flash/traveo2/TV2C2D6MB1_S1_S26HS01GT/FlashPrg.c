#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../flm/glue_functions.c"
#include "../flm/sdl_helpers.c"
#include "../sdl/common/hdr/cy_project.h"
#include "../sdl/common/src/drivers/gpio/cy_gpio.c"
#include "../sdl/common/src/drivers/ipc/cy_ipc_drv.c"
#include "../sdl/common/src/drivers/smif/common/cy_smif.c"
#include "../sdl/common/src/drivers/smif/ver4/cy_smif_ver_specific.c"
#include "../sdl/common/src/drivers/srom/cy_srom.c"
#include "../sdl/common/src/drivers/syslib/cy_syslib.c"
#include "../sdl/common/src/drivers/systick/cy_systick.c"
#include "../sdl/common/src/drivers/syswdt/cy_syswdt.c"
#include "../sdl/tviic2d6m/src/drivers/sysclk/cy_sysclk.c"
#include "../sdl/tviic2d6m/src/examples/smif/tvii_series_smif_ex_adopter.h"
#include "../sdl/tviic2d6m/src/mw/power/cy_power.c"
#include "../sdl/tviic2d6m/src/mw/smif_mem/cy_smif_device_common.c"
#include "../sdl/tviic2d6m/src/mw/smif_mem/cy_smif_hb_flash.c"
#include "../sdl/tviic2d6m/src/mw/smif_mem/cy_smif_semp.c"
#include "../sdl/tviic2d6m/src/system/rev_c/system_tviic2d6m_cm0plus.c"
#include "../sdl/tviic2d6mddr/src/drivers/syspmic/cy_syspmic.c"

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

/***** User Modifiable definitions *****/
#define TARGET_PLL_FREQ  (166 * 1000000)
#define SMIF_DEVICE      SMIF_DEVICE0
#define SLAVE_SELECT     CY_SMIF_SLAVE_SELECT_0
#define TEST_WRITE_LC    CY_SMIF_SEMP_WT_LATENCY0
#define TEST_READ_LC     CY_SMIF_SEMP_RD_LATENCY8
#define TEST_READ_REG_LC CY_SMIF_SEMP_REG_LATENCY0
/**************************************/

static cy_stc_smif_context_t smif_ctxt;
static cy_stc_ex_dev_context_t smif_dev_ctxt;

static cy_stc_device_config_t smif_dev_cfg = {
    .xipReadCmdId       = CY_SMIF_SEMP_RDID_1S1S1S_4ADR,
    .xipReadMode        = 0x00,
    .writeEn            = false,
    .xipWriteCmdId      = CY_SMIF_SEMP_WTID_1S1S1S_4ADR,
    .xipWriteMode       = 0u,
    .mergeEnable        = true,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_4096_CYCLE,
    .totalTimeoutEnable = true,
    .totalTimeout       = 1000u,
    .addr               = 0,
    .size               = FLASH_SIZE,
    .rdLatencyCode      = TEST_READ_LC,
    .wtLatencyCode      = TEST_WRITE_LC,
};

typedef struct {
  volatile stc_GPIO_PRT_t* port;
  uint8_t pin;
  en_hsiom_sel_t hsiom;
  uint32_t driveMode;
} cy_stc_smif_port_t;

// clang-format off
cy_stc_smif_port_t smifPortCfg[] =
{
    {CY_SMIF_CLK_PORT,      CY_SMIF_CLK_PIN,     CY_SMIF_CLK_PIN_MUX,     CY_GPIO_DM_STRONG},
    {CY_SMIF_CLK_INV_PORT,  CY_SMIF_CLK_INV_PIN, HSIOM_SEL_GPIO,          CY_GPIO_DM_STRONG_IN_OFF},
    {CY_SMIF_RWDS_PORT,     CY_SMIF_RWDS_PIN,    CY_SMIF_RWDS_PIN_MUX,    CY_GPIO_DM_STRONG},
    {CY_SMIF_SELECT0_PORT,  CY_SMIF_SELECT0_PIN, CY_SMIF_SELECT0_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_SELECT1_PORT,  CY_SMIF_SELECT1_PIN, CY_SMIF_SELECT1_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA0_PORT,    CY_SMIF_DATA0_PIN,   CY_SMIF_DATA0_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA1_PORT,    CY_SMIF_DATA1_PIN,   CY_SMIF_DATA1_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA2_PORT,    CY_SMIF_DATA2_PIN,   CY_SMIF_DATA2_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA3_PORT,    CY_SMIF_DATA3_PIN,   CY_SMIF_DATA3_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA4_PORT,    CY_SMIF_DATA4_PIN,   CY_SMIF_DATA4_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA5_PORT,    CY_SMIF_DATA5_PIN,   CY_SMIF_DATA5_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA6_PORT,    CY_SMIF_DATA6_PIN,   CY_SMIF_DATA6_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_SMIF_DATA7_PORT,    CY_SMIF_DATA7_PIN,   CY_SMIF_DATA7_PIN_MUX,   CY_GPIO_DM_STRONG},
};
// clang-format on

#define SIZE_OF_PORT (sizeof(smifPortCfg) / sizeof(cy_stc_smif_port_t))

static void smif_port_init(const cy_stc_smif_port_t cfg[], uint8_t size) {
  cy_stc_gpio_pin_config_t pinCfg = {0};
  for (uint32_t i = 0; i < size; i++) {
    pinCfg.driveMode = cfg[i].driveMode;
    pinCfg.hsiom     = cfg[i].hsiom;
    Cy_GPIO_Pin_Init(cfg[i].port, cfg[i].pin, &pinCfg);
  }
}

enum cy_smif_mode_t {
  SMIF_MODE_SPI,
  SMIF_MODE_HYPERBUS,
};

static cy_stc_smif_dll_config_t dll_config_spi = {
    .pllFreq    = TARGET_PLL_FREQ,
    .mdlOutDiv  = CY_SMIF_MDL_CLK_OUT_DIV8,
    .mdlTapSel  = CY_SMIF_DDL_7_TAP_DELAY,
    .rxCapMode  = CY_SMIF_CAP_MODE_SPI,
    .txSdrExtra = CY_SMIF_TX_TWO_PERIOD_AHEAD,
};

static cy_stc_smif_config_t smif_spi_config = {
    .blockEvent    = CY_SMIF_BUS_ERROR,
    .capDelay      = CY_SMIF_CAPTURE_DELAY_0_CYCLE,
    .delaySel      = CY_SMIF_1_SEN_SEL_PER_TAP,
    .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
    .dlpAuto       = CY_SMIF_DLP_UPDATE_MANUAL,
    .holdDelay     = CY_SMIF_HOLD_1_CLK_PULUS_MIN,   //
    .mode          = CY_SMIF_NORMAL,                 //
    .rxClk         = CY_SMIF_INV_OUTPUT_CLK,         //
    .setupDelay    = CY_SMIF_SETUP_1_CLK_PULUS_MIN,  //
    .txClk         = CY_SMIF_INV_FOR_DDR,
    .pDllCfg       = &dll_config_spi,
};

static cy_stc_smif_dll_config_t dll_config_hb = {
    .pllFreq    = TARGET_PLL_FREQ,
    .mdlOutDiv  = CY_SMIF_MDL_CLK_OUT_DIV8,
    .mdlTapSel  = CY_SMIF_DDL_7_TAP_DELAY,
    .rxCapMode  = CY_SMIF_CAP_MODE_RWDS,
    .txSdrExtra = CY_SMIF_TX_TWO_PERIOD_AHEAD,
};

static cy_stc_smif_config_t smif_hb_config = {
    .txClk         = CY_SMIF_INV_FOR_DDR,
    .rxClk         = CY_SMIF_INV_RWDS,
    .dlpAuto       = CY_SMIF_DLP_UPDATE_MANUAL,
    .capDelay      = CY_SMIF_CAPTURE_DELAY_1_CYCLE,
    .delaySel      = CY_SMIF_1_SEN_SEL_PER_TAP,
    .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
    .setupDelay    = CY_SMIF_SETUP_3_CLK_PULUS_MIN,
    .holdDelay     = CY_SMIF_HOLD_1_CLK_PULUS_MIN,
    .mode          = CY_SMIF_MEMORY,
    .blockEvent    = CY_SMIF_WAIT_STATES,
    .pDllCfg       = &dll_config_hb,
};

static void smif_configure(volatile cy_stc_smif_reg_device_t* device, enum cy_smif_mode_t mode) {
  smif_spi_config.pDllCfg = &dll_config_spi;
  smif_hb_config.pDllCfg  = &dll_config_hb;

  Cy_SMIF_DeInit(SMIF_USED);
  memset(&smif_ctxt, 0, sizeof(smif_ctxt));
  Cy_SMIF_Init(SMIF_USED, mode == SMIF_MODE_SPI ? &smif_spi_config : &smif_hb_config, 1000, &smif_ctxt);
  Cy_SMIF_Set_DelayTapSel(SMIF_USED, (CY_SMIF_GetDelayTapsNumber(SMIF_USED) - 1));
  Cy_SMIF_DeviceSetDataSelect(device, CY_SMIF_DATA_SEL0);
  Cy_SMIF_DeviceWriteEnable(device);
  Cy_SMIF_Enable(SMIF_USED, &smif_ctxt);
  for (size_t i = 0; i < 100000 && !Cy_SMIF_IsDllLocked(SMIF_USED); i++) {};
  ASSERT(Cy_SMIF_IsDllLocked(SMIF_USED));
  Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH);
}

static cy_stc_device_hb_config_t smifDevHBFlashCfg = {
    .xipReadCmd         = CY_SMIF_HB_READ_CONTINUOUS_BURST,
    .xipWriteCmd        = CY_SMIF_HB_WRITE_CONTINUOUS_BURST,
    .mergeEnable        = true,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_1_CYCLE,
    .totalTimeoutEnable = true,
    .totalTimeout       = 1000u,
    .addr               = 0x00000000,
    .size               = FLASH_SIZE,
    .lc_hb              = CY_SMIF_HB_LC5,
    .hbDevType          = CY_SMIF_HB_FLASH,
};

static cy_en_smif_status_t configure_spi_mode(volatile cy_stc_smif_reg_t* smif,
                                              volatile cy_stc_smif_reg_device_t* device,
                                              cy_en_smif_slave_select_t slave) {
  cy_en_smif_status_t hr;

  smifDevHBFlashCfg.addr = CY_SMIF_GetXIP_Address(SMIF_USED);

  smif_configure(device, SMIF_MODE_SPI);

  cy_en_smif_trx_type_t crnt_trx_type;
  cy_en_smif_semp_reg_latency_code_t crnt_reg_lc;
  hr = semp_detect(smif, slave, TEST_READ_REG_LC, &s26hx01gt_id, &crnt_trx_type, &crnt_reg_lc, &smif_ctxt);

  if (hr != CY_SMIF_SUCCESS) {
    smif_configure(device, SMIF_MODE_HYPERBUS);
    hr = Cy_SMIF_InitDeviceHyperBus(device, &smifDevHBFlashCfg);
    if (hr != CY_SMIF_SUCCESS)
      return hr;

    Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH);

    CY_SMIF_HbFlash_EnterSPIModeCmd((volatile CY_SMIF_FLASHDATA*)CY_SMIF_GetXIP_Address(SMIF_USED));

    smif_configure(device, SMIF_MODE_SPI);

    hr = semp_detect(smif, slave, TEST_READ_REG_LC, &s26hx01gt_id, &crnt_trx_type, &crnt_reg_lc, &smif_ctxt);
    if (hr != CY_SMIF_SUCCESS)
      return hr;
  }

  hr = semp_setup_spi_regs(smif, slave, TEST_READ_REG_LC, TEST_READ_REG_LC, TEST_READ_LC, crnt_trx_type,
                           SEMP_S26H, SECTORS_UNIFORM, &smif_ctxt);

  if (hr != CY_SMIF_SUCCESS)
    return hr;

  hr = semp_reset(smif, slave, CY_SPI_TRANSACTION_1S1S1S, CY_SMIF_SEMP_REG_LATENCY0, &smif_ctxt);
  if (hr != CY_SMIF_SUCCESS)
    return hr;

  hr = semp_detect(smif, slave, TEST_READ_REG_LC, &s26hx01gt_id, &crnt_trx_type, &crnt_reg_lc, &smif_ctxt);
  return hr;
}

__attribute__((used)) int EraseSector(unsigned long adr) {
  cy_en_smif_status_t hr;

  ASSERT(adr >= CY_SMIF_GetXIP_Address(SMIF_USED));
  adr -= CY_SMIF_GetXIP_Address(SMIF_USED);

  smif_set_mode(SMIF_USED, CY_SMIF_NORMAL);
  hr = Cy_SMIF_SEMP_WriteEnable(SMIF_USED, SLAVE_SELECT, CY_SPI_TRANSACTION_1S1S1S, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = Cy_SMIF_SEMP_Erase_256KB_Sector(SMIF_USED, SLAVE_SELECT, adr, CY_SPI_TRANSACTION_1S1S1S, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = semp_wait_ready(SMIF_USED, SLAVE_SELECT, TEST_READ_REG_LC, CY_SPI_TRANSACTION_1S1S1S, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  smif_set_mode(SMIF_USED, CY_SMIF_MEMORY);

  return (0);
}

__attribute__((used)) int EraseChip(void) {
  cy_en_smif_status_t hr;

  smif_set_mode(SMIF_USED, CY_SMIF_NORMAL);

  hr = Cy_SMIF_SEMP_WriteEnable(SMIF_USED, SLAVE_SELECT, CY_SPI_TRANSACTION_1S1S1S, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = Cy_SMIF_SEMP_EraseChip(SMIF_USED, SLAVE_SELECT, CY_SPI_TRANSACTION_1S1S1S, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = semp_wait_ready(SMIF_USED, SLAVE_SELECT, TEST_READ_REG_LC, CY_SPI_TRANSACTION_1S1S1S, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  smif_set_mode(SMIF_USED, CY_SMIF_MEMORY);

  return 0;
}

__attribute__((used)) int ProgramPage(unsigned long adr, unsigned long sz, const unsigned char* buf) {
  cy_en_smif_status_t hr;

  ASSERT(adr >= CY_SMIF_GetXIP_Address(SMIF_USED));
  adr -= CY_SMIF_GetXIP_Address(SMIF_USED);

  smif_set_mode(SMIF_USED, CY_SMIF_NORMAL);

  hr = Cy_SMIF_SEMP_WriteEnable(SMIF_USED, SLAVE_SELECT, CY_SPI_TRANSACTION_1S1S1S, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = Cy_SMIF_MMIO_Program_ExMem(SMIF_USED, SLAVE_SELECT, CY_SMIF_SEMP_WTID_1S1S1S_4ADR, adr, sz, buf,
                                  CY_SMIF_BLOCKING, TEST_WRITE_LC, 0, &smif_dev_ctxt, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = semp_wait_ready(SMIF_USED, SLAVE_SELECT, TEST_READ_REG_LC, CY_SPI_TRANSACTION_1S1S1S, &smif_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  smif_set_mode(SMIF_USED, CY_SMIF_MEMORY);

  return (0);
}

__attribute__((used)) int Init(unsigned long adr, unsigned long clk, unsigned long fnc) {
  (void)adr;
  (void)clk;

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

  cy_en_smif_status_t hr;
  smif_dev_cfg.addr = CY_SMIF_GetXIP_Address(SMIF_USED);

  SystemInit();

  Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
  ChangePLLFrequency(TARGET_PLL_FREQ);
  smif_port_init(smifPortCfg, SIZE_OF_PORT);

  hr = Cy_SMIF_InitExMemContext(CY_SMIF_SEMP, &smif_dev_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = configure_spi_mode(SMIF_USED, SMIF_DEVICE, SLAVE_SELECT);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = Cy_SMIF_InitXIPModeForExMem(SMIF_DEVICE0, CY_SMIF_DEV_SINGLE_BIT_0To1, &smif_dev_cfg, &smif_dev_ctxt);
  ASSERT(hr == CY_SMIF_SUCCESS);

  smif_set_mode(SMIF_USED, CY_SMIF_MEMORY);

  Cy_SMIF_CacheInvalidate(SMIF_USED, CY_SMIF_CACHE_BOTH);
  Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH);

  if (fnc == 3) /* Re-enable cache for Read/Verify op */
    Cy_SMIF_CacheEnable(SMIF_USED, CY_SMIF_CACHE_BOTH);

  return 0;
}

__attribute__((used)) int UnInit(unsigned long fnc) {
  (void)fnc;

  Cy_SMIF_CacheInvalidate(SMIF_USED, CY_SMIF_CACHE_BOTH);
  Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH);
  Cy_SMIF_CacheEnable(SMIF_USED, CY_SMIF_CACHE_BOTH);

  return 0;
}
