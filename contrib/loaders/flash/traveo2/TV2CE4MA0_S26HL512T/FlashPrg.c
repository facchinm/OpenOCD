
#include <string.h>

#include "../flm/glue_functions.c"
#include "../flm/sdl_helpers.c"
#include "../sdl/common/hdr/cy_project.h"
#include "../sdl/common/src/drivers/gpio/cy_gpio.c"
#include "../sdl/common/src/drivers/ipc/cy_ipc_drv.c"
#include "../sdl/common/src/drivers/smif/common/cy_smif.c"
#include "../sdl/common/src/drivers/smif/ver3_1/cy_smif_ver_specific.c"
#include "../sdl/common/src/drivers/syslib/cy_syslib.c"
#include "../sdl/common/src/drivers/systick/cy_systick.c"
#include "../sdl/common/src/drivers/syswdt/cy_syswdt.c"
#include "../sdl/tviic2d4m/src/mw/smif_mem/cy_smif_semp.c"
#include "../sdl/tviice4m/src/drivers/sysclk/cy_sysclk.c"
#include "../sdl/tviice4m/src/mw/smif_mem/cy_smif_device_common.c"
#include "../sdl/tviice4m/src/mw/smif_mem/cy_smif_hb_flash.c"
#include "../sdl/tviice4m/src/system/rev_a/system_tviice4m_cm0plus.c"

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
#define SMIF_USED        CY_BSP_SMIF0_TYPE
#define SMIF_DEVICE_USED SMIF0_DEVICE0
#define SLAVE_SELECT     CY_SMIF_SLAVE_SELECT_0
#define READ_LATENCY     CY_SMIF_HB_LC20
/**************************************/

static volatile CY_SMIF_FLASHDATA* g_base_addr;
static cy_stc_smif_context_t smif_ctxt;

static cy_stc_device_hb_config_t smif_dev_config = {
    .addr               = 0,  // will be updated in the application
    .hbDevType          = CY_SMIF_HB_FLASH,
    .lc_hb              = READ_LATENCY,
    .mergeEnable        = true,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_1_CYCLE,
    .size               = CY_SMIF_DEVICE_64M_BYTE,
    .totalTimeout       = 500,
    .totalTimeoutEnable = false,
    .xipReadCmd         = CY_SMIF_HB_READ_CONTINUOUS_BURST,
    .xipWriteCmd        = CY_SMIF_HB_WRITE_CONTINUOUS_BURST,
};

typedef struct {
  volatile stc_GPIO_PRT_t* port;
  uint8_t pin;
  en_hsiom_sel_t hsiom;
  uint32_t driveMode;
} cy_stc_smif_port_t;

// clang-format off
static const cy_stc_smif_port_t smifPortCfg[] = {
	{CY_BSP_SMIF0_CLK_PORT,     CY_BSP_SMIF0_CLK_PIN,     CY_BSP_SMIF0_CLK_PIN_MUX,     CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_RWDS_PORT,    CY_BSP_SMIF0_RWDS_PIN,    CY_BSP_SMIF0_RWDS_PIN_MUX,    CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_SELECT0_PORT, CY_BSP_SMIF0_SELECT0_PIN, CY_BSP_SMIF0_SELECT0_PIN_MUX, CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_SELECT1_PORT, CY_BSP_SMIF0_SELECT1_PIN, CY_BSP_SMIF0_SELECT1_PIN_MUX, CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_DATA0_PORT,   CY_BSP_SMIF0_DATA0_PIN,   CY_BSP_SMIF0_DATA0_PIN_MUX,   CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_DATA1_PORT,   CY_BSP_SMIF0_DATA1_PIN,   CY_BSP_SMIF0_DATA1_PIN_MUX,   CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_DATA2_PORT,   CY_BSP_SMIF0_DATA2_PIN,   CY_BSP_SMIF0_DATA2_PIN_MUX,   CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_DATA3_PORT,   CY_BSP_SMIF0_DATA3_PIN,   CY_BSP_SMIF0_DATA3_PIN_MUX,   CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_DATA4_PORT,   CY_BSP_SMIF0_DATA4_PIN,   CY_BSP_SMIF0_DATA4_PIN_MUX,   CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_DATA5_PORT,   CY_BSP_SMIF0_DATA5_PIN,   CY_BSP_SMIF0_DATA5_PIN_MUX,   CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_DATA6_PORT,   CY_BSP_SMIF0_DATA6_PIN,   CY_BSP_SMIF0_DATA6_PIN_MUX,   CY_GPIO_DM_STRONG},
	{CY_BSP_SMIF0_DATA7_PORT,   CY_BSP_SMIF0_DATA7_PIN,   CY_BSP_SMIF0_DATA7_PIN_MUX,   CY_GPIO_DM_STRONG},
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

#define SMIF_MPU_REG_NO    MPU_REG_SMIF0_DEV
#define SMIF_HF_CLOCK      CY_SYSCLK_HFCLK_5
#define PLL_200M_0_PATH_NO (3u)
#define PLL_200M_0_PLL_NO  (PLL_200M_0_PATH_NO - 1u)
static void ChangePLLFrequency(uint32_t outputFreq) {
  uint32_t pllInputClockFreq = 0;
  ASSERT(Cy_SysClk_GetPllInputFrequency(PLL_200M_0_PLL_NO, &pllInputClockFreq) == CY_SYSCLK_SUCCESS);

  cy_stc_pll_config_t pll200_1_Config = {
      .inputFreq  = pllInputClockFreq,
      .outputFreq = outputFreq,  // target PLL output
      .lfMode     = 0u,          // VCO frequency is [200MHz, 400MHz]
      .outputMode = CY_SYSCLK_FLLPLL_OUTPUT_AUTO,
  };

  ASSERT(Cy_SysClk_PllDisable(PLL_200M_0_PATH_NO) == CY_SYSCLK_SUCCESS);
  ASSERT(Cy_SysClk_PllConfigure(PLL_200M_0_PATH_NO, &pll200_1_Config) == CY_SYSCLK_SUCCESS);
  ASSERT(Cy_SysClk_PllEnable(PLL_200M_0_PATH_NO, 10000ul) == CY_SYSCLK_SUCCESS);
  return;
}

enum cy_smif_mode_t {
  SMIF_MODE_SPI,
  SMIF_MODE_HYPERBUS,
};

static void smif_configure(volatile cy_stc_smif_reg_device_t* device, enum cy_smif_mode_t mode) {
  static const cy_stc_smif_config_t smif_spi_config = {
      .blockEvent    = CY_SMIF_BUS_ERROR,
      .capDelay      = CY_SMIF_CAPTURE_DELAY_0_CYCLE,
      .delaySel      = CY_SMIF_1_SEN_SEL_PER_TAP,
      .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
      .dlpAuto       = CY_SMIF_DLP_UPDATE_MANUAL,
      .holdDelay     = CY_SMIF_HOLD_0_CLK_PULUS_MIN,
      .mode          = CY_SMIF_NORMAL,
      .rxClk         = CY_SMIF_INV_OUTPUT_CLK,
      .setupDelay    = CY_SMIF_SETUP_0_CLK_PULUS_MIN,
      .txClk         = CY_SMIF_INV_FOR_DDR,
  };

  static const cy_stc_smif_config_t smif_hb_config = {
      .blockEvent    = CY_SMIF_BUS_ERROR,
      .capDelay      = CY_SMIF_CAPTURE_DELAY_0_CYCLE,
      .delaySel      = CY_SMIF_1_SEN_SEL_PER_TAP,
      .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
      .dlpAuto       = CY_SMIF_DLP_UPDATE_MANUAL,
      .holdDelay     = CY_SMIF_HOLD_3_CLK_PULUS_MIN,
      .mode          = CY_SMIF_MEMORY,
      .rxClk         = CY_SMIF_INV_RWDS,
      .setupDelay    = CY_SMIF_SETUP_3_CLK_PULUS_MIN,
      .txClk         = CY_SMIF_INV_FOR_DDR,
  };

  memset(&smif_ctxt, 0, sizeof(smif_ctxt));
  Cy_SMIF_DeInit(SMIF_USED);
  Cy_SMIF_Init(SMIF_USED, mode == SMIF_MODE_SPI ? &smif_spi_config : &smif_hb_config, 1000, &smif_ctxt);
  Cy_SMIF_Set_DelayTapSel(SMIF_USED, (CY_SMIF_GetDelayTapsNumber(SMIF_USED) - 1));
  Cy_SMIF_DeviceSetDataSelect(device, CY_SMIF_DATA_SEL0);
  Cy_SMIF_DeviceWriteEnable(device);
  Cy_SMIF_Enable(SMIF_USED, &smif_ctxt);
  Cy_SMIF_CacheInvalidate(SMIF_USED, CY_SMIF_CACHE_BOTH);
  Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH);
}

static cy_en_smif_status_t enter_hb_mode(volatile stc_SMIF_t* smif, volatile cy_stc_smif_reg_device_t* device,
                                         cy_en_smif_slave_select_t slave) {
  cy_en_smif_status_t hr;

  smif_configure(device, SMIF_MODE_SPI);

  cy_en_smif_trx_type_t crnt_trx_type;
  cy_en_smif_semp_reg_latency_code_t crnt_reg_lc;
  hr = semp_detect(smif, slave, CY_SMIF_SEMP_RD_LATENCY8, &s26hx512t_id, &crnt_trx_type, &crnt_reg_lc, &smif_ctxt);

  if (hr == CY_SMIF_SUCCESS) {
    hr = semp_setup_spi_regs(smif, slave, crnt_reg_lc, crnt_reg_lc, CY_SMIF_SEMP_RD_LATENCY8, crnt_trx_type, SEMP_S26H,
                             SECTORS_UNIFORM, &smif_ctxt);
    if (hr != CY_SMIF_SUCCESS)
      return hr;

    hr = semp_s26h_enter_hb_mode(smif, slave, crnt_reg_lc, crnt_trx_type, &smif_ctxt);
    if (hr != CY_SMIF_SUCCESS)
      return hr;
  }

  smif_configure(device, SMIF_MODE_HYPERBUS);
  hr = Cy_SMIF_InitDeviceHyperBus(device, &smif_dev_config);
  return hr;
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
  smif_dev_config.addr = CY_SMIF_GetXIP_Address(SMIF_USED);
  g_base_addr          = (CY_SMIF_FLASHDATA*)(smif_dev_config.addr);

  SystemInit();
  Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
  ChangePLLFrequency(66000000);
  smif_port_init(smifPortCfg, SIZE_OF_PORT);

  hr = enter_hb_mode(SMIF_USED, SMIF_DEVICE_USED, SLAVE_SELECT);
  ASSERT(hr == CY_SMIF_SUCCESS);

  CY_SMIF_HbFlash_EnterSPIModeCmd(g_base_addr);

  hr = enter_hb_mode(SMIF_USED, SMIF_DEVICE_USED, SLAVE_SELECT);
  ASSERT(hr == CY_SMIF_SUCCESS);

  hr = semp_setup_hb_regs(g_base_addr, SECTORS_UNIFORM, READ_LATENCY);
  ASSERT(hr == CY_SMIF_SUCCESS);

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

__attribute__((used)) int EraseSector(unsigned long adr) {
  ASSERT(adr >= (unsigned long)g_base_addr);
  adr -= (unsigned long)g_base_addr;

  CY_SMIF_HB_DEVSTATUS hr;
  hr = CY_SMIF_HbFlash_SectorEraseOp(g_base_addr, (CY_SMIF_ADDRESS)(adr / 2));
  return (hr != CY_SMIF_DEV_NOT_BUSY);
}

__attribute__((used)) int EraseChip() {
  CY_SMIF_HB_DEVSTATUS hr = CY_SMIF_HbFlash_ChipEraseOp(g_base_addr);
  return (hr != CY_SMIF_DEV_NOT_BUSY);
}

__attribute__((used)) int ProgramPage(unsigned long adr, unsigned long sz, const unsigned char* buf) {
  ASSERT(adr >= (unsigned long)g_base_addr);
  adr -= (unsigned long)g_base_addr;

  CY_SMIF_ADDRESS offset  = (adr / 2);
  CY_SMIF_FLASHDATA* data = (CY_SMIF_FLASHDATA*)buf;

  CY_SMIF_HB_DEVSTATUS hr;
  hr = CY_SMIF_HbFlash_WriteBufferProgramOp(g_base_addr, offset, sz >> 1, data);
  return (hr != CY_SMIF_DEV_NOT_BUSY);
}
