
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
#define SMIF_DEVICE_USED SMIF0_DEVICE1
#define HB_INITIAL_LC    CY_SMIF_HB_LC4
/**************************************/

static volatile CY_SMIF_FLASHDATA* g_base_addr;
static cy_stc_smif_context_t smif_ctxt;

static const cy_stc_smif_config_t smif_config = {
    .txClk         = CY_SMIF_INV_FOR_DDR,
    .rxClk         = CY_SMIF_INV_RWDS,
    .dlpAuto       = CY_SMIF_DLP_UPDATE_MANUAL,
    .capDelay      = CY_SMIF_CAPTURE_DELAY_0_CYCLE,
    .delaySel      = CY_SMIF_1_SEN_SEL_PER_TAP,
    .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
    .setupDelay    = CY_SMIF_SETUP_3_CLK_PULUS_MIN,
    .holdDelay     = CY_SMIF_HOLD_1_CLK_PULUS_MIN,
    .mode          = CY_SMIF_MEMORY,
    .blockEvent    = CY_SMIF_BUS_ERROR,
};

static cy_stc_device_hb_config_t smif_dev_hbsram_config = {
    .xipReadCmd         = CY_SMIF_HB_READ_CONTINUOUS_BURST,
    .xipWriteCmd        = CY_SMIF_HB_WRITE_CONTINUOUS_BURST,
    .mergeEnable        = false,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_1_CYCLE,
    .totalTimeoutEnable = false,
    .totalTimeout       = 1000u,
    .addr               = 0,  // will be updated in the application
    .size               = CY_SMIF_DEVICE_8M_BYTE,
    .lc_hb              = HB_INITIAL_LC,
    .hbDevType          = CY_SMIF_HB_SRAM,
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

#define INIT_ATTRS __attribute__((noreturn, used, __section__(".start")))

INIT_ATTRS int Init() {
  SystemInit();
  Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
  ChangePLLFrequency(66000000);

  smif_port_init(smifPortCfg, SIZE_OF_PORT);

  Cy_SMIF_Init(SMIF_USED, &smif_config, 1000, &smif_ctxt);
  Cy_SMIF_Set_DelayTapSel(SMIF_DEVICE_USED, (CY_SMIF_GetDelayTapsNumber(SMIF_USED) - 1));
  Cy_SMIF_Enable(SMIF_USED, &smif_ctxt);
  Cy_SMIF_CacheInvalidate(SMIF_USED, CY_SMIF_CACHE_BOTH);
  Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH);

  smif_dev_hbsram_config.addr = CY_SMIF_GetXIP_Address(SMIF_USED) + 0x04000000ul;
  g_base_addr                 = (volatile CY_SMIF_FLASHDATA*)smif_dev_hbsram_config.addr;

  Cy_SMIF_InitDeviceHyperBus(SMIF_DEVICE_USED, &smif_dev_hbsram_config);

  cy_un_h_ram_cfg0_reg_t wVCR = {.u16 = CY_SMIF_CFG_REG0_DEFAULT_S27KXXXX2};
  wVCR.fld.readLatency        = smif_dev_hbsram_config.lc_hb;
  CY_SMIF_WriteHYPERRAM_REG(SMIF_DEVICE_USED, CY_SMIF_CFG_REG0_ADDR_S27K, wVCR.u16);

  uint16_t rVCR = 0;
  CY_SMIF_ReadHYPERRAM_REG(SMIF_DEVICE_USED, CY_SMIF_CFG_REG0_ADDR_S27K, &rVCR);
  ASSERT(rVCR == wVCR.u16);

  // Cy_SMIF_CacheInvalidate(SMIF_USED, CY_SMIF_CACHE_BOTH);
  // Cy_SMIF_CacheEnable(SMIF_USED, CY_SMIF_CACHE_BOTH);

  for (;;)
    __BKPT(0);
}
