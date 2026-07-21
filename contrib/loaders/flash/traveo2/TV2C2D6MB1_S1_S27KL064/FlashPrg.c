
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
#include "../sdl/tviic2d6m/src/mw/smif_mem/cy_smif_hb_flash.c"
#include "../sdl/tviic2d6m/src/mw/smif_mem/cy_smif_semp.c"
#include "../sdl/tviic2d6m/src/system/rev_c/system_tviic2d6m_cm0plus.c"
#include "../sdl/tviic2d6m/src/drivers/syspmic/cy_syspmic.c"
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

/**************************************/

cy_stc_smif_dll_config_t smifDllConfig = {
    .pllFreq    = 0, /* will be updated in runtime */
    .mdlOutDiv  = CY_SMIF_MDL_CLK_OUT_DIV16,
    .mdlTapSel  = CY_SMIF_DDL_7_TAP_DELAY,
    .rxCapMode  = CY_SMIF_CAP_MODE_RWDS,
    .txSdrExtra = CY_SMIF_TX_TWO_PERIOD_AHEAD,
};

cy_stc_smif_config_t smifConfig = {
    .txClk         = CY_SMIF_INV_FOR_DDR,
    .rxClk         = CY_SMIF_INV_RWDS,
    .dlpAuto       = CY_SMIF_DLP_UPDATE_MANUAL,
    .capDelay      = CY_SMIF_CAPTURE_DELAY_1_CYCLE,
    .delaySel      = CY_SMIF_1_SEN_SEL_PER_TAP,
    .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
    .setupDelay    = CY_SMIF_SETUP_3_CLK_PULUS_MIN,
    .holdDelay     = CY_SMIF_HOLD_1_CLK_PULUS_MIN,
    .mode          = CY_SMIF_MEMORY,
    .blockEvent    = CY_SMIF_BUS_ERROR,
    .pDllCfg       = &smifDllConfig,
};
cy_stc_smif_context_t smifContext;

#define HB_INITIAL_LC CY_SMIF_HB_LC6  // CY_SMIF_HB_LC4

cy_stc_device_hb_config_t smifDevHBSramCfg = {
    .xipReadCmd         = CY_SMIF_HB_READ_CONTINUOUS_BURST,
    .xipWriteCmd        = CY_SMIF_HB_WRITE_CONTINUOUS_BURST,
    .mergeEnable        = true,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_4096_CYCLE,
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
  {CY_SMIF_CLK_PORT,         CY_SMIF_CLK_PIN,       CY_SMIF_CLK_PIN_MUX,     CY_GPIO_DM_STRONG},
#if (CY_USE_PSVP == 0)
  {CY_SMIF_CLK_INV_PORT,     CY_SMIF_CLK_INV_PIN,   HSIOM_SEL_GPIO,          CY_GPIO_DM_STRONG_IN_OFF},
#endif
  {CY_SMIF_RWDS_PORT,        CY_SMIF_RWDS_PIN,      CY_SMIF_RWDS_PIN_MUX,    CY_GPIO_DM_STRONG},
  {CY_SMIF_SELECT0_PORT,     CY_SMIF_SELECT0_PIN,   CY_SMIF_SELECT0_PIN_MUX, CY_GPIO_DM_PULLUP},
  {CY_SMIF_SELECT1_PORT,     CY_SMIF_SELECT1_PIN,   CY_SMIF_SELECT1_PIN_MUX, CY_GPIO_DM_PULLUP},
  {CY_SMIF_DATA0_PORT,       CY_SMIF_DATA0_PIN,     CY_SMIF_DATA0_PIN_MUX,   CY_GPIO_DM_STRONG},
  {CY_SMIF_DATA1_PORT,       CY_SMIF_DATA1_PIN,     CY_SMIF_DATA1_PIN_MUX,   CY_GPIO_DM_STRONG},
  {CY_SMIF_DATA2_PORT,       CY_SMIF_DATA2_PIN,     CY_SMIF_DATA2_PIN_MUX,   CY_GPIO_DM_STRONG},
  {CY_SMIF_DATA3_PORT,       CY_SMIF_DATA3_PIN,     CY_SMIF_DATA3_PIN_MUX,   CY_GPIO_DM_STRONG},
  {CY_SMIF_DATA4_PORT,       CY_SMIF_DATA4_PIN,     CY_SMIF_DATA4_PIN_MUX,   CY_GPIO_DM_STRONG},
  {CY_SMIF_DATA5_PORT,       CY_SMIF_DATA5_PIN,     CY_SMIF_DATA5_PIN_MUX,   CY_GPIO_DM_STRONG},
  {CY_SMIF_DATA6_PORT,       CY_SMIF_DATA6_PIN,     CY_SMIF_DATA6_PIN_MUX,   CY_GPIO_DM_STRONG},
  {CY_SMIF_DATA7_PORT,       CY_SMIF_DATA7_PIN,     CY_SMIF_DATA7_PIN_MUX,   CY_GPIO_DM_STRONG},
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

uint32_t gPllFrequency = 166000000ul;
#define SMIF_DEVICE_USED SMIF_DEVICE1

#define INIT_ATTRS __attribute__((noreturn, used, __section__(".start")))
INIT_ATTRS int Init() {
  SystemInit();

  smifConfig.pDllCfg = &smifDllConfig;

  __enable_irq();

  /* Should be called before enabling PLL or before modifing PLL frequency */
  Cy_SMIF_DeInit(SMIF_USED);

  /* Enable PLL clock */
  Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
  ChangePLLFrequency(gPllFrequency);

  /*************************************/
  /*       SMIF Initialization         */
  /*************************************/
  smif_port_init(smifPortCfg, SIZE_OF_PORT);
  smifDllConfig.pllFreq = gPllFrequency;
  Cy_SMIF_Init(SMIF_USED, &smifConfig, 1000, &smifContext);
  Cy_SMIF_Enable(SMIF_USED, &smifContext);

  while (Cy_SMIF_IsDllLocked(SMIF_USED) == false) {};

  Cy_SMIF_CacheInvalidate(SMIF_USED, CY_SMIF_CACHE_BOTH);  // Only affects CM0+ side
  Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH);     // this is required when checking status register of external device. Only affects CM0+ side

  volatile uint32_t* pHyperRamBaseAddr = (uint32_t*)(CY_SMIF_GetXIP_Address(SMIF_USED) + 0x04000000ul);

  /*************************************/
  /*     SMIF DEVICE Initialization    */
  /*************************************/
  smifDevHBSramCfg.addr = (uint32_t)pHyperRamBaseAddr;
  Cy_SMIF_InitDeviceHyperBus(SMIF_DEVICE_USED, &smifDevHBSramCfg);
  Cy_SMIF_DeviceSetRxCaptureDdr(SMIF_DEVICE_USED, CY_SMIF_RX_CAP_STYLE_DDR_HYPERBUS);

  cy_un_h_ram_cfg0_reg_t ramVCR = {.u16 = CY_SMIF_CFG_REG0_DEFAULT_S27KXXXX1};
  ramVCR.fld.readLatency        = smifDevHBSramCfg.lc_hb;
  CY_SMIF_WriteHYPERRAM_REG(SMIF_DEVICE_USED, CY_SMIF_CFG_REG0_ADDR_S27K, ramVCR.u16);

  uint16_t readVCRValue0 = 0;
  CY_SMIF_ReadHYPERRAM_REG(SMIF_DEVICE_USED, CY_SMIF_CFG_REG0_ADDR_S27K, &readVCRValue0);
  ASSERT(readVCRValue0 == ramVCR.u16);

  // Cy_SMIF_CacheEnable(SMIF_USED, CY_SMIF_CACHE_BOTH);             // Only affects CM0+ side
  // Cy_SMIF_CachePrefetchingEnable(SMIF_USED, CY_SMIF_CACHE_BOTH);  // Only affects CM0+ side

  while (Cy_SMIF_IsBusy(SMIF_USED)) {};

  for (;;)
    __BKPT(0);
}
