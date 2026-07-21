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
#define TEST_WRITE_LC CY_SMIF_SEMP_WT_LATENCY0
#define TEST_READ_LC  CY_SMIF_SEMP_RD_LATENCY9
// SDR SPI: up to 133MHz, SDR OCTAL: up to 133MHz, DDR OCTAL: up to 66MHz
#define TEST_READ_REG_LC CY_SMIF_SEMP_REG_LATENCY1
#define TEST_ADDR_LEN    CY_TRX_ADDR_4BYTE
#define TEST_DLP_VALUE   (0xAA)
#define TGT_PLL_FREQ     160000000u
/**************************************/

#define DATA_WIDTH_1_OR_2_BITS (0)
#define DATA_WIDTH_4_BITS      (1)
#define DATA_WIDTH             (CY_SPI_TRANSACTION_4S4S4S)
#define SLAVE_USED             (CY_SMIF_SLAVE_SELECT_0 | CY_SMIF_SLAVE_SELECT_1)

typedef struct {
  volatile stc_GPIO_PRT_t* port;
  uint8_t pin;
  en_hsiom_sel_t hsiom;
  uint32_t driveMode;
} cy_stc_smif_port_t;

// clang-format off
static cy_stc_smif_port_t smifPortCfg[] = {
  {CY_SMIF_CLK_PORT,         CY_SMIF_CLK_PIN,       CY_SMIF_CLK_PIN_MUX,     CY_GPIO_DM_STRONG},
  {CY_SMIF_CLK_INV_PORT,     CY_SMIF_CLK_INV_PIN,   HSIOM_SEL_GPIO,          CY_GPIO_DM_STRONG_IN_OFF},
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

static void SmifPortInit(cy_stc_smif_port_t cfg[], uint8_t size) {
  cy_stc_gpio_pin_config_t pinCfg = {0};
  for (uint32_t i = 0; i < size; i++) {
    pinCfg.driveMode = cfg[i].driveMode;
    pinCfg.hsiom     = cfg[i].hsiom;
    Cy_GPIO_Pin_Init(cfg[i].port, cfg[i].pin, &pinCfg);
  }
}

static cy_stc_smif_context_t smifContext;
static cy_stc_ex_dev_context_t smifDevice0Context;

static cy_stc_smif_dll_config_t dll_cfg = {
    .pllFreq    = TGT_PLL_FREQ,
    .mdlOutDiv  = CY_SMIF_MDL_CLK_OUT_DIV4,
    .mdlTapSel  = CY_SMIF_DDL_7_TAP_DELAY,
    .rxCapMode  = CY_SMIF_CAP_MODE_SPI,
    .txSdrExtra = CY_SMIF_TX_ONE_PERIOD_AHEAD,
};

static const cy_stc_smif_config_t smifConfig = {
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
    .pDllCfg       = &dll_cfg,
};

// will be updated in the application
static cy_stc_device_config_t smifDev0Cfg = {
    .addr               = 0,
    .mergeEnable        = true,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_4096_CYCLE,
    .rdLatencyCode      = TEST_READ_LC,
    .size               = CY_SMIF_DEVICE_128M_BYTE,
    .totalTimeout       = 5000u,
    .totalTimeoutEnable = true,
    .writeEn            = false,
    .wtLatencyCode      = TEST_WRITE_LC,
    .xipReadCmdId       = CY_SMIF_SEMP_RDID_4S4S4S_4ADR_DLP,
    .xipReadMode        = Cy_SMIF_SEMP_MODE_NOT_CONTINUOUS,
    .xipWriteCmdId      = CY_SMIF_SEMP_WTID_4S4S4S_4ADR,
    .xipWriteMode       = 0u,
};

static void SetModeWithBusyCheck(volatile cy_stc_smif_reg_t* base, cy_en_smif_mode_t mode) {
  cy_en_smif_status_t status;
  do {
    status = Cy_SMIF_SetMode(base, mode);
  } while (status != CY_SMIF_SUCCESS);
}

// return 1, when one of selected device return error during erasing.
// return 0, when erasing completed without error.
static bool WaitDeviceBecomesReady(cy_en_smif_slave_select_t slave, cy_en_smif_semp_reg_latency_code_t regLc, cy_en_smif_trx_type_t transMode) {
  for (uint8_t i_slaveNum = 0; i_slaveNum < CY_SMIF_GetDeviceNumber(SMIF_USED); i_slaveNum++) {
    cy_en_smif_slave_select_t slaveToBeChecked = ((uint8_t)slave & (0x01 << i_slaveNum));
    if (slaveToBeChecked == 0u)
      continue;

    // only selected slave will reach hare.
    while (1) {
      un_cy_smif_semp_STR1_t stat;
      Cy_SMIF_SEMP_ReadStatusRegister1(SMIF_USED, slaveToBeChecked, &stat.u8, regLc, transMode, &smifContext);

      // After transfer from Legacy SPI to OCTAL(SDR/DDR). RDYBSY may be also "0".
      if (stat.field.u1ERSERR == 1)
        return 1;

      if (stat.field.u1PRGERR == 1)
        return 1;

      if (stat.field.u1RDYBSY == 0)
        break;
    }
  }

  return 0;
}

static uint32_t WriteRegisterSequence(cy_en_smif_slave_select_t slave, cy_en_smif_semp_reg_addr_t addr, uint8_t value, cy_en_smif_trx_addr_len_t addrLen,
                                      cy_en_smif_semp_reg_latency_code_t regLc, cy_en_smif_trx_type_t transMode) {
  ASSERT(Cy_SMIF_SEMP_WriteEnable(SMIF_USED, slave, transMode, &smifContext) == CY_SMIF_SUCCESS);
  ASSERT(Cy_SMIF_SEMP_WriteAnyRegister(SMIF_USED, slave, value, addr, addrLen, transMode, &smifContext) == CY_SMIF_SUCCESS);
  ASSERT(WaitDeviceBecomesReady(slave, regLc, transMode) == 0);
  return 0;
}

__attribute__((used)) int Init(unsigned long adr, unsigned long clk, unsigned long fnc) {
  (void)adr;
  (void)clk;
  (void)fnc;
  cy_en_smif_status_t hr;

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

  SystemInit();
  Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
  ChangePLLFrequency(TGT_PLL_FREQ);

  SmifPortInit(smifPortCfg, SIZE_OF_PORT);
  ASSERT(Cy_SMIF_InitExMemContext(CY_SMIF_SEMP, &smifDevice0Context) == CY_SMIF_SUCCESS);

  Cy_SMIF_DeInit(SMIF_USED);
  hr = Cy_SMIF_Init(SMIF_USED, &smifConfig, 1000, &smifContext);
  ASSERT(hr == CY_SMIF_SUCCESS);

  /************************************/
  /* 2. Setting for SMIF 0 Device 0/1 */
  /************************************/
  Cy_SMIF_DeviceSetDataSelect(SMIF_DEVICE0, CY_SMIF_DATA_SEL0);
  Cy_SMIF_DeviceWriteEnable(SMIF_DEVICE0);
  Cy_SMIF_DeviceSetDataSelect(SMIF_DEVICE1, CY_SMIF_DATA_SEL2);
  Cy_SMIF_DeviceWriteEnable(SMIF_DEVICE1);

  /********************/
  /* 3. Enable SMIF 0 */
  /********************/
  Cy_SMIF_Enable(SMIF_USED, &smifContext);
  for (size_t i = 0; i < 100000 && !Cy_SMIF_IsDllLocked(SMIF_USED); i++) {};
  ASSERT(Cy_SMIF_IsDllLocked(SMIF_USED));

  /********* Issue Software Reset ******/
  Cy_SMIF_SEMP_SoftwareResetEnable(SMIF_USED, SLAVE_USED, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  Cy_SMIF_SEMP_SoftwareReset(SMIF_USED, SLAVE_USED, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  WaitDeviceBecomesReady(SLAVE_USED, CY_SMIF_SEMP_REG_LATENCY0, CY_SPI_TRANSACTION_1S1S1S);

  /*********************************************************/
  /********* Write Status and Configuration registers ******/
  /*********************************************************/

  /*** Write Configuration register ***/
  // Setting Status Register 1
  un_cy_smif_semp_STR1_t wSR1 = {0};

  // Setting Configuration Register 1
  un_cy_smif_semp_CFR1_t wCR1 = {0};
  wCR1.field.u1QUADIT         = DATA_WIDTH_1_OR_2_BITS;

  // Setting Configuration Register 2
  un_cy_smif_semp_CFR2_t wCR2 = {0};
  wCR2.field.u4MEMLAT         = TEST_READ_LC;
  wCR2.field.u1QPI_IT         = DATA_WIDTH_4_BITS;
  wCR2.field.u1ADRBYT         = TEST_ADDR_LEN;

  // Setting Configuration Register 3
  un_cy_smif_semp_CFR3_t wCR3 = {0};
  wCR3.field.u2VRGLAT         = TEST_READ_REG_LC;
  wCR3.field.u1UNHYSA         = 1;
  wCR3.field.u1PGMBUF         = 1;
  wCR3.field.u1BLKCHK         = 1;

  // Setting Configuration Register 4
  un_cy_smif_semp_CFR4_t wCR4 = {0};
  wCR4.field.u1RBSTWP         = 0;

  // Write Enable Volatile register
  // !! Attention if "Write Enable" command issued, following
  //    "Write Register" command will write to Non-Volatile registers.
  Cy_SMIF_SEMP_WriteEnable(SMIF_USED, SLAVE_USED, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  Cy_SMIF_SEMP_WriteRegister(SMIF_USED, SLAVE_USED, wSR1.u8, wCR1.u8, wCR2.u8, wCR3.u8, wCR4.u8, CY_SPI_TRANSACTION_1S1S1S, &smifContext);
  WaitDeviceBecomesReady(SLAVE_USED, TEST_READ_REG_LC, CY_SPI_TRANSACTION_1S1S1S);

  Cy_SMIF_SEMP_WriteEnable(SMIF_USED, SLAVE_USED, CY_SPI_TRANSACTION_4S4S4S, &smifContext);
  Cy_SMIF_SEMP_WriteRegister(SMIF_USED, SLAVE_USED, wSR1.u8, wCR1.u8, wCR2.u8, wCR3.u8, wCR4.u8, CY_SPI_TRANSACTION_4S4S4S, &smifContext);
  WaitDeviceBecomesReady(SLAVE_USED, TEST_READ_REG_LC, DATA_WIDTH);

  /*** Read back and Verify Configuration register ***/
  // Read and Verify Setting Configuration Register 1
  un_cy_smif_semp_CFR1_t rCR1_0 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &rCR1_0.u8, CY_SEMP_REG_ADDR_CFR1_V, TEST_READ_REG_LC, TEST_ADDR_LEN, DATA_WIDTH,
                                       &smifContext);
  ASSERT(wCR1.u8 == rCR1_0.u8);

  un_cy_smif_semp_CFR1_t rCR1_1 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &rCR1_0.u8, CY_SEMP_REG_ADDR_CFR1_V, TEST_READ_REG_LC, TEST_ADDR_LEN, DATA_WIDTH,
                                       &smifContext);
  ASSERT(wCR1.u8 == rCR1_1.u8);

  // Read and Verify Setting Configuration Register 2
  un_cy_smif_semp_CFR2_t rCR2_0 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &rCR2_0.u8, CY_SEMP_REG_ADDR_CFR2_V, TEST_READ_REG_LC, TEST_ADDR_LEN, DATA_WIDTH,
                                       &smifContext);
  ASSERT(wCR2.u8 == rCR2_0.u8);

  un_cy_smif_semp_CFR2_t rCR2_1 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &rCR2_1.u8, CY_SEMP_REG_ADDR_CFR2_V, TEST_READ_REG_LC, TEST_ADDR_LEN, DATA_WIDTH,
                                       &smifContext);
  ASSERT(wCR2.u8 == rCR2_1.u8);

  // Read and Verify Setting Configuration Register 3
  un_cy_smif_semp_CFR3_t rCR3_0 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &rCR3_0.u8, CY_SEMP_REG_ADDR_CFR3_V, TEST_READ_REG_LC, TEST_ADDR_LEN, DATA_WIDTH,
                                       &smifContext);
  ASSERT(wCR3.u8 == rCR3_0.u8);

  un_cy_smif_semp_CFR3_t rCR3_1 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &rCR3_1.u8, CY_SEMP_REG_ADDR_CFR3_V, TEST_READ_REG_LC, TEST_ADDR_LEN, DATA_WIDTH,
                                       &smifContext);
  ASSERT(wCR3.u8 == rCR3_1.u8);

  // Read and Verify Setting Configuration Register 4
  un_cy_smif_semp_CFR4_t rCR4_0 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &rCR4_0.u8, CY_SEMP_REG_ADDR_CFR4_V, TEST_READ_REG_LC, TEST_ADDR_LEN, DATA_WIDTH,
                                       &smifContext);
  ASSERT(wCR4.u8 == rCR4_0.u8);

  un_cy_smif_semp_CFR4_t rCR4_1 = {0};
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &rCR4_1.u8, CY_SEMP_REG_ADDR_CFR4_V, TEST_READ_REG_LC, TEST_ADDR_LEN, DATA_WIDTH,
                                       &smifContext);
  ASSERT(wCR4.u8 == rCR4_1.u8);

  /*********************************************************/
  /*******          Write/Read DLP register           ******/
  /*********************************************************/
  // just for read/write test for now
  WriteRegisterSequence(SLAVE_USED, CY_SEMP_REG_ADDR_DLP_V, TEST_DLP_VALUE, TEST_ADDR_LEN, TEST_READ_REG_LC, DATA_WIDTH);
  uint8_t readbackDLP = 0u;
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &readbackDLP, CY_SEMP_REG_ADDR_DLP_V, TEST_READ_REG_LC, TEST_ADDR_LEN, DATA_WIDTH,
                                       &smifContext);
  ASSERT(readbackDLP == TEST_DLP_VALUE);

  readbackDLP = 0u;
  Cy_SMIF_SEMP_ReadAnyVolatileRegister(SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &readbackDLP, CY_SEMP_REG_ADDR_DLP_V, TEST_READ_REG_LC, TEST_ADDR_LEN, DATA_WIDTH,
                                       &smifContext);
  ASSERT(readbackDLP == TEST_DLP_VALUE);

  smifDev0Cfg.addr         = CY_SMIF_GetXIP_Address(SMIF_USED);
  smifDev0Cfg.xipReadCmdId = CY_SMIF_SEMP_RDID_4S4S4S_4ADR_DLP;
  smifDev0Cfg.xipReadMode  = Cy_SMIF_SEMP_MODE_NOT_CONTINUOUS;
  ASSERT(Cy_SMIF_InitXIPModeForExMem(SMIF_DEVICE0, CY_SMIF_DEV_D_QUAD_BIT_0To3, &smifDev0Cfg, &smifDevice0Context) == CY_SMIF_SUCCESS);
  ASSERT(Cy_SMIF_InitXIPModeForExMem(SMIF_DEVICE1, CY_SMIF_DEV_D_QUAD_BIT_4To7, &smifDev0Cfg, &smifDevice0Context) == CY_SMIF_SUCCESS);

  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_MEMORY);
  return 0;
}

__attribute__((used)) int UnInit(unsigned long fnc) {
  (void)fnc;
  return (0);
}

__attribute__((used)) int EraseSector(unsigned long adr) {
  if (adr < CY_SMIF_GetXIP_Address(SMIF_USED))
    return __LINE__;
  adr -= CY_SMIF_GetXIP_Address(SMIF_USED);

  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);
  Cy_SMIF_SEMP_WriteEnable(SMIF_USED, SLAVE_USED, DATA_WIDTH, &smifContext);

  // Issue Erase Sector
  Cy_SMIF_SEMP_Erase_256KB_Sector(SMIF_USED, SLAVE_USED, adr >> 1, DATA_WIDTH, &smifContext);
  ASSERT(WaitDeviceBecomesReady(SLAVE_USED, TEST_READ_REG_LC, DATA_WIDTH) == 0);

  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_MEMORY);

  return (0);
}

__attribute__((used)) int EraseChip(void) {
  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);
  ASSERT(Cy_SMIF_SEMP_WriteEnable(SMIF_USED, SLAVE_USED, DATA_WIDTH, &smifContext) == CY_SMIF_SUCCESS);
  ASSERT(Cy_SMIF_SEMP_EraseChip(SMIF_USED, SLAVE_USED, DATA_WIDTH, &smifContext) == CY_SMIF_SUCCESS);
  ASSERT(WaitDeviceBecomesReady(SLAVE_USED, TEST_READ_REG_LC, DATA_WIDTH) == 0);
  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_MEMORY);

  return 0;
}

__attribute__((used)) int ProgramPage(unsigned long adr, unsigned long sz, const unsigned char* buf) {
  if (adr < CY_SMIF_GetXIP_Address(SMIF_USED))
    return __LINE__;
  adr -= CY_SMIF_GetXIP_Address(SMIF_USED);

  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);

  // Write Enable
  Cy_SMIF_SEMP_WriteEnable(SMIF_USED, SLAVE_USED, DATA_WIDTH, &smifContext);
  // Write test data
  Cy_SMIF_MMIO_Program_ExMem(SMIF_USED, SLAVE_USED, CY_SMIF_SEMP_WTID_4S4S4S_4ADR, adr >> 1, sz, buf, CY_SMIF_BLOCKING, TEST_WRITE_LC,
                             Cy_SMIF_SEMP_MODE_NOT_CONTINUOUS, /* mode value */
                             &smifDevice0Context, &smifContext);
  ASSERT(WaitDeviceBecomesReady(SLAVE_USED, TEST_READ_REG_LC, DATA_WIDTH) == 0);
  SetModeWithBusyCheck(SMIF_USED, CY_SMIF_MEMORY);

  return (0);
}

/* [] END OF FILE */
