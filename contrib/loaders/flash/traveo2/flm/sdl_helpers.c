#include "sdl_helpers.h"

#include <string.h>

const cy_stc_smif_semp_id_t s26hx512t_id = {
    0x34, 0x00, 0x6A, 0x00, 0x1A, 0x00,
};

const cy_stc_smif_semp_id_t s26hx01gt_id = {
    0x34, 0x00, 0x7b, 0x00, 0x1b, 0x00,
};

const cy_stc_smif_semp_id_t s28hx512t_id = {
    0x34, 0x5B, 0x1A, 0x0F, 0x03, 0x90,
};

bool semp_wait_ready(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                     cy_en_smif_semp_reg_latency_code_t reg_lc, cy_en_smif_trx_type_t trx_type,
                     cy_stc_smif_context_t* ctxt) {
  while (1) {
    un_cy_smif_semp_STR1_t sr1;
    Cy_SMIF_SEMP_ReadStatusRegister1(base, slave, &sr1.u8, reg_lc, trx_type, ctxt);
    // After transfer from Legacy SPI to OCTAL(SDR/DDR). RDYBSY may be also "0".
    if (sr1.field.u1ERSERR == 1 || sr1.field.u1PRGERR == 1)
      return 1;

    if (sr1.field.u1RDYBSY == 0)
      break;
  }

  return 0;
}

cy_en_smif_status_t semp_write_any_reg(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                                       cy_en_smif_semp_reg_addr_t addr, uint8_t value,
                                       cy_en_smif_trx_addr_len_t addr_len,
                                       cy_en_smif_semp_reg_latency_code_t reg_lc,
                                       cy_en_smif_trx_type_t trx_type, cy_stc_smif_context_t* ctxt) {
  cy_en_smif_status_t hr;

  hr = Cy_SMIF_SEMP_WriteEnable(base, slave, trx_type, ctxt);
  if (hr != CY_SMIF_SUCCESS)
    return hr;

  hr = Cy_SMIF_SEMP_WriteAnyRegister(base, slave, value, addr, addr_len, trx_type, ctxt);
  if (hr != CY_SMIF_SUCCESS)
    return hr;

  return semp_wait_ready(base, slave, reg_lc, trx_type, ctxt) ? CY_SMIF_GENERAL_ERROR : CY_SMIF_SUCCESS;
}

static void _latensy_mismatch_fixup(uint8_t* iData, uint8_t* oData, uint32_t size, uint8_t mismatchCount) {
  uint8_t mask = ~(1 << ((8 - mismatchCount) - 1));
  for (uint32_t i = 0; i < size; i += 1) {
    oData[i] = (iData[i] << mismatchCount) + ((iData[i + 1] & mask) >> (8 - mismatchCount));
  }
}

void smif_set_mode(volatile cy_stc_smif_reg_t* base, cy_en_smif_mode_t mode) {
  while (Cy_SMIF_SetMode(base, mode) != CY_SMIF_SUCCESS) {};
}

cy_en_smif_status_t semp_detect(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                                uint8_t target_mem_rd_lat, const cy_stc_smif_semp_id_t* expected_id,
                                cy_en_smif_trx_type_t* trx_type, cy_en_smif_semp_reg_latency_code_t* reg_lc,
                                cy_stc_smif_context_t* ctxt) {
  cy_en_smif_status_t hr;

  uint8_t id_1s[7]  = {0};
  uint8_t id_8s[9]  = {0};
  uint8_t id_8d[12] = {0};

  const cy_en_smif_semp_reg_latency_code_t test_lc = CY_SMIF_SEMP_REG_LATENCY0;
  Cy_SMIF_SEMP_ReadId(base, slave, (void*)id_1s, test_lc, CY_SPI_TRANSACTION_1S1S1S, sizeof(id_1s), ctxt);
  Cy_SMIF_SEMP_ReadId(base, slave, (void*)id_8s, test_lc, CY_SPI_TRANSACTION_8S8S8S, sizeof(id_8s), ctxt);
  Cy_SMIF_SEMP_ReadId(base, slave, (void*)id_8d, test_lc, CY_SPI_TRANSACTION_8D8D8D, sizeof(id_8d), ctxt);

  uint8_t readID_1S_0[6];
  uint8_t readID_1S_1[6];

  _latensy_mismatch_fixup(id_1s, readID_1S_0, sizeof(readID_1S_0), 1);
  _latensy_mismatch_fixup(id_1s, readID_1S_1, sizeof(readID_1S_1), 2);

  if (memcmp(&id_1s[0], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_1S1S1S;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY0;  // or CY_SMIF_SEMP_REG_LATENCY1. will decide below.
  } else if (memcmp(&readID_1S_0[0], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_1S1S1S;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY2;
  } else if (memcmp(&readID_1S_1[0], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_1S1S1S;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY3;
  } else if (memcmp(&id_8s[0], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_8S8S8S;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY0;
  } else if (memcmp(&id_8s[1], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_8S8S8S;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY1;
  } else if (memcmp(&id_8s[2], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_8S8S8S;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY2;
  } else if (memcmp(&id_8s[3], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_8S8S8S;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY3;
  } else if (memcmp(&id_8d[0], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_8D8D8D;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY0;
  } else if (memcmp(&id_8d[2], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_8D8D8D;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY1;
  } else if (memcmp(&id_8d[4], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_8D8D8D;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY2;
  } else if (memcmp(&id_8d[6], expected_id, sizeof(cy_stc_smif_semp_id_t)) == 0) {
    *trx_type = CY_SPI_TRANSACTION_8D8D8D;
    *reg_lc   = CY_SMIF_SEMP_REG_LATENCY3;
  } else {
    return CY_SMIF_GENERAL_ERROR;
  }

  // At here, we don't know the device setting of address length
  // then try both 4 byte address and 3 byte address anyway!!
  un_cy_smif_semp_CFR2_t wCR2 = {0};
  wCR2.field.u4MEMLAT         = target_mem_rd_lat;
  wCR2.field.u1ADRBYT         = CY_TRX_ADDR_4BYTE;

  semp_write_any_reg(base, slave, CY_SEMP_REG_ADDR_CFR2_V, wCR2.u8, CY_TRX_ADDR_4BYTE, *reg_lc, *trx_type,
                     ctxt);
  semp_write_any_reg(base, slave, CY_SEMP_REG_ADDR_CFR2_V, wCR2.u8, CY_TRX_ADDR_3BYTE, *reg_lc, *trx_type,
                     ctxt);

  if ((*trx_type == CY_SPI_TRANSACTION_1S1S1S) && (*reg_lc == CY_SMIF_SEMP_REG_LATENCY0)) {
    un_cy_smif_semp_CFR2_t rCR2 = {0};
    hr = Cy_SMIF_SEMP_ReadAnyVolatileRegister(base, slave, &rCR2.u8, CY_SEMP_REG_ADDR_CFR2_V,
                                              CY_SMIF_SEMP_REG_LATENCY0, CY_TRX_ADDR_4BYTE,
                                              CY_SPI_TRANSACTION_1S1S1S, ctxt);
    if (hr != CY_SMIF_SUCCESS)
      return hr;

    if (rCR2.u8 == wCR2.u8) {
      *reg_lc = CY_SMIF_SEMP_REG_LATENCY0;
      return CY_SMIF_SUCCESS;
    }

    hr = Cy_SMIF_SEMP_ReadAnyVolatileRegister(base, slave, &rCR2.u8, CY_SEMP_REG_ADDR_CFR2_V,
                                              CY_SMIF_SEMP_REG_LATENCY1, CY_TRX_ADDR_4BYTE,
                                              CY_SPI_TRANSACTION_1S1S1S, ctxt);
    if (hr != CY_SMIF_SUCCESS)
      return hr;

    if (rCR2.u8 == wCR2.u8) {
      *reg_lc = CY_SMIF_SEMP_REG_LATENCY1;
      return CY_SMIF_SUCCESS;
    }

    return CY_SMIF_GENERAL_ERROR;
  }

  return CY_SMIF_SUCCESS;
}

cy_en_smif_status_t semp_setup_hb_regs(volatile CY_SMIF_FLASHDATA* base, sector_arch_t sector_arch,
                                       uint8_t target_hb_read_lc) {
  cy_un_h_flash_cfg1_reg_t wCR1 = {.u16 = CFG_REG1_DEFAULT_S26H};
  wCR1.fld.readLatency          = target_hb_read_lc;
  cy_un_h_flash_cfg2_reg_t wCR2 = {.u16 = CFG_REG2_DEFAULT_S26H};

  switch (sector_arch) {
    case SECTORS_HYBRID_B:
      wCR1.fld.sectorMapping  = 0;
      wCR2.fld.split4KBSector = 0;
      break;
    case SECTORS_HYBRID_T:
      wCR1.fld.sectorMapping  = 1;
      wCR2.fld.split4KBSector = 0;
      break;
    case SECTORS_HYBRID_BT:
      wCR1.fld.sectorMapping  = 0;
      wCR2.fld.split4KBSector = 1;
      break;
    default:
    case SECTORS_UNIFORM:
      wCR1.fld.sectorMapping  = 2;
      wCR2.fld.split4KBSector = 0;
      break;
  }

  CY_SMIF_HbFlash_LoadVolatileConfigReg(base, wCR1.u16);
  CY_SMIF_HbFlash_LoadVolatileConfigReg2(base, wCR2.u16);

  cy_un_h_flash_cfg1_reg_t rCR1;
  cy_un_h_flash_cfg2_reg_t rCR2;

  rCR1.u16 = CY_SMIF_HbFlash_ReadNonVolatileConfigReg(base);
  rCR2.u16 = CY_SMIF_HbFlash_ReadNonVolatileConfigReg2(base);

  if (rCR1.u16 != wCR1.u16 || rCR2.u16 != wCR2.u16) {
    CY_SMIF_FLASHDATA hr;
    CY_SMIF_HbFlash_EraseNonVolatileConfigReg(base);

    hr = CY_SMIF_HbFlash_Poll(base, 0);
    if ((hr & CY_SMIF_DEV_ERASE_MASK) == CY_SMIF_DEV_ERASE_MASK)
      return CY_SMIF_GENERAL_ERROR;

    /* Program NVCR1 */
    CY_SMIF_HbFlash_ProgramNonVolatileConfigReg(base, wCR1.u16);
    hr = CY_SMIF_HbFlash_Poll(base, 0);
    if ((hr & CY_SMIF_DEV_PROGRAM_MASK) == CY_SMIF_DEV_PROGRAM_MASK)
      return CY_SMIF_GENERAL_ERROR;

    /* Program NVCR2 */
    CY_SMIF_HbFlash_ProgramNonVolatileConfigReg2(base, wCR2.u16);
    hr = CY_SMIF_HbFlash_Poll(base, 0);
    if ((hr & CY_SMIF_DEV_PROGRAM_MASK) == CY_SMIF_DEV_PROGRAM_MASK)
      return CY_SMIF_GENERAL_ERROR;
  }

  rCR1.u16 = CY_SMIF_HbFlash_ReadVolatileConfigReg(base);
  rCR2.u16 = CY_SMIF_HbFlash_ReadVolatileConfigReg2(base);

  return (rCR1.u16 == wCR1.u16 && rCR2.u16 == wCR2.u16) ? CY_SMIF_SUCCESS : CY_SMIF_GENERAL_ERROR;
}

cy_en_smif_status_t semp_setup_spi_regs(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                                        cy_en_smif_semp_reg_latency_code_t current_reg_lc,
                                        cy_en_smif_semp_reg_latency_code_t target_reg_lc,
                                        cy_en_smif_semp_read_latency_code_t target_read_lc,
                                        cy_en_smif_trx_type_t current_trx_type, semper_memory_type_t mem_type,
                                        sector_arch_t sector_arch, cy_stc_smif_context_t* ctxt) {
  cy_en_smif_status_t hr;
  /*************************************************************************************************************/
  {
    un_cy_smif_semp_CFR1_t wCR1 = {0};
    switch (sector_arch) {
      case SECTORS_HYBRID_T:
        wCR1.field.u1TB4KBS = 1;
        wCR1.field.u1SP4KBS = 0;
        break;
      case SECTORS_HYBRID_BT:
        wCR1.field.u1TB4KBS = 0;
        wCR1.field.u1SP4KBS = 1;
        break;
      case SECTORS_UNIFORM:
      case SECTORS_HYBRID_B:
      default:
        wCR1.field.u1TB4KBS = 0;
        wCR1.field.u1SP4KBS = 0;
        break;
    }

    un_cy_smif_semp_CFR2_t rCR1 = {0};
    hr = Cy_SMIF_SEMP_ReadAnyNonVolatileRegister(base, slave, &rCR1.u8, CY_SEMP_REG_ADDR_CFR1_NV,
                                                 target_read_lc, CY_TRX_ADDR_4BYTE, current_trx_type, ctxt);
    if (hr != CY_SMIF_SUCCESS)
      return hr;

    if (wCR1.u8 != rCR1.u8) {  // Setting Configuration Register 1
      hr = semp_write_any_reg(base, slave, CY_SEMP_REG_ADDR_CFR1_NV, wCR1.u8, CY_TRX_ADDR_4BYTE,
                              current_reg_lc, current_trx_type, ctxt);
      if (hr != CY_SMIF_SUCCESS)
        return hr;

      hr = Cy_SMIF_SEMP_ReadAnyVolatileRegister(base, slave, &rCR1.u8, CY_SEMP_REG_ADDR_CFR1_V,
                                                current_reg_lc, CY_TRX_ADDR_4BYTE, current_trx_type, ctxt);
      if (hr != CY_SMIF_SUCCESS)
        return hr;

      if (wCR1.u8 != rCR1.u8)
        return CY_SMIF_GENERAL_ERROR;
    }
  }

  /*************************************************************************************************************/
  {
    un_cy_smif_semp_CFR2_t wCR2 = {0};
    wCR2.field.u4MEMLAT         = target_read_lc;
    wCR2.field.u1ADRBYT         = CY_TRX_ADDR_4BYTE;

    un_cy_smif_semp_CFR2_t rCR2 = {0};
    hr = Cy_SMIF_SEMP_ReadAnyNonVolatileRegister(base, slave, &rCR2.u8, CY_SEMP_REG_ADDR_CFR2_NV,
                                                 target_read_lc, CY_TRX_ADDR_4BYTE, current_trx_type, ctxt);
    if (hr != CY_SMIF_SUCCESS)
      return hr;
    if (wCR2.u8 != rCR2.u8) {
      hr = semp_write_any_reg(base, slave, CY_SEMP_REG_ADDR_CFR2_NV, wCR2.u8, CY_TRX_ADDR_4BYTE,
                              current_reg_lc, current_trx_type, ctxt);
      if (hr != CY_SMIF_SUCCESS)
        return hr;

      un_cy_smif_semp_CFR2_t rCR2 = {0};
      hr = Cy_SMIF_SEMP_ReadAnyVolatileRegister(base, slave, &rCR2.u8, CY_SEMP_REG_ADDR_CFR2_V,
                                                current_reg_lc, CY_TRX_ADDR_4BYTE, current_trx_type, ctxt);
      if (hr != CY_SMIF_SUCCESS)
        return hr;

      if (wCR2.u8 != rCR2.u8)
        return CY_SMIF_GENERAL_ERROR;
    }
  }

  /*************************************************************************************************************/
  {
    un_cy_smif_semp_CFR3_t wCR3 = {0};
    wCR3.field.u2VRGLAT         = target_reg_lc;
    wCR3.field.u1BLKCHK         = 1;
#if (ROW_SIZE == 512)
    wCR3.field.u1PGMBUF = 1;
#elif (ROW_SIZE == 256)
    wCR3.field.u1PGMBUF = 1;
#else
#error "ROW_SIZE should be 256 or 512!"
#endif
    switch (sector_arch) {
      case SECTORS_UNIFORM:
        wCR3.field.u1UNHYSA = 1;
        break;
      case SECTORS_HYBRID_T:
      case SECTORS_HYBRID_BT:
      case SECTORS_HYBRID_B:
      default:
        wCR3.field.u1UNHYSA = 0;
        break;
    }

    un_cy_smif_semp_CFR3_t rCR3 = {0};
    hr = Cy_SMIF_SEMP_ReadAnyNonVolatileRegister(base, slave, &rCR3.u8, CY_SEMP_REG_ADDR_CFR3_NV,
                                                 target_read_lc, CY_TRX_ADDR_4BYTE, current_trx_type, ctxt);
    if (hr != CY_SMIF_SUCCESS)
      return hr;

    if (wCR3.u8 != rCR3.u8) {  // Setting Configuration Register 3
      hr = semp_write_any_reg(base, slave, CY_SEMP_REG_ADDR_CFR3_NV, wCR3.u8, CY_TRX_ADDR_4BYTE,
                              current_reg_lc, current_trx_type, ctxt);
      if (hr != CY_SMIF_SUCCESS)
        return hr;

      hr = Cy_SMIF_SEMP_ReadAnyVolatileRegister(base, slave, &rCR3.u8, CY_SEMP_REG_ADDR_CFR3_V, target_reg_lc,
                                                CY_TRX_ADDR_4BYTE, current_trx_type, ctxt);
      if (hr != CY_SMIF_SUCCESS)
        return hr;

      if (wCR3.u8 != rCR3.u8)
        return CY_SMIF_GENERAL_ERROR;
    }
  }

  /*************************************************************************************************************/
  {
    un_cy_smif_semp_CFR4_t wCR4 = {0};
    wCR4.field.u3IOIMPD         = 5;

    un_cy_smif_semp_CFR4_t rCR4 = {0};
    Cy_SMIF_SEMP_ReadAnyNonVolatileRegister(base, slave, &rCR4.u8, CY_SEMP_REG_ADDR_CFR4_NV, target_read_lc,
                                            CY_TRX_ADDR_4BYTE, current_trx_type, ctxt);
    if (wCR4.u8 != rCR4.u8) {  // Setting Configuration Register 4
      hr = semp_write_any_reg(base, slave, CY_SEMP_REG_ADDR_CFR4_NV, wCR4.u8, CY_TRX_ADDR_4BYTE,
                              target_reg_lc, current_trx_type, ctxt);
      if (hr != CY_SMIF_SUCCESS)
        return hr;

      hr = Cy_SMIF_SEMP_ReadAnyVolatileRegister(base, slave, &rCR4.u8, CY_SEMP_REG_ADDR_CFR4_V, target_reg_lc,
                                                CY_TRX_ADDR_4BYTE, current_trx_type, ctxt);
      if (hr != CY_SMIF_SUCCESS)
        return hr;

      if (wCR4.u8 != rCR4.u8)
        return CY_SMIF_GENERAL_ERROR;
    }
  }

  /*** CFR5 only present on S28H chips
   * **************************************************************************/
  if (mem_type == SEMP_S28H) {
    un_cy_smif_semp_CFR5_t wCR5 = {0};
    wCR5.field.u6RESRVD         = 0b010000;

    un_cy_smif_semp_CFR5_t rCR5 = {0};
    Cy_SMIF_SEMP_ReadAnyNonVolatileRegister(base, slave, &rCR5.u8, CY_SEMP_REG_ADDR_CFR5_NV, target_read_lc,
                                            CY_TRX_ADDR_4BYTE, current_trx_type, ctxt);
    if (wCR5.u8 != rCR5.u8) {  // Setting Configuration Register 4
      hr = semp_write_any_reg(base, slave, CY_SEMP_REG_ADDR_CFR5_NV, wCR5.u8, CY_TRX_ADDR_4BYTE,
                              target_reg_lc, current_trx_type, ctxt);
      if (hr != CY_SMIF_SUCCESS)
        return hr;

      hr = Cy_SMIF_SEMP_ReadAnyVolatileRegister(base, slave, &rCR5.u8, CY_SEMP_REG_ADDR_CFR5_V, target_reg_lc,
                                                CY_TRX_ADDR_4BYTE, current_trx_type, ctxt);
      if (hr != CY_SMIF_SUCCESS)
        return hr;

      if (wCR5.u8 != rCR5.u8)
        return CY_SMIF_GENERAL_ERROR;
    }
  }

  return CY_SMIF_SUCCESS;
}

cy_en_smif_status_t semp_reset(volatile cy_stc_smif_reg_t* smif, cy_en_smif_slave_select_t slave,
                               cy_en_smif_trx_type_t trx_type, cy_en_smif_semp_reg_latency_code_t reg_latency,
                               cy_stc_smif_context_t* ctxt) {
  cy_en_smif_status_t hr;
  hr = Cy_SMIF_SEMP_SoftwareResetEnable(smif, slave, trx_type, ctxt);
  if (hr != CY_SMIF_SUCCESS)
    return hr;

  hr = Cy_SMIF_SEMP_SoftwareReset(smif, slave, trx_type, ctxt);
  if (hr != CY_SMIF_SUCCESS)
    return hr;

  hr = semp_wait_ready(smif, slave, reg_latency, trx_type, ctxt);
  return hr;
}

cy_en_smif_status_t semp_detect_and_reset(volatile cy_stc_smif_reg_t* smif, cy_en_smif_slave_select_t slave,
                                          const cy_stc_smif_semp_id_t* expected_id,
                                          cy_stc_smif_context_t* ctxt) {
  cy_en_smif_status_t hr;
  cy_en_smif_trx_type_t crnt_trx_type;
  cy_en_smif_semp_reg_latency_code_t crnt_reg_lc;

  /* Use default latency here */
  hr = semp_detect(smif, slave, CY_SMIF_SEMP_RD_LATENCY8, expected_id, &crnt_trx_type, &crnt_reg_lc, ctxt);
  if (hr != CY_SMIF_SUCCESS)
    return hr;

  hr = semp_reset(smif, slave, crnt_trx_type, crnt_reg_lc, ctxt);
  return hr;
}

cy_en_smif_status_t semp_s26h_enter_hb_mode(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                                            cy_en_smif_semp_reg_latency_code_t reg_lc,
                                            cy_en_smif_trx_type_t trx_type, cy_stc_smif_context_t* ctxt) {
  cy_en_smif_status_t hr;

  un_cy_smif_semp_CFR3_t wCR3 = {0};
  hr = Cy_SMIF_SEMP_ReadAnyVolatileRegister(base, slave, &wCR3.u8, CY_SEMP_REG_ADDR_CFR3_V, reg_lc,
                                            CY_TRX_ADDR_4BYTE, trx_type, ctxt);
  if (hr != CY_SMIF_SUCCESS)
    return hr;

  wCR3.field.u1INTFTP = 1;
  hr = semp_write_any_reg(base, slave, CY_SEMP_REG_ADDR_CFR3_V, wCR3.u8, CY_TRX_ADDR_4BYTE, reg_lc, trx_type,
                          ctxt);
  return hr;
}

cy_en_smif_status_t semp_s28h_enter_ospi_mode(volatile cy_stc_smif_reg_t* base,
                                              cy_en_smif_slave_select_t slave,
                                              cy_en_smif_semp_reg_latency_code_t reg_lc,
                                              cy_en_smif_trx_type_t trx_type, bool use_ddr, cy_stc_smif_context_t* ctxt) {
  cy_en_smif_status_t hr;

  un_cy_smif_semp_CFR5_t wCR5 = {0};
  hr = Cy_SMIF_SEMP_ReadAnyVolatileRegister(base, slave, &wCR5.u8, CY_SEMP_REG_ADDR_CFR5_V, reg_lc,
                                            CY_TRX_ADDR_4BYTE, trx_type, ctxt);
  if (hr != CY_SMIF_SUCCESS)
    return hr;

  wCR5.field.u1OPI_IT = 1;
  wCR5.field.u1SDRDDR = use_ddr;
  hr = semp_write_any_reg(base, slave, CY_SEMP_REG_ADDR_CFR5_V, wCR5.u8, CY_TRX_ADDR_4BYTE, reg_lc, trx_type,
                          ctxt);
  return hr;
}
