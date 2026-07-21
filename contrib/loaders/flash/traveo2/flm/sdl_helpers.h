#pragma once

#include "cy_device_headers.h"
#include "cy_project.h"

typedef enum {
  SEMP_S25H,
  SEMP_S26H,
  SEMP_S28H,
} semper_memory_type_t;

typedef enum {
  SECTORS_UNIFORM,
  SECTORS_HYBRID_B,
  SECTORS_HYBRID_T,
  SECTORS_HYBRID_BT,
} sector_arch_t;

extern const cy_stc_smif_semp_id_t s26hx512t_id;
extern const cy_stc_smif_semp_id_t s26hx01gt_id;
extern const cy_stc_smif_semp_id_t s28hx512t_id;

void smif_set_mode(volatile cy_stc_smif_reg_t* base, cy_en_smif_mode_t mode);

bool semp_wait_ready(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                     cy_en_smif_semp_reg_latency_code_t reg_lc, cy_en_smif_trx_type_t trx_type,
                     cy_stc_smif_context_t* ctxt);

cy_en_smif_status_t semp_reset(volatile cy_stc_smif_reg_t* smif, cy_en_smif_slave_select_t slave,
                               cy_en_smif_trx_type_t trx_type, cy_en_smif_semp_reg_latency_code_t reg_latency,
                               cy_stc_smif_context_t* ctxt);

cy_en_smif_status_t semp_detect_and_reset(volatile cy_stc_smif_reg_t* smif, cy_en_smif_slave_select_t slave,
                                          const cy_stc_smif_semp_id_t* expected_id, cy_stc_smif_context_t* ctxt);

cy_en_smif_status_t semp_write_any_reg(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                                       cy_en_smif_semp_reg_addr_t addr, uint8_t value,
                                       cy_en_smif_trx_addr_len_t addr_len, cy_en_smif_semp_reg_latency_code_t reg_lc,
                                       cy_en_smif_trx_type_t trx_type, cy_stc_smif_context_t* ctxt);

cy_en_smif_status_t semp_detect(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                                uint8_t target_mem_rd_lat, const cy_stc_smif_semp_id_t* expected_id,
                                cy_en_smif_trx_type_t* trx_type, cy_en_smif_semp_reg_latency_code_t* reg_lc,
                                cy_stc_smif_context_t* ctxt);

cy_en_smif_status_t semp_setup_spi_regs(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                                        cy_en_smif_semp_reg_latency_code_t current_reg_lc,
                                        cy_en_smif_semp_reg_latency_code_t target_reg_lc,
                                        cy_en_smif_semp_read_latency_code_t target_read_lc,
                                        cy_en_smif_trx_type_t current_trx_type, semper_memory_type_t mem_type,
                                        sector_arch_t sector_arch, cy_stc_smif_context_t* ctxt);

cy_en_smif_status_t semp_setup_hb_regs(volatile CY_SMIF_FLASHDATA* base, sector_arch_t sector_arch,
                                       uint8_t target_hb_read_lc);

cy_en_smif_status_t semp_s26h_enter_hb_mode(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                                            cy_en_smif_semp_reg_latency_code_t reg_lc, cy_en_smif_trx_type_t trx_type,
                                            cy_stc_smif_context_t* ctxt);

cy_en_smif_status_t semp_s28h_enter_ospi_mode(volatile cy_stc_smif_reg_t* base, cy_en_smif_slave_select_t slave,
                                              cy_en_smif_semp_reg_latency_code_t reg_lc, cy_en_smif_trx_type_t trx_type, bool use_ddr,
                                              cy_stc_smif_context_t* ctxt);
