// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2019-2026, Infineon Technologies AG, or an affiliate of *
 *   Infineon Technologies AG. All rights reserved.                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#include "imp.h"
#include "flash/progress.h"
#include "helper/time_support.h"
#include "target/algorithm.h"
#include "target/arm_adi_v5.h"
#include "target/breakpoints.h"
#include "target/cortex_m.h"
#include "target/register.h"

#define PSOC4_SYSREQ_KEY1				0xB6
#define PSOC4_SYSREQ_KEY2				0xD3
#define PSOC4_SYSREQ_BIT				BIT(31)
#define PSOC4_SYSREQ_HMASTER_BIT		BIT(30)
#define PSOC4_SYSREQ_PRIVILEGED_BIT		BIT(28)
#define PSOC4_SYSREQ_NO_REMAP			(PSOC4_SYSREQ_HMASTER_BIT | BIT(27))

#define PSOC4_SROM_STATUS_MASK			0xF0000000
#define PSOC4_SROM_STATUS_SUCCEEDED		0xA0000000

#define PSOC4_CMD_GET_SILICON_ID		0x00
#define PSOC4_CMD_LOAD_LATCH			0x04
#define PSOC4_CMD_WRITE_ROW				0x05
#define PSOC4_CMD_PROGRAM_ROW			0x06
#define PSOC4_CMD_ERASE_ALL				0x0A
#define PSOC4_CMD_CHECKSUM				0x0B
#define PSOC4_CMD_WRITE_PROTECTION		0x0D
#define PSOC4_CMD_SET_IMO48				0x15
#define PSOC4_CMD_WRITE_SFLASH_ROW		0x18
#define PSOC4_CMD_BULK_ERASE			0x1D

#define PSOC4_CHIP_PROT_VIRGIN			0x00
#define PSOC4_CHIP_PROT_OPEN			0x01
#define PSOC4_CHIP_PROT_PROTECTED		0x02
#define PSOC4_CHIP_PROT_KILL			0x04

#define PSOC4_ROMTABLE_PID0				0xF0000FE0
#define PSOC4_ROMTABLE_DESIGNER			0xB4

#define NVIC_VTOR						0xE000ED08

#define SFLASH_BASE_V1					0x0FFFF000
#define SFLASH_BASE_V2					0x0FFFE000

#define BANK_ADDRESS_MASK				0xFFF00000
#define BANK_WORK_ADDR_MASKED			0x1F000000
#define BANK_SFLASH_ADDR_MASKED			0x0FF00000
#define BANK_FLASHP_ADDR_MASKED			0x90400000
#define BANK_CHIPP_ADDR_MASKED			0x90600000

#define FAULT_BASE(n)					(0x40130000 + (n) * 0x00000100)
#define FAULT_CTL(n)					(FAULT_BASE((n)) + 0x00)
#define FAULT_STATUS(n)					(FAULT_BASE((n)) + 0x0C)
#define FAULT_DATA0(n)					(FAULT_BASE((n)) + 0x10)
#define FAULT_DATA1(n)					(FAULT_BASE((n)) + 0x14)
#define FAULT_MASK0(n)					(FAULT_BASE((n)) + 0x50)

#define FAULT_STATUS_VALID_MASK			BIT(31)
#define FAULT_ENABLE_ECC_MASK			(BIT(4) | BIT(5) | BIT(7) | BIT(8))

#define PSOC4_STACK_WA_SIZE				512
#define PSOC4_IPC_TIMEOUT_MS			1500

#define GET_CPUSS(p4_info)				(p4_info->family->flags & PSOC4_LEGACY ? &cpuss_v1 : &cpuss_v2)

struct cpuss_type {
	uint32_t geometry;
	uint32_t sysreq;
	uint32_t sysarg;
};

static const struct cpuss_type cpuss_v1 = {
	.geometry = 0x400E0000,
	.sysreq = 0x40000004,
	.sysarg = 0x40000008,
};

static const struct cpuss_type cpuss_v2 = {
	.geometry = 0x40110000,
	.sysreq = 0x40100004,
	.sysarg = 0x40100008,
};

struct spcif_type {
	uint32_t mflash_size_mask;
	uint32_t mflash_size_pos;
	uint32_t sflash_size_mask;
	uint32_t sflash_size_pos;
	uint32_t macro_num_mask;
	uint32_t macro_num_pos;
	uint32_t row_size_mask;
	uint32_t rows_size_pos;
};

static const struct spcif_type spcif_v1 = {
	.mflash_size_mask = 0xffff << 0, /* 0:15 */
	.mflash_size_pos = 0,
	.sflash_size_mask = 0x0F << 16, /* 16:19 */
	.sflash_size_pos = 16,
	.macro_num_mask = 0x03 << 20, /* 20:21 */
	.macro_num_pos = 20,
	.row_size_mask = 0x03 << 22, /* 22:23 */
	.rows_size_pos = 22,
};

static const struct spcif_type spcif_v2 = {
	.mflash_size_mask = 0x3FFF << 0, /* 0:13 */
	.mflash_size_pos = 0,
	.sflash_size_mask = 0x3F << 14, /* 14:19 */
	.sflash_size_pos = 14,
	.macro_num_mask = 0x03 << 20, /* 20:21 */
	.macro_num_pos = 20,
	.row_size_mask = 0x03 << 22, /* 22:23 */
	.rows_size_pos = 22,
};

enum psoc4_family_flags {
	PSOC4_LEGACY    = BIT(0),
	PSOC4_HV        = BIT(1),
	PSOC4_48MHZ_NRQ = BIT(2),
	PSOC4_SFLASH_V2 = BIT(3),
	PSOC4_NO_SFLASH = BIT(4),
};

/**
 * Represents a data for supported PSOC4 families.
 *
 * id: Family ID taken from 40-95002 spec
 * siid_range[]: Si ID Range taken from 40-95002 spec
 * {0, 0} means that all range is available e.g E700-E7FF or 1200-12FF, see 40-95002 spec
 * name[]: DeviceSeries taken from cympnvalidator
 * spcif: described in SAS in CorePlatform sheet
 * flags: psoc4_family_flags used in the program logic and consider the specifics
 * of the devices of each family
 */
struct psoc4_family {
	uint8_t id;
	uint16_t siid_range[2];
	char name[60];
	const struct spcif_type *spcif;
	uint32_t flags;
};

// clang-format off
static struct psoc4_family psoc4_families[] = {
	{ 0x93, {0, 0},           "PSoC 4100/4200",                      &spcif_v1, PSOC4_LEGACY, }, /* has known SROM bug (CDT#184422). */
	{ 0x9A, {0, 0},           "PSoC 4000",                           &spcif_v1, 0, },            /* has known SROM bug (CDT#184422) */
	{ 0x9E, {0, 0},           "PSoC 4100/4200/PRoC BLE",             &spcif_v1, 0, },
	{ 0xA0, {0, 0},           "PSoC 4200L",                          &spcif_v1, PSOC4_48MHZ_NRQ, },
	{ 0xA1, {0, 0},           "PSoC 4100M/4200M",                    &spcif_v1, PSOC4_48MHZ_NRQ, },
	{ 0xA3, {0, 0},           "PSoC 4100/4200/PRoC BLE",             &spcif_v1, PSOC4_48MHZ_NRQ, },
	{ 0xA7, {0, 0},           "PSoC 4000DS/4200DS",                  &spcif_v1, 0, },
	{ 0xA9, {0, 0},           "PSoC 4000S/4700S",                    &spcif_v2, 0, },
	{ 0xAA, {0, 0},           "PSoC 4100/4200/PRoC BLE",             &spcif_v1, PSOC4_48MHZ_NRQ, },
	{ 0xAB, {0, 0},           "PSoC 4100S",                          &spcif_v2, 0, },
	{ 0xAC, {0, 0},           "PSoC 4100PS/PSoC Analog CP",          &spcif_v2, 0, },
	{ 0xAE, {0, 0},           "PSoC 4100/PRoC BLE II",               &spcif_v1, PSOC4_48MHZ_NRQ, },
	{ 0xB5, {0, 0},           "PSoC 4100S Plus",                     &spcif_v2, 0, },
	{ 0xB7, {0, 0},           "TrueTouch TSG7L",                     &spcif_v2, 0, },
	{ 0xB8, {0, 0},           "PSoC 4100S Plus/PSoC 4500",           &spcif_v2, 0, },
	{ 0xBE, {0, 0},           "PSoC 4100S Max",                      &spcif_v2, PSOC4_SFLASH_V2, },
	{ 0xC6, {0, 0},           "PSoC 4000T",                          &spcif_v2, 0, },
	{ 0xD0, {0, 0},           "PSOC 4100T Plus",                     &spcif_v2, 0, },

	{ 0xA4, {0, 0},           "CCG2",                                &spcif_v1, PSOC4_NO_SFLASH, },
	{ 0xA8, {0, 0},           "CCG4",                                &spcif_v1, PSOC4_NO_SFLASH, },
	{ 0xAD, {0x1D00, 0x1D1F}, "CCG3",                                &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xAD, {0x1D20, 0x1DFF}, "PMG1S2",                              &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xAF, {0x1FA0, 0x1FBF}, "PMG1B2",                              &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xAF, {0, 0},           "CCG4",                                &spcif_v1, PSOC4_NO_SFLASH, },
	{ 0xB0, {0x2020, 0x204F}, "PMG1S0",                              &spcif_v2, PSOC4_NO_SFLASH, }, /* !!! Subset of CCG3PA */
	{ 0xB0, {0x2000, 0x20FF}, "CCG3PA",                              &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xB4, {0, 0},           "CCG3PA2",                             &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xD2, {0x4200, 0x427F}, "CCG3PA3",                             &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xD2, {0x4280, 0x42FF}, "PMG1S4",                              &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xBA, {0x2A00, 0x2A1F}, "CCG6",                                &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xBA, {0x2A20, 0x2AFF}, "PMG1S1",                              &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xC0, {0, 0},           "CCG6DF",                              &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xC1, {0x3100, 0x317F}, "CCG7D",                               &spcif_v2, PSOC4_NO_SFLASH, }, /* SJZ-733 */
	{ 0xC1, {0x3180, 0x31FF}, "WLC1",                                &spcif_v2, PSOC4_NO_SFLASH, }, /* SJZ-733 */
	{ 0xC3, {0, 0},           "CCG6SF",                              &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xC5, {0, 0},           "PMG1S3",                              &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xCA, {0x3A20, 0x3A3F}, "PMG1B1",                              &spcif_v2, PSOC4_NO_SFLASH, }, /* Subset of CCG7S */
	{ 0xCA, {0, 0},           "CCG7S",                               &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xCD, {0, 0},           "NEO",                                 &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xCE, {0x3E00, 0x3E7F}, "CCG6DF_CFP",                          &spcif_v2, PSOC4_NO_SFLASH, },
	{ 0xCE, {0x3E80, 0x3EFF}, "CCG8DF_CFP",                          &spcif_v2, PSOC4_NO_SFLASH, },

	{ 0xC2, {0, 0},           "PSoC 4 HV PA/144k",                   &spcif_v2, PSOC4_HV | PSOC4_SFLASH_V2, },
	{ 0xC7, {0x3700, 0x3705}, "PSOC 4 HVPA-SPM-1.0",                 &spcif_v2, PSOC4_HV | PSOC4_SFLASH_V2, },
	{ 0xC7, {0, 0},           "PSoC 4 HV PA",                        &spcif_v2, PSOC4_HV | PSOC4_SFLASH_V2, },
	{ 0xC8, {0, 0},           "PSoC 4 HV MS 128k",                   &spcif_v2, PSOC4_HV | PSOC4_SFLASH_V2, },
	{ 0xC9, {0, 0},           "PSoC 4 HV MS 64k",                    &spcif_v2, PSOC4_HV | PSOC4_SFLASH_V2, },
	{ 0xCB, {0, 0},           "PAG2S",                               &spcif_v2, PSOC4_HV | PSOC4_SFLASH_V2, },

	{ 0xD1, {0, 0},           "P4BMS-DEMO",                          &spcif_v2, PSOC4_HV | PSOC4_SFLASH_V2, },
	{ 0xD3, {0x4306, 0x430D}, "Ez-BMS HVPA-BPM 2.0",                 &spcif_v2, PSOC4_HV | PSOC4_SFLASH_V2, },
	{ 0xD3, {0, 0},           "PSOC 4 HVPA-SPM 2.0",                 &spcif_v2, PSOC4_HV | PSOC4_SFLASH_V2, },

	{ 0, {0, 0}, " ", NULL, 0, },
};
// clang-format on

struct psoc4_info {
	uint16_t revision_id;

	uint32_t mflash_size;
	uint32_t sflash_size;
	uint32_t num_macro;
	uint32_t num_macro_unwounded;
	uint32_t row_size;
	uint32_t flctrl_idx;

	struct {
		uint32_t area_idx;
		uint32_t row_prot_addr[4];
		uint32_t chip_prot_offs;
		uint32_t siid_offs;
	} sflash;

	struct psoc4_family *family;
	bool probed;
};

static struct armv7m_algorithm g_armv7m_algo;
static struct working_area *g_stack_area;
static bool g_ecc_enabled;

/* Basic helper functions */
static inline bool is_mflash_bank(struct flash_bank *bank)
{
	return bank->base == 0;
}

static inline bool is_wflash_bank(struct flash_bank *bank)
{
	return (bank->base & BANK_ADDRESS_MASK) == BANK_WORK_ADDR_MASKED;
}

static inline bool is_sflash_bank(struct flash_bank *bank)
{
	return (bank->base & BANK_ADDRESS_MASK) == BANK_SFLASH_ADDR_MASKED;
}

static inline bool is_flashp_bank(struct flash_bank *bank)
{
	return (bank->base & BANK_ADDRESS_MASK) == BANK_FLASHP_ADDR_MASKED;
}

static inline bool is_chipp_bank(struct flash_bank *bank)
{
	return (bank->base & BANK_ADDRESS_MASK) == BANK_CHIPP_ADDR_MASKED;
}

static inline bool is_psoc4hv(struct flash_bank *bank)
{
	struct psoc4_info *p4_info = bank->driver_priv;
	return p4_info->family->flags & PSOC4_HV;
}

static char *psoc4_family_name(uint16_t family_id, uint16_t siid)
{
	struct psoc4_family *p = psoc4_families;
	while (p->id) {
		if (p->id == family_id && p->siid_range[0] == 0 && p->siid_range[1] == 0)
			break;
		if (p->id == family_id && siid >= p->siid_range[0] && siid <= p->siid_range[1])
			break;
		p++;
	}

	return p->name;
}

/** **********************************************************************************************
 * @brief Disables 0x0000:0000 - 0x0000:0007 remap to ROM
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_disable_remap(struct target *target)
{
	struct flash_bank *bank;
	int hr = get_flash_bank_by_addr(target, 0, true, &bank);
	if (hr != ERROR_OK)
		return ERROR_FAIL;

	/* Disable 0x0000:0000 - 0x0000:0007 remap to ROM */
	struct psoc4_info *info = bank->driver_priv;
	return target_write_u32(target, GET_CPUSS(info)->sysreq, PSOC4_SYSREQ_NO_REMAP);
}

/** **********************************************************************************************
 * @brief translates given address of the flash bank to it's macro number
 * @param bank flash bank
 * @param address absolute address within given bank
 * @return number of the flash macro
 *************************************************************************************************/
static uint8_t psoc4_addr_to_macro(struct flash_bank *bank, uint32_t address)
{
	struct psoc4_info *p4_info = bank->driver_priv;
	uint32_t die_base;
	uint32_t unwounded_size;

	if ((address & BANK_ADDRESS_MASK) == BANK_SFLASH_ADDR_MASKED) {
		die_base = p4_info->family->flags & PSOC4_SFLASH_V2 ? SFLASH_BASE_V2 : SFLASH_BASE_V1;
		unwounded_size = p4_info->sflash_size;
	} else {
		die_base = bank->base;
		unwounded_size = p4_info->mflash_size;
	}

	return (address - die_base) * p4_info->num_macro_unwounded / unwounded_size;
}

/** **********************************************************************************************
 * @brief This is the very first step during target detection. Function retrieves family and
 * revision IDs from the ROM table and populates the following fields in the p4_info structure:
 * p4_info->family_id = family_id;
 * p4_info->revision_id = revision_id;
 * p4_info->family = family;
 *
 * @param target current target
 * @param p4_info pointer, will be populated
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_read_family(struct target *target, struct psoc4_info *p4_info)
{
	int retval;

	uint32_t pid_buf[4];
	retval = target_read_memory(target, PSOC4_ROMTABLE_PID0, 4, 4, (uint8_t *)pid_buf);
	if (retval != ERROR_OK)
		return retval;

	uint8_t pid[4];
	for (int i = 0; i < 4; i++) {
		uint32_t tmp = target_buffer_get_u32(target, (uint8_t *)&pid_buf[i]);
		if (tmp & 0xffffff00) {
			LOG_ERROR("Unexpected data in ROMTABLE");
			return ERROR_FAIL;
		}
		pid[i] = tmp;
	}

	uint16_t family_id = pid[0] | ((pid[1] & 0xf) << 8);
	p4_info->revision_id = (pid[2] & 0xF0) | ((pid[3] & 0xF0) >> 4);
	uint32_t designer = ((pid[1] & 0xf0) >> 4) | ((pid[2] & 0xf) << 4);

	if (designer != PSOC4_ROMTABLE_DESIGNER) {
		LOG_ERROR("ROMTABLE designer is not Cypress");
		return ERROR_FAIL;
	}

	p4_info->family = NULL;
	for (struct psoc4_family *p = psoc4_families; p->id; p++) {
		if (p->id == family_id) {
			p4_info->family = p;
			break;
		}
	}

	if (!p4_info->family) {
		LOG_ERROR("PSoC4 family 0x%02X is not supported", family_id);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/** **********************************************************************************************
 * @brief based on the p4_info.controller_idx, reads one of the SPCIF.GEOMETRY registers and
 * populates the following fields in the p4_info structure:
 * p4_info->mflash_size
 * p4_info->sflash_size
 * p4_info->num_macros
 * p4_info->row_size
 * @param target current target
 * @param p4_info pointer, will be populated
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_read_geometry(struct target *target, struct psoc4_info *p4_info)
{
	const struct spcif_type *spcif = p4_info->family->spcif;
	struct armv7m_common *armv7m = target_to_armv7m(target);

	uint32_t geom = 0;
	const struct cpuss_type *cpuss = GET_CPUSS(p4_info);
	int hr = mem_ap_read_atomic_u32(armv7m->debug_ap, cpuss->geometry + (0x10000 * p4_info->flctrl_idx), &geom);
	if (hr != ERROR_OK)
		return hr;

	uint32_t val;
	val = (geom & spcif->mflash_size_mask) >> spcif->mflash_size_pos;
	p4_info->mflash_size = (val + 1) * 256;

	val = (geom & spcif->sflash_size_mask) >> spcif->sflash_size_pos;
	p4_info->sflash_size = (val + 1) * 256;

	val = (geom & spcif->macro_num_mask) >> spcif->macro_num_pos;
	p4_info->num_macro = p4_info->num_macro_unwounded = (val + 1);

	val = (geom & spcif->row_size_mask) >> spcif->rows_size_pos;
	p4_info->row_size = (val + 1) * 64;

	return ERROR_OK;
}

/** **********************************************************************************************
 * @brief translates sysreq error code to the human-readable format
 * @param error_code sysreq error code
 * @return string representing sysreq code in a human-readable format
 *************************************************************************************************/
static const char *psoc4_sysreq_errorstr(uint32_t error_code)
{
	switch (error_code) {
	case 0xF0000001:
		return "Invalid Chip Protection Mode - This API is not available in current chip protection mode";
	case 0xF0000002:
		return "Invalid NVL Address - The NVL address provided must be within the range 0x00 - 0x07";
	case 0xF0000003:
		return "Invalid Page Latch Address/Size. Address is out of bounds or size is too large/unsupported";
	case 0xF0000004:
		return "Invalid Address - The row id or byte address provided is outside of the available memory";
	case 0xF0000005:
		return "Row Protected - The row id provided is a protected row";
	case 0xF000000A:
		return "Checksum Zero Failed - The calculated checksum was not zero";
	case 0xF000000B:
		return "Invalid Opcode - The opcode is not a valid API opcode";
	case 0xF000000C:
		return "Key Opcode Mismatch - The opcode provided does not match key1 and key2";
	case 0xF000000D:
		return "Macro Protected - The macro id provided is protected";
	case 0xF000000E:
		return "Invalid start address - The start address is greater than the end address provided";
	case 0xF000000F:
		return "No NVL Used - This is the return code for any NVL specific commands called for parts without NVLs";
	case 0xF0000010:
		return "No Sector Erase - The Erase Sector operation is not supported";
	case 0xF0000011:
		return "API Not Instantiated - The SRAM API for the opcode is not instantiated";
	case 0xF0000012:
		return "Invalid Flash Clock - set IMO to 48MHz before Write/Erase operations";
	case 0xF0000013:
		return "Invalid Macro ID - the macro provided is outside of the available macros";
	case 0xF0000014:
		return "API not Available in DEAD Mode";
	case 0xF0000015:
		return "An SRSS Register is Lock Protected";
	default:
		LOG_ERROR("Unknown sysreq error 0x%" PRIx32, error_code);
		return "Unknown sysreq error";
	}
}

/** **********************************************************************************************
 * @brief translates chip protection value to the human-readable format
 * @param protection device protection state
 * @return string representing chip protection in a human-readable format
 *************************************************************************************************/
static const char *psoc4_protection_str(uint8_t protection)
{
	static const char *const protection_str[5] = {
		"VIRGIN", "OPEN", "PROTECTED", "UNKNOWN", "KILL",
	};

	if (protection > 4)
		protection = 3; /* UNKNOWN */

	return protection_str[protection];
}

/** ***********************************************************************************************
 * @brief invokes given SROM API passing additional parameters either via SYSARG register or
 * via in RAM depending on the number of parameters. If ram_params is not NULL all parameters,
 * including mandatory cmd_param, will be passed via RAM.
 * @param target current target
 * @param p4_info pointer to structure with target-specific information
 * @param cmd SROM API request code
 * @param cmd_param mandatory SROM API parameter
 * @param ram_params optional pointer to additional parameters
 * @param ram_params_size size of additional parameters, in bytes
 * @param sysarg_out SROM API return code
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_sysreq(struct target *target, struct psoc4_info *p4_info, uint8_t cmd, uint16_t cmd_param,
						uint8_t *ram_params, uint32_t ram_params_size, uint32_t *sysarg_out, bool quiet)
{
	int hr;
	if (ram_params) {
		uint32_t ram_param;
		buf_set_buf(ram_params, 0, &ram_param, 0, 32);
		LOG_DEBUG("sysreq cmd: 0x%02X, param: 0x%04X, ram_param: 0x%08X", cmd, cmd_param, ram_param);
	} else {
		LOG_DEBUG("sysreq cmd: 0x%02X, param: 0x%04X, ram_param: none", cmd, cmd_param);
	}

	const struct cpuss_type *cpuss = GET_CPUSS(p4_info);

	const uint32_t param = PSOC4_SYSREQ_KEY1 | ((PSOC4_SYSREQ_KEY2 + cmd) << 8) | (cmd_param << 16);
	struct working_area *ram_param_area = NULL;

	if (ram_params) {
		hr = target_alloc_working_area(target, ram_params_size + 4, &ram_param_area);
		if (hr != ERROR_OK)
			return hr;

		hr = target_write_u32(target, ram_param_area->address, param);
		if (hr != ERROR_OK)
			goto cleanup_ram_wa;

		hr = target_write_buffer(target, ram_param_area->address + 4, ram_params_size, ram_params);
		if (hr != ERROR_OK)
			goto cleanup_ram_wa;

		hr = target_write_u32(target, cpuss->sysarg, ram_param_area->address);
		if (hr != ERROR_OK)
			goto cleanup_ram_wa;
	} else {
		hr = target_write_u32(target, cpuss->sysarg, param);
		if (hr != ERROR_OK)
			return hr;
	}

	hr = target_write_u32(target, cpuss->sysreq, PSOC4_SYSREQ_BIT | PSOC4_SYSREQ_HMASTER_BIT | cmd);
	if (hr != ERROR_OK)
		goto cleanup_ram_wa;

	uint32_t val;
	int64_t t0 = timeval_ms();
	do {
		int64_t elapsed_ms = timeval_ms() - t0;
		if (elapsed_ms > PSOC4_IPC_TIMEOUT_MS)
			break;

		hr = target_read_u32(target, cpuss->sysreq, &val);
		if (hr != ERROR_OK)
			goto cleanup_ram_wa;

	} while (val & (PSOC4_SYSREQ_BIT | PSOC4_SYSREQ_PRIVILEGED_BIT));

	hr = target_read_u32(target, cpuss->sysarg, &val);
	if (hr != ERROR_OK)
		goto cleanup_ram_wa;

	if ((val & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
		if (!quiet)
			LOG_ERROR("%s", psoc4_sysreq_errorstr(val));

		hr = ERROR_FAIL;
	}

	if (sysarg_out)
		*sysarg_out = val;

cleanup_ram_wa:
	if (ram_param_area)
		target_free_working_area(target, ram_param_area);

	return hr;
}

/** ***********************************************************************************************
 * @brief starts pseudo flash algorithm and leaves it running. Function allocates working area for
 * algorithm code and CPU stack, adjusts stack pointer, uploads and starts the algorithm.
 * Algorithm (a basic infinite loop) runs asynchronously while driver performs Flash operations.
 * @param target current target
 * @param p4_info pointer to structure with target-specific information
 *************************************************************************************************/
static int psoc4_sysreq_prepare(struct target *target, struct psoc4_info *p4_info)
{
	if (target->coreid == 0xFF) { /* Chip is protected */
		if (!(p4_info->family->flags & PSOC4_48MHZ_NRQ))
			return psoc4_sysreq(target, p4_info, PSOC4_CMD_SET_IMO48, 0, NULL, 0, NULL, false);

		return ERROR_OK;
	}

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Initialize Vector Table Offset register (in case FW modified it) */
	int hr = target_write_u32(target, NVIC_VTOR, 0x00000000);
	if (hr != ERROR_OK)
		return hr;

	/* Allocate Working Area for Stack and Flash algorithm */
	hr = target_alloc_working_area(target, PSOC4_STACK_WA_SIZE, &g_stack_area);
	if (hr != ERROR_OK)
		return hr;

	g_armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	g_armv7m_algo.core_mode = ARM_MODE_THREAD;

	struct reg_param reg_params;
	init_reg_param(&reg_params, "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params.value, 0, 32, g_stack_area->address + g_stack_area->size);

	/* Write 'cpsie	i' + basic infinite loop algorithm to target RAM */
	hr = target_write_u32(target, g_stack_area->address, 0xE7FEB662);
	if (hr != ERROR_OK)
		goto destroy_rp_free_wa;

	hr = target_start_algorithm(target, 0, NULL, 1, &reg_params, g_stack_area->address, 0, &g_armv7m_algo);
	if (hr != ERROR_OK)
		goto destroy_rp_free_wa;

	if (!(p4_info->family->flags & PSOC4_48MHZ_NRQ)) {
		hr = psoc4_sysreq(target, p4_info, PSOC4_CMD_SET_IMO48, 0, NULL, 0, NULL, false);
		if (hr != ERROR_OK)
			goto destroy_rp_free_wa;
	}

	destroy_reg_param(&reg_params);
	return hr;

destroy_rp_free_wa:
	/* Something went wrong, do some cleanup */
	destroy_reg_param(&reg_params);

	if (g_stack_area) {
		target_free_working_area(target, g_stack_area);
		g_stack_area = NULL;
	}

	return hr;
}

/** ***********************************************************************************************
 * @brief stops running flash algorithm and releases associated resources.
 * This function is also used for cleanup in case of errors so g_stack_area may be NULL.
 * These cases have to be handled gracefully.
 * @param target current target
 * @param p4_info pointer to structure with target-specific information
 *************************************************************************************************/
static void psoc4_sysreq_release(struct target *target, struct psoc4_info *p4_info)
{
	if (g_stack_area) {
		/* Special core_id for fake SysAP */
		if (target->coreid == 0xFF)
			return;

		/* Stop flash algorithm if it is running */
		if (target->running_alg) {
			int hr = target_halt(target);
			if (hr != ERROR_OK)
				goto exit_free_wa;

			hr = target_wait_algorithm(target, 0, NULL, 0, NULL, 0, PSOC4_IPC_TIMEOUT_MS, &g_armv7m_algo);
			if (hr != ERROR_OK)
				goto exit_free_wa;
		}

	exit_free_wa:
		/* Free Stack/Flash algorithm working area */
		target_free_working_area(target, g_stack_area);
		g_stack_area = NULL;
	}

	/* Possible error is not critical here, not required and not possible via mem_ap */
	if (target->coreid != 255)
		psoc4_disable_remap(target);
}

/** ***********************************************************************************************
 * @brief configures SFlash layout for PSoC4 parts
 * @param bank SFlash bank to configure
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_decode_sflash(struct flash_bank *bank)
{
	struct psoc4_info *p4_info = bank->driver_priv;
	const uint32_t sflash_base = p4_info->family->flags & PSOC4_SFLASH_V2 ? SFLASH_BASE_V2 : SFLASH_BASE_V1;

	for (size_t i = 0; i < p4_info->num_macro; i++)
		p4_info->sflash.row_prot_addr[i] = sflash_base + i * p4_info->row_size * 8;

	switch (p4_info->row_size) {
	case 128:
		p4_info->sflash.chip_prot_offs = 0x7C;
		p4_info->sflash.siid_offs = 0x0144;
		break;
	case 256:
		p4_info->sflash.chip_prot_offs = 0x0FC;
		p4_info->sflash.siid_offs = 0x244;
		break;
	default:
		LOG_ERROR("SFlash probe: unsupported row size - %d bytes", p4_info->row_size);
		return ERROR_FAIL;
	}

	if (is_sflash_bank(bank)) {
		bank->base = sflash_base + p4_info->row_size * 4;
		bank->size = 4 * p4_info->row_size;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief configures SFlash layout for PSoC4-HV parts
 * @param bank SFlash bank to configure
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4hv_decode_sflash(struct flash_bank *bank)
{
	struct psoc4_info *info = bank->driver_priv;

	uint32_t sflash_area;
	const uint32_t user_area_tbl = SFLASH_BASE_V2 + 0x40;
	int hr = target_read_u32(bank->target, user_area_tbl + 4 * info->sflash.area_idx, &sflash_area);
	if (hr != ERROR_OK)
		return hr;

	info->sflash.chip_prot_offs = 0x7C;
	info->sflash.siid_offs = 0x00;
	for (size_t i = 0; i < info->num_macro; i++) {
		const uint32_t prot_tab = SFLASH_BASE_V2 + 0x20;
		uint16_t prot_area;
		hr = target_read_u16(bank->target, prot_tab + 2 * i, &prot_area);
		if (hr != ERROR_OK)
			return hr;
		if (prot_area != 0)
			info->sflash.row_prot_addr[i] = 0x0FFF0000 | prot_area;
	}

	if (is_sflash_bank(bank)) {
		bank->base = 0x0FFF0000 | (sflash_area & 0x0000FFFF);
		bank->size = sflash_area >> 16;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Executes system ROM API to read silicon/family ID and protection from the device
 * @param target current target
 * @param p4_info pointer to structure with target-specific information
 * @param silicon_id pointer to variable, will be populated with silicon ID
 * @param family_id pointer to variable, will be populated with family ID
 * @param protection pointer to variable, will be populated with protection state
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_sysreq_silicon_id(struct target *target, struct psoc4_info *p4_info, uint32_t *silicon_id,
								   uint16_t *family_id, uint8_t *protection)
{
	uint32_t part0, part1;

	int hr = psoc4_sysreq_prepare(target, p4_info);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc4_sysreq(target, p4_info, PSOC4_CMD_GET_SILICON_ID, 0, NULL, 0, &part0, false);
	if (hr != ERROR_OK)
		goto release;

	const struct cpuss_type *cpuss = GET_CPUSS(p4_info);
	hr = target_read_u32(target, cpuss->sysreq, &part1);
	if (hr != ERROR_OK)
		goto release;

	/* build ID as Cypress sw does:
	 * bit 31..16 silicon ID
	 * bit 15..8  revision ID (so far 0x11 for all devices)
	 * bit 7..0   family ID (lowest 8 bits) */
	if (silicon_id)
		*silicon_id = ((part0 & 0x0000ffff) << 16) | ((part0 & 0x00ff0000) >> 8) | (part1 & 0x000000ff);

	if (family_id)
		*family_id = part1 & 0x0fff;

	if (protection)
		*protection = (part1 >> 12) & 0x0f;

release:
	psoc4_sysreq_release(target, p4_info);
	return hr;
}

/** ***********************************************************************************************
 * Rounds the 32-bit integer up to the next highest power of two, e.g.
 * 1000 -> 1024, 1024 -> 1024, 2040 -> 2048 etc.
 * Borrowed from the 'Bit Twiddling Hacks', see
 * https://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
 *************************************************************************************************/
static inline uint32_t next_pow2(uint32_t val)
{
	assert(val != 0);

	val--;
	val |= (val >> 1);
	val |= (val >> 2);
	val |= (val >> 4);
	val |= (val >> 8);
	val |= (val >> 16);

	return val + 1;
}

/** ***********************************************************************************************
 * @brief get_wounded_size
 * @param bank
 * @param max_size
 * @return
 *************************************************************************************************/
static void psoc4_check_wounding(struct flash_bank *bank)
{
	const struct armv7m_common *cm = target_to_armv7m(bank->target);
	struct psoc4_info *p4_info = bank->driver_priv;
	bool check = false;

	/* Set initial bank size to full size, no wounding */
	bank->size = p4_info->mflash_size;


	for (size_t macro = 0; macro < p4_info->num_macro; macro++) {
		if (p4_info->sflash.row_prot_addr[macro] == 0) {
			check = true;
		}
	}

	uint32_t dummy;
	int hr = mem_ap_read_atomic_u32(cm->debug_ap, bank->base + p4_info->mflash_size - 4u, &dummy);
	if ((hr == ERROR_OK) && !check)
		return;

	/* Round initial size up to the nearest power of two */
	uint32_t size = p4_info->mflash_size;

	/* Round initial num_macro up to the nearest even number */
	uint32_t num_macro = (p4_info->num_macro + 1) & ~1u;

	do {
		size >>= 1;
		num_macro >>= 1;
		hr = mem_ap_read_atomic_u32(cm->debug_ap, bank->base + size - 4u, &dummy);
	} while (size && hr != ERROR_OK);

	/* There is at least one macro */
	if (num_macro == 0)
		num_macro = 1;

	LOG_DEBUG("%s: wounding detected (%d -> %d KiB, %d -> %d macro)", bank->name, p4_info->mflash_size / 1024u,
			  size / 1024u, p4_info->num_macro, num_macro);

	bank->size = size;
	p4_info->num_macro = num_macro;
}

/** ***********************************************************************************************
 * @brief Probes the device and populates related data structures with target flash geometry
 * data. This is done by reading ROM Table and (one of the) SPCIF.GEOMETRY register(s) describing
 * available amount of memory.
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_probe(struct flash_bank *bank)
{
	struct psoc4_info *p4_info = bank->driver_priv;

	int hr = psoc4_read_family(bank->target, bank->driver_priv);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc4_read_geometry(bank->target, bank->driver_priv);
	if (hr != ERROR_OK)
		return hr;

	hr = is_psoc4hv(bank) ? psoc4hv_decode_sflash(bank) : psoc4_decode_sflash(bank);
	if (hr != ERROR_OK)
		return hr;

	if (is_flashp_bank(bank)) {
		struct flash_bank *main_bank;
		hr = get_flash_bank_by_addr(bank->target, 0, true, &main_bank);
		if (hr != ERROR_OK)
			return hr;

		bank->size = main_bank->size / p4_info->row_size / 8;
		free(bank->sectors);
		bank->num_sectors = 1;
		bank->sectors = alloc_block_array(0, bank->size, 1);
	} else if (is_chipp_bank(bank)) {
		bank->size = 1;
		free(bank->sectors);
		bank->num_sectors = 1;
		bank->sectors = alloc_block_array(0, 1, 1);
	} else {
		if (is_mflash_bank(bank) || is_wflash_bank(bank))
			psoc4_check_wounding(bank);

		free(bank->sectors);
		bank->num_sectors = bank->size / p4_info->row_size;
		bank->sectors = alloc_block_array(0, p4_info->row_size, bank->size / p4_info->row_size);
		bank->write_start_alignment = p4_info->row_size;
		bank->write_end_alignment = p4_info->row_size;
	}

	if (bank->sectors == NULL)
		return ERROR_FAIL;

	p4_info->probed = true;

	LOG_DEBUG("Flash bank '%s': %d bytes (%d rows, %d bytes/row, %d macro)", bank->name, bank->size,
			  bank->size / p4_info->row_size, p4_info->row_size, p4_info->num_macro);

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Probes given flash bank only if it hasn't been probed yet
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_auto_probe(struct flash_bank *bank)
{
	struct psoc4_info *p4_info = bank->driver_priv;
	if (p4_info->probed)
		return ERROR_OK;
	return psoc4_probe(bank);
}

/** ***********************************************************************************************
 * @brief reads whole row protection array and stores it in given buffer
 * @param target current target
 * @param prot_array pointer to buffer to write data to
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_read_row_protection(struct target *target, uint8_t **prot_array)
{
	*prot_array = NULL;

	struct flash_bank *bank;
	int hr = get_flash_bank_by_addr(target, 0, true, &bank);
	if (hr != ERROR_OK)
		return hr;

	struct psoc4_info *p4_info = bank->driver_priv;

	size_t num_prot_bytes = bank->size / p4_info->row_size / 8;
	size_t bytes_per_macro = num_prot_bytes / p4_info->num_macro;
	uint32_t *prot_table = p4_info->sflash.row_prot_addr;

	*prot_array = malloc(num_prot_bytes);
	if (*prot_array == NULL)
		return ERROR_FAIL;

	uint8_t *ptr = *prot_array;
	for (size_t macro = 0; macro < p4_info->num_macro; macro++) {
		hr = target_read_buffer(target, prot_table[macro], bytes_per_macro, ptr);
		LOG_DEBUG("Reading prot block @0x%08" PRIx32 ", %zu bytes", prot_table[macro], bytes_per_macro);
		if (hr != ERROR_OK) {
			free(*prot_array);
			*prot_array = NULL;
			return hr;
		}
		ptr += bytes_per_macro;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief programs whole flash protection array stored in a given buffer
 * @param target current target
 * @param prot_array pointer to buffer to read data from
 * @param chip_protect - PSOC4_CHIP_PROT_OPEN, PSOC4_CHIP_PROT_PROTECTED or PSOC4_CHIP_PROT_KILL
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_write_row_protection(struct target *target, uint8_t *prot_array, uint8_t chip_protect)
{
	struct flash_bank *bank;
	int hr = get_flash_bank_by_addr(target, 0, true, &bank);
	if (hr != ERROR_OK)
		return hr;

	struct psoc4_info *p4_minfo = bank->driver_priv;
	size_t bytes_per_macro = bank->size / p4_minfo->row_size / p4_minfo->num_macro / 8;
	hr = psoc4_sysreq_prepare(target, p4_minfo);
	if (hr != ERROR_OK)
		return hr;

	uint8_t req_buffer[bytes_per_macro + 4];
	for (size_t macro = 0; macro < p4_minfo->num_macro; macro++) {
		uint16_t load_macro = psoc4_addr_to_macro(bank, p4_minfo->sflash.row_prot_addr[macro]);
		memcpy(&req_buffer[4], prot_array, bytes_per_macro);
		buf_set_u32(req_buffer, 0, 32, bytes_per_macro - 1);
		hr = psoc4_sysreq(target, p4_minfo, PSOC4_CMD_LOAD_LATCH, ((load_macro & 0x7F) << 8), req_buffer,
						  sizeof(req_buffer), NULL, false);
		if (hr != ERROR_OK)
			goto release;

		uint16_t protect_macro = (macro & 0x7F) << 8;
		protect_macro |= chip_protect;
		psoc4_sysreq(target, p4_minfo, PSOC4_CMD_WRITE_PROTECTION, protect_macro, NULL, 0, NULL, false);
		if (hr != ERROR_OK)
			goto release;

		prot_array += bytes_per_macro;
	}

release:
	psoc4_sysreq_release(target, p4_minfo);
	return hr;
}

/** ***********************************************************************************************
 * @brief sector protection routine
 * @param bank the bank to protect or unprotect
 * @param set if non-zero, enable protection; if 0, disable it
 * @param first the first sector to (un)protect, typically 0
 * @param last the last sector to (un)project, typically N-1
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	if (!is_mflash_bank(bank)) {
		LOG_ERROR("Row Protection only supported for Main Flash");
		return ERROR_FAIL;
	}

	struct target *target = bank->target;

	uint8_t *prot_array;
	int hr = psoc4_read_row_protection(target, &prot_array);
	if (hr != ERROR_OK) {
		free(prot_array);
		return hr;
	}

	for (size_t bit = first; bit <= last; bit++) {
		size_t byte = bit / 8;
		size_t shift = bit % 8;
		if (set) {
			prot_array[byte] |= (0x01u << shift);
		} else {
			prot_array[byte] &= ~(0x01u << shift);
		}
	}

	hr = psoc4_write_row_protection(target, prot_array, PSOC4_CHIP_PROT_OPEN);
	free(prot_array);

	return hr;
}

/** ***********************************************************************************************
 * @brief determine if the specific bank is blank
 * field for each of the flash sectors
 * @param bank - the bank to check
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_blank_check(struct flash_bank *bank) {
	if (is_chipp_bank(bank)) {
		LOG_ERROR("Chip Protection bank does not support erase_check operation");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (is_flashp_bank(bank)) {
		assert(bank->num_sectors == 1);

		uint8_t *flash_prot;
		int hr = psoc4_read_row_protection(bank->target, &flash_prot);
		if (hr != ERROR_OK)
			return hr;
		bank->sectors[0].is_erased = 1;
		for (size_t i = 0; i < bank->size; i++) {
			if (flash_prot[i]) {
				bank->sectors[0].is_erased = 0;
				break;
			}
		}
		free(flash_prot);
		return ERROR_OK;
	}

	return default_flash_blank_check(bank);
}

/** ***********************************************************************************************
 * @brief determine if the specific bank is "protected" or not, populates bank.sectors.is_protected
 * field for each of the flash sectors
 * @param bank - the bank to check
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_protect_check(struct flash_bank *bank)
{
	if (!is_mflash_bank(bank)) {
		for (size_t i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_protected = false;
		return ERROR_OK;
	}

	uint8_t *prot_data;
	int hr = psoc4_read_row_protection(bank->target, &prot_data);
	if (hr != ERROR_OK)
		goto exit;

	for (uint32_t i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = prot_data[i / 8] & (1 << (i % 8)) ? 1 : 0;

exit:
	free(prot_data);
	return hr;
}

/** ***********************************************************************************************
 * @brief performs program operation, see psoc4hv_write.c for details
 * @param bank current flash bank
 * @param buffer pointer to the buffer with data
 * @param offset starting offset in flash bank
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
#define SFLASH_PROGRAMMING_MASK 0x00000100
static int psoc4_write_inner(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset,
							 uint32_t count, int operation)
{
	static const uint8_t write_algo_p4hv[] = {
#include "../../../contrib/loaders/flash/psoc4hv/psoc4hv_write.inc"
	};

	static const uint8_t write_algo_p4[] = {
#include "../../../contrib/loaders/flash/psoc4hv/psoc4_write.inc"
	};

	const uint8_t *write_algo_p;
	size_t write_algo_size;
	if (is_psoc4hv(bank)) {
		write_algo_p = write_algo_p4hv;
		write_algo_size = sizeof(write_algo_p4hv);
	} else {
		write_algo_p = write_algo_p4;
		write_algo_size = sizeof(write_algo_p4);
	}

	struct target *target = bank->target;
	struct psoc4_info *p4_info = bank->driver_priv;
	const struct cpuss_type *cpuss = GET_CPUSS(p4_info);

	struct working_area *wa_algo_stack;
	int hr = target_alloc_working_area(target, write_algo_size + PSOC4_STACK_WA_SIZE, &wa_algo_stack);
	if (hr != ERROR_OK)
		return hr;

	size_t wa_avail = target_get_working_area_avail(target);
	/* Header - uint32_t[4], chunk - uint32_t[2] + data */
	wa_avail -= (wa_avail - 16) % (8 + p4_info->row_size);
	const size_t max_num_chunks = wa_avail / (8 + p4_info->row_size);
	uint8_t chunks[wa_avail];

	if (max_num_chunks == 0) {
		LOG_ERROR("No large enough working area available");
		hr = ERROR_FAIL;
		goto exit0;
	}

	struct working_area *wa_buffers;
	hr = target_alloc_working_area(target, wa_avail, &wa_buffers);
	if (hr != ERROR_OK)
		goto exit0;

	hr = target_write_buffer(target, wa_algo_stack->address, write_algo_size, write_algo_p);
	if (hr != ERROR_OK)
		goto exit1;

	uint32_t header[4] = {
		cpuss->sysreq,
		cpuss->sysarg,
		p4_info->row_size,
		0,
	};

	hr = target_write_buffer(target, wa_buffers->address, sizeof(header), (uint8_t *)header);
	if (hr != ERROR_OK)
		goto exit1;

	/* Pass all parameters to the entry point (see psoc4_write.c) */
	struct reg_param reg_params[3];
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, wa_buffers->address);

	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, bank->base + offset);

	init_reg_param(&reg_params[2], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, wa_algo_stack->address + wa_algo_stack->size);

	/* Start the algorithm in the background */
	struct armv7m_algorithm armv7m_algo;
	armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_algo.core_mode = ARM_MODE_THREAD;
	hr = target_start_algorithm(target, 0, NULL, 3, reg_params, wa_algo_stack->address, 0, &armv7m_algo);
	if (hr != ERROR_OK)
		goto exit2;

	if (!(p4_info->family->flags & PSOC4_48MHZ_NRQ)) {
		hr = psoc4_sysreq(target, p4_info, PSOC4_CMD_SET_IMO48, 0, NULL, 0, NULL, false);
		if (hr != ERROR_OK)
			goto exit3;
	}

	size_t total_chunks = count / p4_info->row_size;
	size_t chunk_idx = 0;
	progress_init(total_chunks, operation);

	while (total_chunks) {
		uint32_t row_id = offset / p4_info->row_size;

		const uint32_t macro_id = psoc4_addr_to_macro(bank, bank->base + offset);
		uint32_t param0 = (p4_info->flctrl_idx << 31) | (macro_id & 0x7F) << 24 | (p4_info->row_size - 1);
		uint32_t param1 = (p4_info->flctrl_idx << 31) | (row_id & 0x7FFF) << 16 | (row_id & 0xFF);
		if (is_sflash_bank(bank))
			param1 |= SFLASH_PROGRAMMING_MASK;

		buf_set_u32(&chunks[chunk_idx * (8 + p4_info->row_size) + 0], 0, 32, param0);
		buf_set_u32(&chunks[chunk_idx * (8 + p4_info->row_size) + 4], 0, 32, param1);
		memcpy(&chunks[chunk_idx * (8 + p4_info->row_size) + 8], buffer, p4_info->row_size);

		chunk_idx++;
		total_chunks--;
		buffer += p4_info->row_size;
		offset += p4_info->row_size;
		if (chunk_idx == max_num_chunks || total_chunks == 0) {
			hr = target_write_buffer(target, wa_buffers->address + 16, sizeof(chunks), chunks);
			if (hr != ERROR_OK)
				goto exit4;

			hr = target_write_u32(target, wa_buffers->address + 12, chunk_idx);
			if (hr != ERROR_OK)
				goto exit4;

			uint32_t status = UINT32_MAX;
			int64_t t0 = timeval_ms();
			while (status) {
				if (timeval_ms() - t0 > PSOC4_IPC_TIMEOUT_MS) {
					LOG_ERROR("Timeout waiting for flash write algorithm");
					hr = ERROR_FLASH_OPERATION_FAILED;
					goto exit4;
				}

				hr = target_poll(target);
				if (hr != ERROR_OK)
					goto exit4;

				hr = target_read_u32(target, wa_buffers->address + 12, &status);
				if (hr != ERROR_OK)
					goto exit4;

				if (status & PSOC4_SROM_STATUS_MASK || target->state != TARGET_DEBUG_RUNNING) {
					LOG_ERROR("Flash algorithm reported failure - \"%s\"", psoc4_sysreq_errorstr(status));
					hr = ERROR_FLASH_OPERATION_FAILED;
					goto exit4;
				}
			}

			chunk_idx = 0;
		}

		progress_left(total_chunks);
	}

exit4:
	progress_done(hr);
exit3:
	target_halt(target);
	target_wait_algorithm(target, 0, NULL, 0, NULL, 0, PSOC4_IPC_TIMEOUT_MS, &armv7m_algo);
exit2:
	for (size_t i = 0; i < sizeof(reg_params) / sizeof(reg_params[0]); i++)
		destroy_reg_param(&reg_params[i]);
exit1:
	target_free_working_area(target, wa_buffers);
exit0:
	target_free_working_area(target, wa_algo_stack);
	psoc4_disable_remap(target);
	return hr;
}

extern struct command_context *global_cmd_ctx;

/** ***********************************************************************************************
 * @brief Prevents unintentional activation of KILL mode. This function checks if the user has set
 * ALLOW_KILL_MODE variable to 'YES'. If not, it will print a warning message
 * and return an error code.
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int prevent_unintentional_kill_mode(void)
{
	Jim_Interp *interp = global_cmd_ctx->interp;
	Jim_Obj *objPtr = Jim_GetGlobalVariableStr(interp, "ALLOW_KILL_MODE", 0);
	/* Need to make sure that user understands the consequences of KILL mode */
	if (objPtr != NULL) {
		const char *allow_kill_mode_str;
		int len;

		allow_kill_mode_str = Jim_GetString(objPtr, &len);
		if (len == 3 && strcmp("YES", allow_kill_mode_str) == 0) {
			LOG_INFO("**KILL mode is being activated, any debug access will be prohibited.");
		} else {
			LOG_ERROR("KILL mode is a special mode created to prohibit debug\r\n"
					"access via debug pins. If activated - there is NO way to reverse\r\n"
					"changes. Use this mode ONLY if you have finalized application without\r\n"
					"need to ever debug it.");
			LOG_INFO("KILL mode permission absent. To grant permission, please "
					"set ALLOW_KILL_MODE variable to 'YES'.");
			return ERROR_FAIL;
		}
	} else {
		LOG_ERROR("Tcl variable ALLOW_KILL_MODE does not exist, "
				"transition to KILL mode is impossible");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int psoc4_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	if (is_flashp_bank(bank)) {
		uint8_t *flash_prot;
		int hr = psoc4_read_row_protection(bank->target, &flash_prot);
		if (hr != ERROR_OK)
			return hr;

		memcpy(flash_prot + offset, buffer, count);

		hr = psoc4_write_row_protection(bank->target, flash_prot, PSOC4_CHIP_PROT_OPEN);
		free(flash_prot);
		return hr;
	}

	if (is_chipp_bank(bank)) {
		uint8_t current_prot = 0;
		int hr = psoc4_sysreq_silicon_id(bank->target, bank->driver_priv, NULL, NULL, &current_prot);
		if(hr != ERROR_OK)
			return hr;

		if(current_prot == *buffer)
			return ERROR_OK;

		if(current_prot == PSOC4_CHIP_PROT_PROTECTED && *buffer == PSOC4_CHIP_PROT_OPEN) {
			LOG_ERROR("Protection 'OPEN' can not be applied while chip is in 'PROTECTED' state, "
						"use 'psoc4 mass_erase 0' to unlock the chip");
			return ERROR_FAIL;
		} else if(current_prot == PSOC4_CHIP_PROT_OPEN && *buffer == PSOC4_CHIP_PROT_PROTECTED) {
			uint8_t *row_protection;
			hr = psoc4_read_row_protection(bank->target, &row_protection);
			if (hr != ERROR_OK)
				return hr;

			hr = psoc4_write_row_protection(bank->target, row_protection, PSOC4_CHIP_PROT_PROTECTED);
			if (hr == ERROR_OK)
				LOG_INFO("Chip protection will take effect after Reset");

			free(row_protection);
			return hr;
		} else if(current_prot == PSOC4_CHIP_PROT_OPEN && *buffer == PSOC4_CHIP_PROT_KILL) {
			uint8_t *row_protection;

			LOG_WARNING("Detected KILL mode activated in the image.");
			hr = prevent_unintentional_kill_mode();
			if (hr != ERROR_OK)
				return hr;

			hr = psoc4_read_row_protection(bank->target, &row_protection);
			if (hr != ERROR_OK)
				return hr;

			hr = psoc4_write_row_protection(bank->target, row_protection, PSOC4_CHIP_PROT_KILL);
			if (hr == ERROR_OK)
				LOG_INFO("KILL mode will take effect after Reset");

			free(row_protection);
			return hr;
		} else {
			LOG_ERROR("Transition '%s' -> '%s' is not supported", psoc4_protection_str(current_prot),
					  psoc4_protection_str(*buffer));
			return ERROR_FAIL;
		}
	}

	return psoc4_write_inner(bank, buffer, offset, count, PROGRAMMING);
}

/** ***********************************************************************************************
 * @brief Performs MassErase operation. This will erase ALL flash banks on PSoC4-HV!
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_mass_erase(struct target *target) {
	struct psoc4_info psoc4_info;
	memset(&psoc4_info, 0, sizeof(psoc4_info));

	int hr = psoc4_read_family(target, &psoc4_info);
	if (hr != ERROR_OK)
		return hr;

	uint8_t protection = 0;
	hr = psoc4_sysreq_silicon_id(target, &psoc4_info, NULL, NULL, &protection);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc4_sysreq_prepare(target, &psoc4_info);
	if (hr != ERROR_OK)
		return hr;

	if (protection == PSOC4_CHIP_PROT_PROTECTED) {
		LOG_INFO("Entering OPEN protection mode, this will erase internal Flash...");
		hr = psoc4_sysreq(target, &psoc4_info, PSOC4_CMD_WRITE_PROTECTION, PSOC4_CHIP_PROT_OPEN, NULL, 0, NULL, false);
		if (hr == ERROR_OK)
			LOG_INFO("Chip protection will take effect after Reset");
	} else {
		LOG_INFO("Performing Mass Erase...");
		uint8_t ram_param[4];
		buf_set_u32(ram_param, 0, 32, 0);
		hr = psoc4_sysreq(target, &psoc4_info, PSOC4_CMD_ERASE_ALL, 0, ram_param, sizeof(ram_param), NULL, false);
	}

	psoc4_sysreq_release(target, &psoc4_info);
	return hr;
}

/** ***********************************************************************************************
 * @brief Intended to perform sector erase which is not supported in PSoC4
 * @param bank current flash bank
 * @param first The first sector to erase
 * @param last The last sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	/* SFlash can not be erased so simply fill it with zeros */
	if (is_sflash_bank(bank)) {
		struct psoc4_info *info = bank->driver_priv;
		uint8_t empty[bank->size];
		memset(empty, 0, sizeof(empty));
		return psoc4_write_inner(bank, empty, first * info->row_size,
								 (last - first + 1) * info->row_size, ERASING);
	}

	if(is_flashp_bank(bank)) {
		struct flash_bank *main_bank;
		int hr = get_flash_bank_by_addr(bank->target, 0, true, &main_bank);
		if (hr != ERROR_OK)
			return hr;
		return psoc4_protect(main_bank, false, 0, main_bank->num_sectors - 1);
	}

	if(is_chipp_bank(bank)) {
		LOG_WARNING("Erase of Chip Protection bank is not supported, "
					"use 'psoc4 mass_erase 0' to unlock the chip");
		return ERROR_OK;
	}

	if (first != 0 || last != bank->num_sectors - 1) {
		LOG_WARNING("Only mass erase available, erase skipped! (psoc4 mass_erase)");
		return ERROR_OK;
	}

	return psoc4_mass_erase(bank->target);
}

/** ************************************************************************************************
 * @brief display human-readable information about the target into the given buffer
 * @param target current target
 * @param cmd - command being handled
 * @param use_sysreq - flag to read info of locked silicon
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_silicon_info(struct target *target, struct command_invocation *cmd, bool use_sysreq)
{
	uint32_t si_siid;
	uint32_t protection;

	struct psoc4_info *p4_info = alloca(sizeof(struct psoc4_info));
	if (!p4_info)
		return ERROR_FAIL;

	if (!use_sysreq) {
		struct flash_bank *bank;
		int hr = get_flash_bank_by_addr(target, 0, true, &bank);
		if (hr != ERROR_OK)
			return hr;

		memcpy(p4_info, bank->driver_priv, sizeof(struct psoc4_info));

		const uint32_t sflash_base = p4_info->family->flags & PSOC4_SFLASH_V2 ? SFLASH_BASE_V2 : SFLASH_BASE_V1;
		hr = target_read_u32(target, sflash_base + p4_info->sflash.siid_offs, &si_siid);
		if (hr != ERROR_OK)
			return hr;

		si_siid &= 0xFFFFu;

		hr = target_read_u32(target, sflash_base + p4_info->sflash.chip_prot_offs, &protection);
		if (hr != ERROR_OK)
			return hr;

		protection = (protection >> 24) & 0x0f;
		if (protection == PSOC4_CHIP_PROT_VIRGIN)
			protection = PSOC4_CHIP_PROT_OPEN;
		else if (protection == PSOC4_CHIP_PROT_OPEN)
			protection = PSOC4_CHIP_PROT_VIRGIN;

	} else {
		int hr = psoc4_read_family(target, p4_info);
		if (hr != ERROR_OK)
			return hr;

		uint8_t prot;
		hr = psoc4_sysreq_silicon_id(target, p4_info, &si_siid, NULL, &prot);
		if (hr != ERROR_OK)
			return hr;

		si_siid >>= 16;
		protection = prot;
	}

	const char *tcl = "source [find target/cympn.cfg]; "
					  "set siid \"%04X\"; "
					  "if { [dict exists $MPN $siid ] } {"
					  "  set pn [lindex $MPN($siid) 0]; "
					  "  set main_size [lindex $MPN($siid) 2]; "
					  "  set family [lindex $MPN($siid) 4]; "
					  "  return [list $pn $main_size $family]; "
					  "}";

	char *si_mpn = NULL;
	char *si_family = NULL;
	uint32_t udd_mflash_size = 0;
	char *command = alloc_printf(tcl, si_siid);

	Jim_Interp *interp = global_cmd_ctx->interp;
	Jim_Eval(interp, command);
	Jim_Obj *list = Jim_GetResult(interp);

	if (Jim_IsList(list) && Jim_ListLength(interp, list) == 3) {
		si_mpn = strdup(Jim_GetString(Jim_ListGetIndex(interp, list, 0), NULL));
		long l;
		Jim_GetLong(interp, Jim_ListGetIndex(interp, list, 1), &l);
		udd_mflash_size = (uint32_t)(l * 1024u);
		si_family = strdup(Jim_GetString(Jim_ListGetIndex(interp, list, 2), NULL));
	}
	free(command);

	char si_rev_major = "XABCDEFGHIJKLMNOP"[p4_info->revision_id >> 4u];
	char si_rev_minor = "X0123456789abcdef"[p4_info->revision_id & 0x0F];
	const char *format = "*****************************************\n"
						 "** Silicon: 0x%04X, Family: 0x%02X, Rev.: 0x%02X (%c%c)\n"
						 "** Detected Family: %s\n"
						 "%s"
						 "%s"
						 "** Chip Protection: %s\n"
						 "*****************************************\n";

	char mpn_str[80];
	char family_str[80];
	char mflash_size_str[64];
	if (si_mpn && si_family) {
		snprintf(mpn_str, sizeof(mpn_str), "** Detected Device: %s\n", si_mpn);
		strncpy(family_str, si_family, sizeof(family_str) - 1);
		strncpy(p4_info->family->name, si_family, sizeof(p4_info->family->name) - 1);
		snprintf(mflash_size_str, sizeof(mflash_size_str), "** Detected Main Flash size, kb: %d\n",
				 udd_mflash_size / 1024u);
		free(si_mpn);
		free(si_family);
	} else {
		snprintf(mpn_str, sizeof(mpn_str), "** The connected device is not available in the device database\n");
		strncpy(family_str, psoc4_family_name(p4_info->family->id, si_siid), sizeof(family_str) - 1);
		snprintf(mflash_size_str, sizeof(mflash_size_str), "** Main Flash size will be auto-detected\n");
	}

	command_print_sameline(cmd, format, si_siid, p4_info->family->id, p4_info->revision_id, si_rev_major,
			si_rev_minor, family_str, mpn_str, mflash_size_str, psoc4_protection_str(protection));
	return ERROR_OK;
}

static int psoc4_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct target *target = bank->target;
	struct psoc4_info *p4_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		command_print_sameline(cmd,
				 "%s: target state: %s, halt the target for more information\n"
				 "%s: %d bytes (%d rows, %d bytes/row, %d macro)",
				 p4_info->family->name, target_state_name(target), bank->name, bank->size,
				 bank->size / p4_info->row_size, p4_info->row_size, p4_info->num_macro);
		return ERROR_OK;
	}

	uint32_t silicon_id;
	uint8_t protection;
	int hr = psoc4_sysreq_silicon_id(target, p4_info, &silicon_id, NULL, &protection);
	if (hr != ERROR_OK)
		return hr;

	command_print_sameline(cmd,
			 "%s: silicon id: 0x%08X, protection: %s\n"
			 "%s: %d bytes (%d rows, %d bytes/row, %d macro)",
			 p4_info->family->name, silicon_id, psoc4_protection_str(protection), bank->name, bank->size,
			 bank->size / p4_info->row_size, p4_info->row_size, p4_info->num_macro);

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Performs Flash Read operation with ECC error reporting
 * @param bank The bank to read
 * @param buffer The data bytes read
 * @param offset The offset into the chip to read
 * @param count The number of bytes to read
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc4_flash_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	if (is_flashp_bank(bank)) {
		uint8_t *flash_prot;
		int hr = psoc4_read_row_protection(bank->target, &flash_prot);
		if (hr != ERROR_OK)
			return hr;
		memcpy(buffer, flash_prot + offset, count);
		free(flash_prot);
		return ERROR_OK;
	}

	if (is_chipp_bank(bank)) {
		struct psoc4_info *info = bank->driver_priv;
		const uint32_t sflash_base = info->family->flags & PSOC4_SFLASH_V2 ? SFLASH_BASE_V2 : SFLASH_BASE_V1;

		int hr = target_read_u8(bank->target, sflash_base + info->sflash.chip_prot_offs + 3, buffer);
		if (*buffer == PSOC4_CHIP_PROT_VIRGIN)
			*buffer = PSOC4_CHIP_PROT_OPEN;
		else if (*buffer == PSOC4_CHIP_PROT_OPEN)
			*buffer = PSOC4_CHIP_PROT_VIRGIN;

		return hr;
	}

	if (!g_ecc_enabled)
		return default_flash_read(bank, buffer, offset, count);

	assert(offset % 4 == 0);
	assert(count % 4 == 0);
	int hr = target_write_u32(bank->target, FAULT_STATUS(0), 0x00);
	if (hr != ERROR_OK)
		return hr;

	uint32_t address = bank->base + offset;

	while (count) {
		const int lvl = change_debug_level(LOG_LVL_USER);
		target_read_buffer(bank->target, address, 4, buffer);
		change_debug_level(lvl);

		uint32_t status = 0;
		hr = target_read_u32(bank->target, FAULT_STATUS(0), &status);
		if (hr != ERROR_OK)
			return hr;

		if (status & FAULT_STATUS_VALID_MASK) {
			uint32_t er_addr;
			hr = target_read_u32(bank->target, FAULT_DATA0(0), &er_addr);
			if (hr != ERROR_OK)
				return hr;

			LOG_WARNING("ECC Error at address 0x%08X", er_addr);

			hr = target_write_u32(bank->target, FAULT_STATUS(0), 0x00);
			if (hr != ERROR_OK)
				return hr;
		}

		count -= 4;
		buffer += 4;
		address += 4;
	}

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(psoc4_flash_bank_command)
{
	(void)cmd;
	bank->driver_priv = calloc(1, sizeof(struct psoc4_info));
	if (!bank->driver_priv)
		return ERROR_FAIL;

	bank->default_padded_value = bank->erased_value = 0x00;

	return ERROR_OK;
}

COMMAND_HANDLER(psoc4_handle_mass_erase)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Chip is protected, transition to OPEN */
	struct target *target = get_current_target(CMD_CTX);
	if (target->coreid == 255)
		return psoc4_mass_erase(target);

	struct flash_bank *bank;
	int hr = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (hr != ERROR_OK)
		return hr;

	if (is_sflash_bank(bank) || is_flashp_bank(bank) || is_chipp_bank(bank))
		hr = psoc4_erase(bank, 0, bank->num_sectors - 1);
	else
		hr = psoc4_mass_erase(get_current_target(CMD_CTX));

	command_print(CMD, hr == ERROR_OK ? "psoc mass erase complete" : "psoc mass erase failed");

	return hr;
}

COMMAND_HANDLER(psoc4_handle_chip_protect)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	if (target->coreid == 0xFF) {
		LOG_ERROR("The 'chip_protect' command is not available via mem_ap");
		return ERROR_FAIL;
	}

	struct flash_bank *mflash_bank;
	int hr = get_flash_bank_by_addr(target, 0, true, &mflash_bank);
	if (hr != ERROR_OK)
		return hr;

	struct psoc4_info *p4_info = mflash_bank->driver_priv;
	uint8_t protection = 0;
	hr = psoc4_sysreq_silicon_id(target, p4_info, NULL, NULL, &protection);
	if (hr != ERROR_OK)
		return hr;

	if (protection != PSOC4_CHIP_PROT_OPEN) {
		LOG_ERROR("Operation not supported when chip is in %s state", psoc4_protection_str(protection));
		return ERROR_FAIL;
	}

	uint8_t *row_protection;
	hr = psoc4_read_row_protection(target, &row_protection);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc4_write_row_protection(target, row_protection, PSOC4_CHIP_PROT_PROTECTED);
	if (hr == ERROR_OK)
		LOG_INFO("Chip protection will take effect after Reset");

	free(row_protection);
	return hr;
}

COMMAND_HANDLER(psoc4_handle_kill_mode)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	if (target->coreid == 0xFF) {
		LOG_ERROR("The 'kill_mode' command is not available via mem_ap");
		return ERROR_FAIL;
	}

	struct flash_bank *mflash_bank;
	int hr = get_flash_bank_by_addr(target, 0, true, &mflash_bank);
	if (hr != ERROR_OK)
		return hr;

	hr = prevent_unintentional_kill_mode();
	if (hr != ERROR_OK)
		return hr;

	struct psoc4_info *p4_info = mflash_bank->driver_priv;
	uint8_t protection = 0;
	hr = psoc4_sysreq_silicon_id(target, p4_info, NULL, NULL, &protection);
	if (hr != ERROR_OK)
		return hr;

	if (protection != PSOC4_CHIP_PROT_OPEN) {
		LOG_ERROR("Operation not supported when chip is in %s state", psoc4_protection_str(protection));
		return ERROR_FAIL;
	}

	uint8_t *row_protection;
	hr = psoc4_read_row_protection(target, &row_protection);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc4_write_row_protection(target, row_protection, PSOC4_CHIP_PROT_KILL);
	if (hr == ERROR_OK)
		LOG_INFO("KILL mode will take effect after Reset");

	free(row_protection);
	return hr;
}

COMMAND_HANDLER(psoc4_handle_reset_halt)
{
	if (CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int hr;
	struct target *target = get_current_target(CMD_CTX);

	if (target->coreid == 0xFF) {
		LOG_ERROR("The 'reset_halt' command is not available via mem_ap");
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		hr = target_halt(target);
		if (hr != ERROR_OK)
			return hr;

		hr = target_wait_state(target, TARGET_HALTED, PSOC4_IPC_TIMEOUT_MS);
		if (hr != ERROR_OK)
			return hr;
	}

	struct flash_bank *main_flash;
	hr = get_flash_bank_by_addr(target, 0, true, &main_flash);
	if (hr != ERROR_OK)
		return hr;

	hr = psoc4_disable_remap(target);
	if (hr != ERROR_OK)
		return hr;

	uint32_t reset_addr;
	/* Read and validate Reset Vector value */
	hr = target_read_u32(target, 0x00000004, &reset_addr);
	if (hr != ERROR_OK)
		return hr;

	/* Check the reset address in the user's application (2nd word in the flash) points to the correct location.
	 * It must not be out of flash and it must not be less than the minimal size of the CM0 Vector Table.
	 * The minimum size could be as small as 0x10 if the IRQs are relocated. */
	if (reset_addr > main_flash->size || reset_addr < 0x10) {
		LOG_INFO("Entry Point address invalid (0x%08X), reset_halt skipped", reset_addr);
		return ERROR_OK;
	}

	/* Set breakpoint at User Application entry point */
	hr = breakpoint_add(target, reset_addr, 2, BKPT_HARD);
	if (hr != ERROR_OK)
		return hr;

	/* Reset the CM0 by asserting SYSRESETREQ */
	const struct armv7m_common *cm = target_to_armv7m(target);
	LOG_INFO("%s: bkpt @0x%08X, issuing SYSRESETREQ", target->cmd_name, reset_addr);
	mem_ap_write_atomic_u32(cm->debug_ap, NVIC_AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ);

	dap_invalidate_cache(cm->debug_ap->dap);
	register_cache_invalidate(target->reg_cache);

	/* Target is now in RESET */
	target->state = TARGET_RESET;

	enum log_levels lvl = change_debug_level(LOG_LVL_USER);
	int64_t t0 = timeval_ms();
	while ((timeval_ms() - t0) < PSOC4_IPC_TIMEOUT_MS) {
		dap_dp_init(cm->debug_ap->dap);
		target->examined = false;
		hr = cortex_m_examine(target);
		if (hr == ERROR_OK)
			break;
		alive_sleep(5);
	}
	change_debug_level(lvl);

	hr = target_wait_state(target, TARGET_HALTED, PSOC4_IPC_TIMEOUT_MS);

	/* Remove the break point first */
	breakpoint_remove(target, reset_addr);

	return hr;
}

COMMAND_HANDLER(psoc4_handle_silicon_info)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	bool use_sysreq = (CMD_ARGC == 1 && !strcmp(CMD_ARGV[0], "sysreq"));
	return psoc4_silicon_info(get_current_target(CMD_CTX), CMD, use_sysreq);
}

struct flash_bank *psoc4_alloc_bank(struct flash_bank *main_bank, uint32_t flctrl_idx, uint32_t base,
										const char *suffix, bool add_to_list)
{
	const char *p_dot = strrchr(main_bank->name, '.');
	if (!p_dot) {
		LOG_ERROR("Flash bank '%s' does not have a dot '.' in the name!", main_bank->name);
		return NULL;
	}

	char *bank_name = calloc(p_dot - main_bank->name + strlen(suffix) + 2, 1);
	if (!bank_name)
		return NULL;

	memcpy(bank_name, main_bank->name, p_dot - main_bank->name + 1);
	strcat(bank_name, suffix);

	if (get_flash_bank_by_name_noprobe(bank_name)) {
		free(bank_name);
		return NULL;
	}

	struct flash_bank *new_bank = calloc(1, sizeof(struct flash_bank));
	if (!new_bank) {
		free(bank_name);
		return NULL;
	}

	struct psoc4_info *info = calloc(1, sizeof(struct psoc4_info));
	if (!info) {
		free(new_bank);
		free(bank_name);
		return NULL;
	}

	info->flctrl_idx = flctrl_idx;
	new_bank->driver_priv = info;
	new_bank->driver = main_bank->driver;
	new_bank->target = main_bank->target;
	new_bank->name = bank_name;
	new_bank->base = base;
	new_bank->write_start_alignment = new_bank->write_end_alignment = FLASH_WRITE_ALIGN_SECTOR;
	new_bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;
	new_bank->bus_width = new_bank->chip_width = 4;
	new_bank->default_padded_value = new_bank->erased_value = 0x00;
	new_bank->is_memory_mapped = true;

	if (add_to_list)
		flash_bank_add(new_bank);

	return new_bank;
}

static int maybe_add_sflash_bank(struct flash_bank *bank, const char *name, uint32_t area_idx)
{
	struct target *target = bank->target;
	int hr = ERROR_OK;

	struct flash_bank *sflash_bank = NULL;
	sflash_bank = psoc4_alloc_bank(bank, 0, 0x0FFFF000, name, false);
	if (sflash_bank) {
		struct psoc4_info *info = sflash_bank->driver_priv;
		hr = psoc4_read_family(target, info);
		if (hr != ERROR_OK)
			goto free_sflash;

		hr = psoc4_read_geometry(target, info);
		if (hr != ERROR_OK)
			goto free_sflash;

		info->sflash.area_idx = area_idx;
		hr = is_psoc4hv(bank) ? psoc4hv_decode_sflash(sflash_bank) : psoc4_decode_sflash(sflash_bank);
		if (hr != ERROR_OK)
			goto free_sflash;

		if (sflash_bank->size)
			flash_bank_add(sflash_bank);
		else
			goto free_sflash;
	}

	return ERROR_OK;

free_sflash:
	free(sflash_bank->driver_priv);
	free(sflash_bank->name);
	free(sflash_bank);
	return hr;
}

COMMAND_HANDLER(psoc4_handle_add_flash_banks)
{
	if (CMD_ARGC)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	struct flash_bank *bank;
	int hr = get_flash_bank_by_addr(target, 0, true, &bank);
	if (hr != ERROR_OK)
		return hr;

	struct psoc4_info p4_info;
	hr = psoc4_read_family(target, &p4_info);
	if (hr != ERROR_OK)
		return hr;

	/* PSoC4-HV: Check if FlashController #1 is present, add WorkFlash if so */
	if (is_psoc4hv(bank)) {
		p4_info.flctrl_idx = 1;
		hr = psoc4_read_geometry(target, &p4_info);
		if (hr == ERROR_OK)
			psoc4_alloc_bank(bank, 1, 0x1F000000, "wflash", true);
	}

	/* Devices with row size = 64b does not have SFlash */
	struct psoc4_info *main_info = bank->driver_priv;

	/* SFlash is not available or occupied by TRIMS on some parts */
	if (main_info->row_size > 64 && !(main_info->family->flags & PSOC4_NO_SFLASH)) {

		/* Add SupervisoryFlash #0 */
		hr = maybe_add_sflash_bank(bank, "sflash", 0);
		if (hr != ERROR_OK)
			return hr;

		/* PSoC4-HV: Check for SupervisoryFlash #1 */
		if (is_psoc4hv(bank)) {
			hr = maybe_add_sflash_bank(bank, "sflash1", 1);
			if (hr != ERROR_OK)
				return hr;
		}
	}

	struct flash_bank *b;
	b = psoc4_alloc_bank(bank, 0, BANK_FLASHP_ADDR_MASKED, "flash_prot", true);
	b->is_memory_mapped = false;
	b = psoc4_alloc_bank(bank, 0, BANK_CHIPP_ADDR_MASKED, "chip_prot", true);
	b->is_memory_mapped = false;

	return hr;
}

COMMAND_HANDLER(psoc4_handle_ecc_error_reporting)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], g_ecc_enabled);

	struct target *target = get_current_target(CMD_CTX);

	/* Disable FAULT[1] */
	int hr = target_write_u32(target, FAULT_CTL(1), 0);
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(target, FAULT_MASK0(1), 0);
	if (hr != ERROR_OK)
		return hr;

	/* Configure FAULT[0] */
	hr = target_write_u32(target, FAULT_CTL(0), 0);
	if (hr != ERROR_OK)
		return hr;

	const uint32_t fault_mask0 = g_ecc_enabled ? FAULT_ENABLE_ECC_MASK : 0;
	hr = target_write_u32(target, FAULT_MASK0(0), fault_mask0);
	if (hr != ERROR_OK)
		return hr;

	LOG_INFO("ECC error reporting is now %s", g_ecc_enabled ? "Enabled" : "Disabled");
	return ERROR_OK;
}

COMMAND_HANDLER(psoc4_handle_disable_cpu_read_relocations_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return psoc4_disable_remap(get_current_target(CMD_CTX));
}

static const struct command_registration psoc4_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = psoc4_handle_mass_erase,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Erases all flash banks and/or unlocks the chip",
	},
	{
		.name = "chip_protect",
		.handler = psoc4_handle_chip_protect,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Moves the part to PROTECTED state",
	},
	{
		.name = "kill_mode",
		.handler = psoc4_handle_kill_mode,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Moves the part to KILL state",
	},
	{
		.name = "reset_halt",
		.handler = psoc4_handle_reset_halt,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Tries to simulate broken Vector Catch",
	},
	{
		.name = "silicon_info",
		.handler = psoc4_handle_silicon_info,
		.mode = COMMAND_EXEC,
		.usage = "[sysreq]",
		.help = "Prints detailed info about the connected silicon",
	},
	{
		.name = "add_flash_banks",
		.handler = psoc4_handle_add_flash_banks,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "",
	},
	{
		.name = "ecc_error_reporting",
		.handler = psoc4_handle_ecc_error_reporting,
		.mode = COMMAND_EXEC,
		.usage = "on|off",
		.help = "Controls ECC error reporting",
	},
	{
		.name = "disable_cpu_read_relocations",
		.handler = psoc4_handle_disable_cpu_read_relocations_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Disables Reset Vector fetch relocation",
	},
	COMMAND_REGISTRATION_DONE,
};

static const struct command_registration psoc4_command_handlers[] = {
	{
		.name = "psoc4",
		.mode = COMMAND_ANY,
		.help = "PSoC 4 flash command group",
		.usage = "",
		.chain = psoc4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE,
};

const struct flash_driver psoc4_flash = {
	.name = "psoc4",
	.commands = psoc4_command_handlers,
	.flash_bank_command = psoc4_flash_bank_command,
	.erase = psoc4_erase,
	.protect = psoc4_protect,
	.write = psoc4_write,
	.read = psoc4_flash_read,
	.probe = psoc4_probe,
	.auto_probe = psoc4_auto_probe,
	.erase_check = psoc4_blank_check,
	.protect_check = psoc4_protect_check,
	.info = psoc4_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
