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

#include "flash/nor/mxs40/mxs40.h"
#include "flash/nor/imp.h"
#include "target/target.h"
#include "target/cortex_m.h"
#include "flash/progress.h"

#define MEM_SFLASH_SIZE             32768u

#define FLASHC_BASE                 0x40240000
#define FLASHC_FLASH_CTL            (FLASHC_BASE + 0x00)
#define FLASH_CTL_MAIN_ECC_EN       (0x01 << 16)
#define FLASH_CTL_WORK_ECC_EN       (0x01 << 20)
#define FLASH_CTL_MAIN_ERR_SILENT   (0x01 << 18)
#define FLASH_CTL_WORK_ERR_SILENT   (0x01 << 22)
#define FLASH_CTL_MAIN_ECC_INJ_EN   (0x01 << 17)
#define FLASH_CTL_WORK_ECC_INJ_EN   (0x01 << 21)

#define FAULT_STRUCT_SIZE       0x00000100
#define FAULT_STATUS_VALID_MASK 0x80000000
#define FAULT_BASE(n)           (0x40210000 + (n) * FAULT_STRUCT_SIZE)

#define FAULT_CTL(n)      (FAULT_BASE((n)) + 0x00)
#define FAULT_STATUS(n)   (FAULT_BASE((n)) + 0x0C)
#define FAULT_DATA0(n)    (FAULT_BASE((n)) + 0x10)
#define FAULT_DATA1(n)    (FAULT_BASE((n)) + 0x14)
#define FAULT_DATA2(n)    (FAULT_BASE((n)) + 0x18)
#define FAULT_DATA3(n)    (FAULT_BASE((n)) + 0x1C)
#define FAULT_MASK0(n)    (FAULT_BASE((n)) + 0x50)
#define FAULT_MASK1(n)    (FAULT_BASE((n)) + 0x54)
#define FAULT_MASK2(n)    (FAULT_BASE((n)) + 0x58)

#define FAULT_MASK1_MAIN_C_ECC  (0x01 << 17)
#define FAULT_MASK1_MAIN_NC_ECC (0x01 << 18)
#define FAULT_MASK1_WORK_C_ECC  (0x01 << 20)
#define FAULT_MASK1_WORK_NC_ECC (0x01 << 21)
#define FAULT_MASK1_ENABLE_ECC_MASK (FAULT_MASK1_MAIN_C_ECC | FAULT_MASK1_MAIN_NC_ECC | \
	FAULT_MASK1_WORK_C_ECC | FAULT_MASK1_WORK_NC_ECC)

#define MEM_BASE_IPC2                  0x40220040u
#define MEM_IPC2_INTR_MASK             0x40221008u
#define MEM_VTBASE2_CM0                0x40201120u /* CPUSS_CM0_VECTOR_TABLE_BASE   */
#define MEM_VTBASE2_CM4                0x40200200u /* CPUSS_CM4_VECTOR_TABLE_BASE   */
#define MEM_VTBASE2_CM70               0x40200200u /* CPUSS_CM7_0_VECTOR_TABLE_BASE */
#define MEM_VTBASE2_CM71               0x40200600u /* CPUSS_CM7_1_VECTOR_TABLE_BASE */
#define MEM_VTBASE2_CM72               0x40200A00u /* CPUSS_CM7_2_VECTOR_TABLE_BASE */
#define MEM_VTBASE2_CM73               0x40200E00u /* CPUSS_CM7_3_VECTOR_TABLE_BASE */
#define MEM_IPC2_ACQUIRE               (MEM_BASE_IPC2 + 0x00u)
#define MEM_IPC2_NOTIFY                (MEM_BASE_IPC2 + 0x08u)
#define MEM_IPC2_DATA                  (MEM_BASE_IPC2 + 0x0Cu)
#define MEM_IPC2_LOCK_STATUS           (MEM_BASE_IPC2 + 0x1Cu)

/* TRAVEO™II registers */
const struct mxs40_regs traveo2_regs = {
	.variant = MXS40_VARIANT_TRAVEO_II,
	.ipc_acquire = MEM_IPC2_ACQUIRE,
	.ipc_notify = MEM_IPC2_NOTIFY,
	.ipc_data = MEM_IPC2_DATA,
	.ipc_lock_stat = MEM_IPC2_LOCK_STATUS,
	.ipc_intr = MEM_IPC2_INTR_MASK,
	.ipc_intr_msk = IPC_INTR_MASK_2_CORE,
	.vtbase = {MEM_VTBASE2_CM0, MEM_VTBASE2_CM4, 0, 0, 0},
	.mem_base_main = {
		{0xFF000000, 0x10000000},
		BANK_MATCH_END
	},
	.mem_base_work = {
		{0xFF000000, 0x14000000},
		BANK_MATCH_END
	},
	.mem_base_sflash = {
		{0xFF000000, 0x17000000},
		BANK_MATCH_END
	}
};

#define MEM_BASE_IPC3                  0x40220060u
#define MEM_IPC3_ACQUIRE               (MEM_BASE_IPC3 + 0x00u)
#define MEM_IPC3_NOTIFY                (MEM_BASE_IPC3 + 0x08u)
#define MEM_IPC3_DATA                  (MEM_BASE_IPC3 + 0x0Cu)
#define MEM_IPC3_LOCK_STATUS           (MEM_BASE_IPC3 + 0x1Cu)

/* TRAVEO™II 8M registers */
const struct mxs40_regs traveo2_8m_regs = {
	.variant = MXS40_VARIANT_TRAVEO_II_8M,
	.ipc_acquire = MEM_IPC3_ACQUIRE,
	.ipc_notify = MEM_IPC3_NOTIFY,
	.ipc_data = MEM_IPC3_DATA,
	.ipc_lock_stat = MEM_IPC3_LOCK_STATUS,
	.ipc_intr = MEM_IPC2_INTR_MASK,
	.ipc_intr_msk = IPC_INTR_MASK_3_CORE,
	.vtbase = {MEM_VTBASE2_CM0, MEM_VTBASE2_CM70, MEM_VTBASE2_CM71, 0, 0},
	.mem_base_main = {
		{0xFF000000, 0x10000000},
		BANK_MATCH_END
	},
	.mem_base_work = {
		{0xFF000000, 0x14000000},
		BANK_MATCH_END
	},
	.mem_base_sflash = {
		{0xFF000000, 0x17000000},
		BANK_MATCH_END
	}
};

#define MEM_BASE_IPC5                  0x402200A0u
#define MEM_IPC5_ACQUIRE               (MEM_BASE_IPC5 + 0x00u)
#define MEM_IPC5_NOTIFY                (MEM_BASE_IPC5 + 0x08u)
#define MEM_IPC5_DATA                  (MEM_BASE_IPC5 + 0x0Cu)
#define MEM_IPC5_LOCK_STATUS           (MEM_BASE_IPC5 + 0x1Cu)

/* TRAVEO™II 16M registers */
const struct mxs40_regs traveo2_16m_regs = {
	.variant = MXS40_VARIANT_TRAVEO_II_16M,
	.ipc_acquire = MEM_IPC5_ACQUIRE,
	.ipc_notify = MEM_IPC5_NOTIFY,
	.ipc_data = MEM_IPC5_DATA,
	.ipc_lock_stat = MEM_IPC5_LOCK_STATUS,
	.ipc_intr = MEM_IPC2_INTR_MASK,
	.ipc_intr_msk = IPC_INTR_MASK_3_CORE,
	.vtbase = {MEM_VTBASE2_CM0, MEM_VTBASE2_CM70, MEM_VTBASE2_CM71, MEM_VTBASE2_CM72, MEM_VTBASE2_CM73},
	.mem_base_main = {
		{0xFF000000, 0x10000000},
		{0xFF000000, 0x18000000},
		{0xFFF00000, 0x1F000000}, /* Extended code region, algorithms for main flash reused */
		{0xFFF00000, 0x1F800000}, /* Extended Alt code region */
		BANK_MATCH_END
	},
	.mem_base_work = {
		{0xFF000000, 0x14000000},
		{0xFF000000, 0x1C000000},
		BANK_MATCH_END
	},
	.mem_base_sflash = {
		{0xFF000000, 0x17000000},
		BANK_MATCH_END
	}
};

static bool g_ecc_enabled;

// Define SFlash layout (USER, TOC2, NAR, KEY)
// All regions must have '0xEE' attribute except NAR - '0xC0'
static const struct sflash_region traveo2_safe_sflash_regions[4] = {
	{0x17000800, 0x800, 0xEE},
	{0x17001A00, 0x200, 0xC0},
	{0x17006400, 0xC00, 0xEE},
	{0x17007C00, 0x200, 0xEE},
};

/** ***********************************************************************************************
 * @brief Configures ECC error reporting on TRAVEO™II devices
 * @param target current target
 * @param enabled true if ECC error reporting should be enabled
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int traveo2_configure_ecc(struct target *target, bool enabled)
{
	int hr;

	for (size_t idx = 1; idx <= 3; idx++) {
		/* Disable FAULT[1] */
		hr = target_write_u32(target, FAULT_CTL(idx), 0);
		if (hr != ERROR_OK)
			return hr;

		hr = target_write_u32(target, FAULT_MASK0(idx), 0);
		if (hr != ERROR_OK)
			return hr;

		hr = target_write_u32(target, FAULT_MASK1(idx), 0);
		if (hr != ERROR_OK)
			return hr;

		hr = target_write_u32(target, FAULT_MASK2(idx), 0);
		if (hr != ERROR_OK)
			return hr;
	}

	/* Common config for FAULT[0] */
	hr = target_write_u32(target, FAULT_CTL(0), 0);
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(target, FAULT_MASK0(0), 0);
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(target, FAULT_MASK2(0), 0);
	if (hr != ERROR_OK)
		return hr;

	uint32_t flashc_ctl_val = 0;
	hr = target_read_u32(target, FLASHC_FLASH_CTL, &flashc_ctl_val);
	if (hr != ERROR_OK)
		return hr;

	if (enabled) {
		flashc_ctl_val |=
			(FLASH_CTL_MAIN_ECC_EN | FLASH_CTL_WORK_ECC_EN | FLASH_CTL_MAIN_ERR_SILENT |
			FLASH_CTL_WORK_ERR_SILENT);
		hr = target_write_u32(target, FLASHC_FLASH_CTL, flashc_ctl_val);
		if (hr != ERROR_OK)
			return hr;

		hr = target_write_u32(target, FAULT_MASK1(0), FAULT_MASK1_ENABLE_ECC_MASK);
		if (hr != ERROR_OK)
			return hr;
	} else {
		flashc_ctl_val &= ~(FLASH_CTL_MAIN_ECC_EN | FLASH_CTL_WORK_ECC_EN);
		hr = target_write_u32(target, FLASHC_FLASH_CTL, flashc_ctl_val);
		if (hr != ERROR_OK)
			return hr;
		hr = target_write_u32(target, FAULT_MASK1(0), 0);
		if (hr != ERROR_OK)
			return hr;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Performs initial setup of the TRAVEO™II target
 * @param bank The flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int traveo2_prepare(struct flash_bank *bank)
{
	int hr;

	hr = target_write_u32(bank->target, 0x4024F400, 0x01);
	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(bank->target, 0x4024F500, 0x01);
	if (hr != ERROR_OK)
		return hr;

	/* Disable Flash Security for the second Flash Controller, if present */
	struct flash_bank *fc1_main = NULL;
	get_flash_bank_by_addr(bank->target, TV2_MAIN_FLASH1_BASE, false, &fc1_main);
	if (fc1_main) {
		hr = target_write_u32(bank->target, 0x4025F400, 0x01);
		if (hr != ERROR_OK)
			return hr;

		hr = target_write_u32(bank->target, 0x4025F500, 0x01);
		if (hr != ERROR_OK)
			return hr;
	}

	hr = target_write_u32(bank->target, 0xE000E100, 0x03);
	if (hr != ERROR_OK)
		return hr;

	/* RAM0_CTL0.ECC_CHECK_DIS, bit 19 */
	uint32_t ram0_ctl0;
	hr = target_read_u32(bank->target, 0x40201300, &ram0_ctl0);
	if (hr != ERROR_OK)
		return hr;

	ram0_ctl0 |= BIT(19);

	hr = target_write_u32(bank->target, 0x40201300, ram0_ctl0);
	if (hr != ERROR_OK)
		return hr;

	return hr;
}

/** ***********************************************************************************************
 * @brief Performs Flash Read operation with ECC error reporting. This function is used in TRAVEO™II
 * devices only since PSoC6 does not support ECC.
 * @param bank The bank to read
 * @param buffer The data bytes read
 * @param offset The offset into the chip to read
 * @param count The number of bytes to read
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int traveo2_flash_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset,
	uint32_t count)
{
	if (!g_ecc_enabled)
		return default_flash_read(bank, buffer, offset, count);

	assert(offset % 4 == 0);
	assert(count % 4 == 0);
	int hr = target_write_u32(bank->target, FAULT_STATUS(0), 0x00);
	if (hr != ERROR_OK)
		return hr;

	uint32_t address = bank->base + offset;

	while (count) {
		hr = target_read_buffer(bank->target, address, 4, buffer);
		if (hr != ERROR_OK)
			return hr;

		uint32_t status = 0;
		do {
			hr = target_read_u32(bank->target, FAULT_STATUS(0), &status);
			if (hr != ERROR_OK)
				return hr;

			if (status & FAULT_STATUS_VALID_MASK) {
				uint32_t er_addr;
				hr = target_read_u32(bank->target, FAULT_DATA0(0), &er_addr);
				if (hr != ERROR_OK)
					return hr;

				er_addr |= (address & 0xF8000000);
				LOG_WARNING("ECC Error at address 0x%08X", er_addr);

				hr = target_write_u32(bank->target, FAULT_STATUS(0), 0x00);
				if (hr != ERROR_OK)
					return hr;
			}
		} while (status & FAULT_STATUS_VALID_MASK);

		count -= 4;
		buffer += 4;
		address += 4;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief builds data buffer with list of sectors addresses to erase
 * @param bank current flash bank
 * @param first first sector to erase
 * @param last last sector to erase
 * @param address_buffer pointer to the buffer
 * @return number of addresses in the buffer
 *************************************************************************************************/
static size_t traveo2_erase_builder(struct flash_bank *bank, int first, int last, uint32_t *address_buffer)
{
	size_t sector_count = 0;
	for(int i = first; i <= last; i++)
		address_buffer[sector_count++] = (bank->base + bank->sectors[i].offset) | 1u;

	return sector_count;
}

/** ***********************************************************************************************
 * @brief Performs Erase operation. Function will try to use biggest erase block possible to
 * speedup the operation.
 *
 * @param bank current flash bank
 * @param first first sector to erase
 * @param last last sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int traveo2_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct mxs40_bank_info *info = bank->driver_priv;

	if (mxs40_flash_bank_matches(bank, info->regs->mem_base_sflash)) {
		mxs40_erase_sflash(bank, first, last);
		return ERROR_OK;
	}

	if(bank->target->coreid != 0xFF)
		return mxs40_erase_with_algo(bank, first, last, traveo2_erase_builder);

	/* Fallback for SYS_AP */
	struct target *target = bank->target;
	int hr = mxs40_sromalgo_prepare(bank);
	if (hr != ERROR_OK)
		goto exit;

	progress_init(last - first + 1, ERASING);
	for (unsigned int i = first; i <= last; i++) {
		hr = mxs40_erase_row(bank, bank->base + bank->sectors[i].offset, true);
		if (hr != ERROR_OK)
			goto exit;
		progress_sofar(i - first + 1);
	}

exit:
	mxs40_sromalgo_release(target);
	progress_done(hr);
	return hr;
}

/** *************************************************************************************
 * @brief Returns largest possible Work Flash programming page size
 ***************************************************************************************/
static uint32_t traveo2_get_wflash_page_size(struct target *target)
{
	#include "../../../../contrib/loaders/flash/psoc6/flashboot.h"

	uint32_t val;
	int hr = target_read_u32(target, FLASHBOOT_VER_HI_ADDR, &val);
	if(hr != ERROR_OK)
		goto error;

	val = val >> 28u;
	if(val < 2)
		return 4;

	hr = target_read_u32(target, FLASHBOOT_VER_LO_ADDR, &val);
	if(hr != ERROR_OK)
		goto error;

	val = val & 0xFFFFu;
	uint32_t wf_page = (val < FLASHBOOT_BUILD_BULK_SUPPORTED) ? 4 : 512;
	return wf_page;

error:
	LOG_WARNING("Failed to determine WorkFlash page saze, assuming 4 bytes");
	return 4;
}

/** ***********************************************************************************************
 * @brief Probes the device and populates related data structures with target flash geometry data.
 * This is done in non-intrusive way, no SROM API calls are involved so GDB can safely attach to a
 * running target. This is for TRAVEO™II devices only.
 *
 * @param bank current flash bank
 * @param geometry value in FLASHC.GEONETRY register
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int traveo2_probe(struct flash_bank *bank)
{
	struct mxs40_bank_info *info = bank->driver_priv;
	uint32_t large_sec_num = 0;
	uint32_t small_sec_num = 0;
	uint32_t large_sec_size = 0;
	uint32_t small_sec_size = 0;
	int hr;

	hr = traveo2_configure_ecc(bank->target, g_ecc_enabled);
	if (hr != ERROR_OK)
		return hr;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
		bank->num_sectors = 0;
	}

	if (mxs40_flash_bank_matches(bank, info->regs->mem_base_main)) {

		if (!info->size_override) {
			LOG_ERROR("Unknown Main Flash geometry");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		large_sec_num = info->size_override >> 16;
		small_sec_num = info->size_override & 0xFFFF;

		LOG_DEBUG("MainFlash geometry:"
			" %d large sectors + %d small sectors",
			large_sec_num, small_sec_num);

		large_sec_size = 32768;
		small_sec_size = 8192;
		info->page_size = 512;

	} else if (mxs40_flash_bank_matches(bank, info->regs->mem_base_work)) {

		if (!info->size_override) {
			LOG_ERROR("Unknown Work Flash geometry");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		large_sec_num = info->size_override >> 16;
		small_sec_num = info->size_override & 0xFFFF;

		LOG_DEBUG("WorkFlash geometry:"
			" %d large sectors + %d small sectors",
			large_sec_num, small_sec_num);

		large_sec_size = 2048;
		small_sec_size = 128;
		info->page_size = traveo2_get_wflash_page_size(bank->target);
	} else if (mxs40_flash_bank_matches(bank, info->regs->mem_base_sflash)) {
		info->page_size = 512;
		bank->size = MEM_SFLASH_SIZE;
		bank->num_sectors = bank->size / info->page_size;

		large_sec_num = bank->num_sectors;
		large_sec_size = info->page_size;
		small_sec_num = 0;
		small_sec_size = 0;
	} else
		return ERROR_FLASH_BANK_INVALID;

	bank->size = large_sec_num * large_sec_size + small_sec_num * small_sec_size;
	bank->chip_width = 4;
	bank->bus_width = 4;
	bank->erased_value = 0xFF;
	bank->default_padded_value = 0xFF;

	bank->write_start_alignment = info->page_size;
	bank->write_end_alignment = info->page_size;
	bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;

	bank->num_sectors = large_sec_num + small_sec_num;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	uint32_t sec_offset = 0;
	uint32_t sec_size;
	for (size_t i = 0; i < large_sec_num + small_sec_num; i++) {
		if (i < large_sec_num)
			sec_size = large_sec_size;
		else
			sec_size = small_sec_size;

		bank->sectors[i].size = sec_size;
		bank->sectors[i].offset = sec_offset;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
		sec_offset += sec_size;
	}

	info->is_probed = true;

	return 0;
}

static void traveo2_flash_bank_command_inner(struct flash_bank *bank)
{
	static const uint8_t traveo2_program_algo[] = {
		#include "../../../../contrib/loaders/flash/psoc6/tv2_write.inc"
	};

	static const uint8_t traveo2_erase_algo[] = {
		#include "../../../../contrib/loaders/flash/psoc6/tv2_erase.inc"
	};

	struct mxs40_bank_info *info = calloc(1, sizeof(struct mxs40_bank_info));
	info->program_algo_p = traveo2_program_algo;
	info->program_algo_size = sizeof(traveo2_program_algo);
	info->erase_algo_p = traveo2_erase_algo;
	info->erase_algo_size = sizeof(traveo2_erase_algo);
	info->size_override = bank->size;
	info->prepare_function = traveo2_prepare;
	info->sflash_regions = traveo2_safe_sflash_regions;

	/* Bank Size has special encoding in TVII (number of large and small sectors)
	 * nullify value in bank->size to not confuse the user. Size will be populated
	 * during first probing */
	bank->size = 0;
	bank->driver_priv = info;
}

FLASH_BANK_COMMAND_HANDLER(traveo2_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	traveo2_flash_bank_command_inner(bank);
	struct mxs40_bank_info *info = bank->driver_priv;
	info->regs = &traveo2_regs;

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(traveo2_8m_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	traveo2_flash_bank_command_inner(bank);
	struct mxs40_bank_info *info = bank->driver_priv;
	info->regs = &traveo2_8m_regs;

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(traveo2_16m_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	traveo2_flash_bank_command_inner(bank);
	struct mxs40_bank_info *info = bank->driver_priv;
	info->regs = &traveo2_16m_regs;

	return ERROR_OK;
}

COMMAND_HANDLER(traveo2_handle_ecc_error_reporting_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], g_ecc_enabled);

	struct target *target = get_current_target(CMD_CTX);

	int hr = traveo2_configure_ecc(target, g_ecc_enabled);
	LOG_INFO("ECC error reporting is now %s", g_ecc_enabled ? "Enabled" : "Disabled");

	return hr;
}

extern const struct command_registration tv2_wflash_command_handlers[];
static const struct command_registration tv2_exec_command_handlers[] = {
	{
		.name = "ecc_error_reporting",
		.handler = traveo2_handle_ecc_error_reporting_command,
		.mode = COMMAND_EXEC,
		.usage = "on|off",
		.help = "Controls ECC error reporting",
	},
	{
		.name = "wflash",
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "wflash command group",
		.chain = tv2_wflash_command_handlers,
	},
	{
		.chain = mxs40_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration traveo2_command_handlers[] = {
	{
		.name = "traveo2",
		.mode = COMMAND_ANY,
		.help = "Traveo-II flash command group",
		.usage = "",
		.chain = tv2_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration cat1c_command_handlers[] = {
	{
		.name = "cat1c",
		.mode = COMMAND_ANY,
		.help = "cat1c flash command group",
		.usage = "",
		.chain = tv2_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration xmc5xxx_command_handlers[] = {
	{
		.name = "xmc5xxx",
		.mode = COMMAND_ANY,
		.help = "XMC5xxx flash command group",
		.usage = "",
		.chain = tv2_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver traveo21_flash = {
	.name = "traveo21",
	.commands = traveo2_command_handlers,
	.flash_bank_command = traveo2_flash_bank_command,
	.erase = traveo2_erase,
	.protect = mxs40_protect,
	.write = mxs40_program,
	.read = traveo2_flash_read,
	.probe = traveo2_probe,
	.auto_probe = mxs40_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mxs40_protect_check,
	.info = mxs40_get_info,
	.free_driver_priv = mxs40_free_driver_priv,
};

const struct flash_driver traveo22_flash = {
	.name = "traveo22",
	.commands = traveo2_command_handlers,
	.flash_bank_command = traveo2_8m_flash_bank_command,
	.erase = traveo2_erase,
	.protect = mxs40_protect,
	.write = mxs40_program,
	.read = traveo2_flash_read,
	.probe = traveo2_probe,
	.auto_probe = mxs40_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mxs40_protect_check,
	.info = mxs40_get_info,
	.free_driver_priv = mxs40_free_driver_priv,
};

const struct flash_driver traveo23_flash = {
	.name = "traveo23",
	.commands = traveo2_command_handlers,
	.flash_bank_command = traveo2_16m_flash_bank_command,
	.erase = traveo2_erase,
	.protect = mxs40_protect,
	.write = mxs40_program,
	.read = traveo2_flash_read,
	.probe = traveo2_probe,
	.auto_probe = mxs40_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mxs40_protect_check,
	.info = mxs40_get_info,
	.free_driver_priv = mxs40_free_driver_priv,
};

const struct flash_driver xmc5xxx_flash = {
	.name = "xmc5xxx",
	.commands = xmc5xxx_command_handlers,
	.flash_bank_command = traveo2_flash_bank_command,
	.erase = traveo2_erase,
	.protect = mxs40_protect,
	.write = mxs40_program,
	.read = traveo2_flash_read,
	.probe = traveo2_probe,
	.auto_probe = mxs40_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mxs40_protect_check,
	.info = mxs40_get_info,
	.free_driver_priv = mxs40_free_driver_priv,
};

const struct flash_driver cat1c_flash = {
	.name = "cat1c",
	.commands = cat1c_command_handlers,
	.flash_bank_command = traveo2_8m_flash_bank_command,
	.erase = traveo2_erase,
	.protect = mxs40_protect,
	.write = mxs40_program,
	.read = traveo2_flash_read,
	.probe = traveo2_probe,
	.auto_probe = mxs40_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mxs40_protect_check,
	.info = mxs40_get_info,
	.free_driver_priv = mxs40_free_driver_priv,
};
