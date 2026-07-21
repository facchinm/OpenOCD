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
#include "rtos/rtos.h"

#define KiB(x)   ((x) << 10u)
#define MiB(x)   ((x) << 20u)

#define MFLASH_SECTOR_SIZE_256K        KiB(256u)
#define MFLASH_SECTOR_SIZE_128K        KiB(128u)
#define WFLASH_SECTOR_SIZE             KiB(32u)

#define MEM_WFLASH_SIZE                32768u
#define MEM_SFLASH_SIZE                32768u

#define MEM_BASE_IPC1                  0x40230040u
#define MEM_IPC1_INTR_MASK             0x40231008u
#define MEM_VTBASE1_CM0                0x402102B0u
#define MEM_VTBASE1_CM4                0x402102C0u
#define MEM_IPC1_ACQUIRE               (MEM_BASE_IPC1 + 0x00u)
#define MEM_IPC1_NOTIFY                (MEM_BASE_IPC1 + 0x08u)
#define MEM_IPC1_DATA                  (MEM_BASE_IPC1 + 0x0Cu)
#define MEM_IPC1_LOCK_STATUS           (MEM_BASE_IPC1 + 0x10u)

/* PSOC6A1M registers */
const struct mxs40_regs psoc6_ble2_regs = {
	.variant = MXS40_VARIANT_PSOC6_BLE2,
	.ipc_acquire = MEM_IPC1_ACQUIRE,
	.ipc_notify = MEM_IPC1_NOTIFY,
	.ipc_data = MEM_IPC1_DATA,
	.ipc_lock_stat = MEM_IPC1_LOCK_STATUS,
	.ipc_intr = MEM_IPC1_INTR_MASK,
	.ipc_intr_msk = IPC_INTR_MASK_2_CORE,
	.vtbase = { MEM_VTBASE1_CM0, MEM_VTBASE1_CM4, 0, 0, 0},
	.mem_base_main = {
		{0xFF000000, 0x10000000},
		BANK_MATCH_END
	},
	.mem_base_work = {
		{0xFF000000, 0x14000000},
		BANK_MATCH_END
	},
	.mem_base_sflash = {
		{0xFF000000, 0x16000000},
		BANK_MATCH_END
	},
};

#define MEM_BASE_IPC2                  0x40220040u
#define MEM_IPC2_INTR_MASK             0x40221008u
#define MEM_VTBASE2_CM0                0x40201120u
#define MEM_VTBASE2_CM4                0x40200200u
#define MEM_IPC2_ACQUIRE               (MEM_BASE_IPC2 + 0x00u)
#define MEM_IPC2_NOTIFY                (MEM_BASE_IPC2 + 0x08u)
#define MEM_IPC2_DATA                  (MEM_BASE_IPC2 + 0x0Cu)
#define MEM_IPC2_LOCK_STATUS           (MEM_BASE_IPC2 + 0x1Cu)

/* MACAW devices - IPC[1] is dedicated for DAP */
#define MEM_BASE_IPC3                  0x40220020u
#define MEM_IPC3_ACQUIRE               (MEM_BASE_IPC3 + 0x00u)
#define MEM_IPC3_NOTIFY                (MEM_BASE_IPC3 + 0x08u)
#define MEM_IPC3_DATA                  (MEM_BASE_IPC3 + 0x0Cu)
#define MEM_IPC3_LOCK_STATUS           (MEM_BASE_IPC3 + 0x1Cu)

/* PSoC6A2M registers */
const struct mxs40_regs psoc6_2m_regs = {
	.variant = MXS40_VARIANT_PSOC6A_2M,
	.ipc_acquire = MEM_IPC2_ACQUIRE,
	.ipc_notify = MEM_IPC2_NOTIFY,
	.ipc_data = MEM_IPC2_DATA,
	.ipc_lock_stat = MEM_IPC2_LOCK_STATUS,
	.ipc_intr = MEM_IPC2_INTR_MASK,
	.ipc_intr_msk = IPC_INTR_MASK_2_CORE,
	.ppu_flush = 0x40010100,
	.vtbase = { MEM_VTBASE2_CM0, MEM_VTBASE2_CM4, 0, 0, 0},
	.mem_base_main = {
		{0xFF000000, 0x10000000},
		BANK_MATCH_END
	},
	.mem_base_work = {
		{0xFF000000, 0x14000000},
		BANK_MATCH_END
	},
	.mem_base_sflash = {
		{0xFF000000, 0x16000000},
		BANK_MATCH_END
	},

};
/* Macaw registers */
const struct mxs40_regs macaw_regs = {
	.variant = MXS40_VARIANT_MACAW,
	.ipc_acquire = MEM_IPC3_ACQUIRE,
	.ipc_notify = MEM_IPC3_NOTIFY,
	.ipc_data = MEM_IPC3_DATA,
	.ipc_lock_stat = MEM_IPC3_LOCK_STATUS,
	.ipc_intr = MEM_IPC2_INTR_MASK,
	.ipc_intr_msk = IPC_INTR_MASK_1_CORE,
	.vtbase = { MEM_VTBASE2_CM0, 0, 0, 0, 0},
};

// Define SFlash layout (USER, TOC2, NAR, KEY)
// All regions must have '0xEE' attribute except NAR - '0xC0'
static const struct sflash_region psoc6_safe_sflash_regions[4] = {
	{0x16000800, 0x800, 0xEE},
	{0x16001A00, 0x200, 0xC0},
	{0x16005A00, 0xC00, 0xEE},
	{0x16007C00, 0x400, 0xEE},
};

/** ***********************************************************************************************
 * @brief Checks for known bugs in the boot code, updates mxs40_bank_info accordingly
 *
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_check_quirks(struct flash_bank *bank)
{
	struct mxs40_bank_info *info = bank->driver_priv;

	/* Pre-ES100 FlashBoot (older than 3.1.0.129) has buggy EraseSubSector API */
	uint32_t fb_ver_hi;
	int hr = target_read_u32(bank->target, 0x16002004, &fb_ver_hi);
	if (hr != ERROR_OK)
		return hr;

	uint32_t  fb_ver_lo;
	hr = target_read_u32(bank->target, 0x16002018, &fb_ver_lo);
	if (hr != ERROR_OK)
		return hr;

	uint8_t b0 =  fb_ver_hi >> 28;
	uint8_t b1 = (fb_ver_hi >> 24) & 0xF;
	uint8_t b2 = (fb_ver_hi >> 16) & 0xFF;

	uint16_t fb_build = (fb_ver_lo & 0xFFFF);
	uint8_t fb_patch = fb_ver_lo >> 24;

	if (b0 == 2 && b1 == 3 && b2 == 1 && fb_patch == 0 && fb_build < 129) {
		LOG_DEBUG("Chip has EraseSubSector bug, FB %d.%d.%d.(%d < 129)", b1, b2, fb_patch, fb_build);
		info->has_erase_subsector_bug = true;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Probes the device and populates related data structures with target flash geometry data.
 * This is done in non-intrusive way, no SROM API calls are involved so GDB can safely attach to a
 * running target. Function assumes that size of Work Flash is 32kB (true for all current part numbers)
 *
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psoc6_probe(struct flash_bank *bank)
{
	struct mxs40_bank_info *info = bank->driver_priv;
	struct target *target = bank->target;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
		bank->num_sectors = 0;
	}

	uint32_t row_sz = 512;
	size_t bank_size = 0;

	if (mxs40_flash_bank_matches(bank, info->regs->mem_base_main)) {
		if (!info->size_override)
			info->size_override = mxs40_probe_mem_area(target, bank->base,
													   KiB(128), KiB(1));

		if (!info->size_override) {
			LOG_ERROR("MainFlash size is unknown");
			return ERROR_FLASH_BANK_INVALID;
		}
		bank_size = info->size_override;
	} else if (mxs40_flash_bank_matches(bank, info->regs->mem_base_work))
		bank_size = MEM_WFLASH_SIZE;
	else if (mxs40_flash_bank_matches(bank, info->regs->mem_base_sflash))
		bank_size = MEM_SFLASH_SIZE;

	if (bank_size == 0) {
		LOG_ERROR("Invalid Flash Bank base address in config file");
		return ERROR_FLASH_BANK_INVALID;
	}

	int hr = psoc6_check_quirks(bank);
	if (hr != ERROR_OK)
		return hr;

	size_t num_sectors = bank_size / row_sz;
	bank->size = bank_size;
	bank->num_sectors = num_sectors;

	bank->chip_width = 4;
	bank->bus_width = 4;
	bank->erased_value = 0;
	bank->default_padded_value = 0;
	bank->sectors = alloc_block_array(0, row_sz, num_sectors);

	bank->write_start_alignment = row_sz;
	bank->write_end_alignment = row_sz;
	bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;

	info->page_size = row_sz;
	info->is_probed = true;

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
static size_t psoc6_erase_builder(struct flash_bank *bank, int first, int last, uint32_t *address_buffer)
{
	struct mxs40_bank_info *info = bank->driver_priv;
	uint32_t sector_size = 0;

	if(mxs40_flash_bank_matches(bank, info->regs->mem_base_work)) {
		sector_size = WFLASH_SECTOR_SIZE;
	} else if (mxs40_flash_bank_matches(bank, info->regs->mem_base_main)) {
		sector_size = bank->size > KiB(256) ? MFLASH_SECTOR_SIZE_256K : MFLASH_SECTOR_SIZE_128K;
	}

	assert(sector_size != 0);

	/* Number of rows in single sector */
	const int rows_in_sector = sector_size / info->page_size;
	const int rows_in_sub_sector = 8;
	size_t sector_count = 0;

	while (last >= first) {
		const uint32_t address = bank->base + first * info->page_size;
		if ((first % rows_in_sector) == 0 && (last - first + 1) >= rows_in_sector) {
			/* Erase Sector if we are on sector boundary and erase size covers whole sector */
			address_buffer[sector_count++] = address | 1u;
			first += rows_in_sector;
		} else if (!info->has_erase_subsector_bug &&
				  (first % rows_in_sub_sector) == 0 && (last - first + 1) >= rows_in_sub_sector) {
			/* Erase Sub-Sector if we are on sub-sector boundary and erase size covers whole sub-sector */
			address_buffer[sector_count++] = address | 2u;
			first += rows_in_sub_sector;
		} else {
			/* Erase Row otherwise */
			address_buffer[sector_count++] = address;
			first += 1;
		}
	}

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
static int psoc6_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct mxs40_bank_info *info = bank->driver_priv;

	if (mxs40_flash_bank_matches(bank, info->regs->mem_base_sflash))
		return mxs40_erase_sflash(bank, first, last);

	if(bank->target->coreid != 0xFF)
		return mxs40_erase_with_algo(bank, first, last, psoc6_erase_builder);

	/* Fallback for SYS_AP */
	struct target *target = bank->target;
	int hr = mxs40_sromalgo_prepare(bank);
	if (hr != ERROR_OK)
		goto exit;

	uint32_t sector_size = 0;
	if(mxs40_flash_bank_matches(bank, info->regs->mem_base_work)) {
		sector_size = WFLASH_SECTOR_SIZE;
	} else if (mxs40_flash_bank_matches(bank, info->regs->mem_base_main)) {
		sector_size = bank->size > KiB(256) ? MFLASH_SECTOR_SIZE_256K : MFLASH_SECTOR_SIZE_128K;
	}

	assert(sector_size != 0);

	/* Number of rows in single sector */
	const unsigned int rows_in_sector = sector_size / info->page_size;

	progress_init(last - first + 1, ERASING);
	while (last >= first) {
		const uint32_t address = bank->base + first * info->page_size;
		/* Erase Sector if we are on sector boundary and erase size covers whole sector */
		if ((first % rows_in_sector) == 0 && (last - first + 1) >= rows_in_sector) {
			hr = mxs40_erase_row(bank, address, true);
			if (hr != ERROR_OK)
				goto exit;

			first += rows_in_sector;
		} else {
			/* Perform Row Erase otherwise */
			hr = mxs40_erase_row(bank, address, false);
			if (hr != ERROR_OK)
				goto exit;

			first += 1;
		}
		progress_left(last - first + 1);
	}

exit:
	mxs40_sromalgo_release(target);
	progress_done(hr);
	return hr;
}

FLASH_BANK_COMMAND_HANDLER(psoc6_ble2_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	static const uint8_t p6_program_algo[] = {
		#include "../../../../contrib/loaders/flash/psoc6/psoc6_write.inc"
	};

	static const uint8_t p6_erase_algo[] = {
		#include "../../../../contrib/loaders/flash/psoc6/psoc6_erase.inc"
	};

	struct mxs40_bank_info *info = calloc(1, sizeof(struct mxs40_bank_info));
	info->program_algo_p = p6_program_algo;
	info->program_algo_size = sizeof(p6_program_algo);
	info->erase_algo_p = p6_erase_algo;
	info->erase_algo_size = sizeof(p6_erase_algo);
	info->regs = &psoc6_ble2_regs;
	info->sflash_regions = psoc6_safe_sflash_regions;
	info->size_override = bank->size;
	bank->driver_priv = info;

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(psoc6_2m_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	static const uint8_t p6_2m_program_algo[] = {
		#include "../../../../contrib/loaders/flash/psoc6/psoc62m_write.inc"
	};

	static const uint8_t p6_2m_erase_algo[] = {
		#include "../../../../contrib/loaders/flash/psoc6/psoc62m_erase.inc"
	};

	struct mxs40_bank_info *info = calloc(1, sizeof(struct mxs40_bank_info));
	info->program_algo_p = p6_2m_program_algo;
	info->program_algo_size = sizeof(p6_2m_program_algo);
	info->erase_algo_p = p6_2m_erase_algo;
	info->erase_algo_size = sizeof(p6_2m_erase_algo);
	info->regs = &psoc6_2m_regs;
	info->sflash_regions = psoc6_safe_sflash_regions;
	info->size_override = bank->size;
	bank->driver_priv = info;

	return ERROR_OK;
}

COMMAND_HANDLER(psoc6_handle_secure_acquire)
{
	int hr;
	struct timeout to;
	struct target *target = get_current_target(CMD_CTX);
	uint32_t ap = target->coreid == 0xFF ? 0 : target_to_cm(target)->apsel;
	bool acquire_mode_halt;
	bool do_handshake;


	if(CMD_ARGC != 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t magic_addr;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], magic_addr);

	if(strcmp(CMD_ARGV[1], "run") == 0) {
		acquire_mode_halt = false;
	} else if(strcmp(CMD_ARGV[1], "halt") == 0) {
		acquire_mode_halt = true;
	} else {
		LOG_ERROR("Invalid mode for secure_acquire: '%s'. "
				  "Only 'run' and 'halt' are currently supported.", CMD_ARGV[1]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if(strcmp(CMD_ARGV[2], "handshake") == 0) {
		do_handshake = true;
	} else if(strcmp(CMD_ARGV[2], "no_handshake") == 0) {
		do_handshake = false;
	} else {
		LOG_ERROR("Invalid handshake mode for secure_acquire: '%s'. "
				  "Only 'handshake' and 'no_handshake' are currently supported.", CMD_ARGV[2]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t timeout;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], timeout);

	LOG_INFO("Waiting up to %d.%d sec for the bootloader to open AP #%d...",
			 timeout / 1000u, timeout % 1000u, ap);

	const enum log_levels old_level = change_debug_level(LOG_LVL_USER);
	mxs40_timeout_init(&to, timeout);

	const char *error_msg = "";

	/* Wait for boot code to open corresponding AP */
	while(!mxs40_timeout_expired(&to)) {
		keep_alive();

		error_msg = "Failed to access debug sybsystem";
		target->examined = false;
		if(target_examine_one(target) == ERROR_OK)
			break;
	};

	/* Set TEST_MODE bit (only if we are going to halt) */
	while(do_handshake && acquire_mode_halt && !mxs40_timeout_expired(&to)) {
		keep_alive();

		error_msg = "failed to write SRSS.TST_MODE register";
		if(target_write_u32(target, 0x40260100, 0x80000000) != ERROR_OK)
			continue;

		error_msg = "";
		break;
	};
	change_debug_level(old_level);

	if(mxs40_timeout_expired(&to)) {
		LOG_ERROR("AP #%d is still not opened: %s, giving up", ap, error_msg);
		return ERROR_TARGET_FAILURE;
	}

	/* Wait for handshake (only if we are going to halt) */
	if(do_handshake && acquire_mode_halt) {
		LOG_INFO("Waiting up to %d.%d sec for the handshake from the target...",
				 timeout / 1000u, timeout % 1000u);

		change_debug_level(LOG_LVL_USER);
		mxs40_timeout_init(&to, timeout);
		while(!mxs40_timeout_expired(&to)) {
			keep_alive();

			uint32_t ipc_data;
			if(target_read_u32(target, magic_addr, &ipc_data) != ERROR_OK)
				continue;

			if(ipc_data == 0x12344321)
				break;
		};
		change_debug_level(old_level);

		if(mxs40_timeout_expired(&to))
			LOG_WARNING("No handshake from the target, continuing anyway");
	}

	hr = target_poll(target);
	if(hr != ERROR_OK)
		return hr;

	hr = target_poll(target);
	if(hr != ERROR_OK)
		return hr;

	/* Halt the target, if requested */
	if(acquire_mode_halt) {
		hr = target_halt(target);
		if(hr != ERROR_OK)
			return hr;

		hr = target_wait_state(target, TARGET_HALTED, 1000);
		if(hr != ERROR_OK)
			return hr;
	}

	/* Wipe-out previous RTOS state, if any */
	if(target->rtos_wipe_on_reset_halt)
		rtos_wipe(target);

	return ERROR_OK;
}

static const struct command_registration psoc6_exec_command_handlers[] = {
	{
		.name = "secure_acquire",
		.handler = psoc6_handle_secure_acquire,
		.mode = COMMAND_EXEC,
		.usage = "<psoc6|psoc6_2m> <run|halt> <handshake|no_handshake> <timeout>",
		.help = "",
	},
	{
		.chain = mxs40_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration psoc6_command_handlers[] = {
	{
		.name = "psoc6",
		.mode = COMMAND_ANY,
		.help = "PSoC 6 flash command group",
		.usage = "",
		.chain = psoc6_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/* Flash driver for PSoC6 BLE2 */
const struct flash_driver psoc6_flash = {
	.name = "psoc6",
	.commands = psoc6_command_handlers,
	.flash_bank_command = psoc6_ble2_flash_bank_command,
	.erase = psoc6_erase,
	.protect = mxs40_protect,
	.write = mxs40_program,
	.read = default_flash_read,
	.probe = psoc6_probe,
	.auto_probe = mxs40_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mxs40_protect_check,
	.info = mxs40_get_info,
	.free_driver_priv = mxs40_free_driver_priv,
};

/* Flash driver for PSoC6 BLE2 */
const struct flash_driver psoc6_2m_flash = {
	.name = "psoc6_2m",
	.commands = psoc6_command_handlers,
	.flash_bank_command = psoc6_2m_flash_bank_command,
	.erase = psoc6_erase,
	.protect = mxs40_protect,
	.write = mxs40_program,
	.read = default_flash_read,
	.probe = psoc6_probe,
	.auto_probe = mxs40_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mxs40_protect_check,
	.info = mxs40_get_info,
	.free_driver_priv = mxs40_free_driver_priv,
};
