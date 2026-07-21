// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2023-2026, Infineon Technologies AG, or an affiliate of *
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

#include "flash/nor/imp.h"
#include "target/target.h"
#include "flash/progress.h"

#include "target/arm_adi_v5.h"
#include "target/cortex_m.h"
#include "target/breakpoints.h"
#include "target/target_type.h"
#include "target/algorithm.h"
#include "flash/nor/mxs40/mxs40.h"

#define FLASH_BANK_BASE_SECURE              0x32000000
#define FLASH_BANK_BASE_NON_SECURE          0x22000000
#define FLAH_BANK_BASE_VIRTUAL_SECURE       0x12000000
#define FLAH_BANK_BASE_VIRTUAL_NON_SECURE   0x02000000

#define SROMAPI_ERASE_ROW       0x1080FFE0u
#define SROMAPI_PROGRAM_ROW     0x1080FFE4u

#define SROMAPI_STACK_SIZE      4096
#define SROMAPI_TIMEOUT_MS      3000
#define FLASH_ROW_SIZE          512
#define USE_ASYNC_ALGOS         1
#define BLOCKING_OPERATION      0

#define DUAL_BKPT_INSTR         0xBE00BE00

#define VIRGIN_LCS              0x00
#define SORT_LCS                0x29

#define DUAL_BANK_MODE          (1 << 12)
#define DUAL_BANK_OFFSET        0x800000

#define VIRTUAL_BANK_NAME_BASE  "psc3.cm33.main1_cbus_"
#define VIRTUAL_BANK_OFFSET     0x20000000

#define SECURE                  true
#define NON_SECURE              false

#define KiB(x)                  ((x) << 10u)

struct psc3_info {
	uint32_t bank_size_override;
	uint32_t bank_size_max;
	uint32_t row_size;
	bool probed;
	bool dual_bank_mode;
};

#if !(USE_ASYNC_ALGOS)
typedef void (*cyboot_flash_callback_t)(void * ctx);
typedef struct
{
    uint32_t min_count;
    uint32_t max_count;
    uint32_t min_page_addr;
    uint32_t scratch_row_idx;
} cyboot_flash_refresh_t;

typedef struct {
    /*
    * [0] 0 -BLOCKING, 1-NON-BLOCKING
    * [1] RWW or STALL_READ, for BLOCKING
    * [2] REFRESH_CTL, determines who provides the data for Column 33.
    *   0- Flash API computes the data for column 33.
    *   1- column 33 is programmed as is. Up to a caller to provide the data.
    * [3] A mode for Program Sector and Program Bulk operations.
    *   0- ALL
    *   1- Even/Odd
    */
    uint32_t flags;
    uint32_t hv_params_addr;
    cyboot_flash_refresh_t *refresh;
 
    /* params for non-blocking operations */
    cyboot_flash_callback_t callback_pre_irq;
    cyboot_flash_callback_t callback_post_irq;
    cyboot_flash_callback_t callback_complete;
    uint32_t callback_param;
    uint32_t state; /* A state machine - EMPTY, ERASE_0, PROGRAM_0, WRITE_0,1 */
    uint32_t flash_addr; /* Non-blocking WriteRow */
    uint32_t data_addr;  /* Non-blocking WriteRow */
    uint32_t reserved[2]; /* size to be 8 bytes aligned */
} flash_context_t;
#endif

static uint32_t g_sflash_restrictions;

static int psc3_get_lifecycle(struct target *target, uint32_t *lifecycle);
static bool psc3_allow_write_sflash(struct target *target);

FLASH_BANK_COMMAND_HANDLER(psc3_flashbank_command)
{
	uint32_t row_size = FLASH_ROW_SIZE;

	if (CMD_ARGC < 6 || CMD_ARGC > 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 7)
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[6], row_size);

	struct psc3_info *info = calloc(1, sizeof(struct psc3_info));
	if (!info)
		return ERROR_FAIL;

	if (bank->size % row_size) {
		LOG_ERROR("%s is not a multiple of block size", bank->name);
		return ERROR_FAIL;
	}

	info->row_size = row_size;
	bank->driver_priv = info;

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Probes the device and populates related data structures with target flash geometry data.
 * If data isn't set from within tcl script, then flash is probed by sequential read of flash in
 * order to detect it's size.
 *
 * @param bank - current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int psc3_flash_probe(struct flash_bank *bank)
{
	struct psc3_info *info = (struct psc3_info *)bank->driver_priv;
	struct target *target = bank->target;

	if (bank->sectors){
		free (bank->sectors);
	}

	if (bank->size == 0) {
		/* Auto-detecting flash size is done onto NS banks*/
		if (strstr(bank->name, "_ns") != NULL) {
			info->bank_size_override = mxs40_probe_mem_area(target, bank->base,
														KiB(64), KiB(1));
			if (!info->bank_size_override) {
				LOG_ERROR("Fail to probe bank size for '%s'", bank->name);
				return ERROR_FLASH_BANK_INVALID;
			}
			//Apply limits for flash bank size in case of misoperation
			bank->size = (info->bank_size_override > info->bank_size_max) ? info->bank_size_max: info->bank_size_override;
		} else {
			struct flash_bank *ref_ns_bank;
			get_flash_bank_by_name("psc3.cm33.main0_ns", &ref_ns_bank);
			bank->size = ref_ns_bank->size;
			info->bank_size_override = ref_ns_bank->size;
		}
	}

	bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;
	bank->write_end_alignment = info->row_size;
	bank->write_start_alignment = info->row_size;
	bank->erased_value = bank->default_padded_value = 0x00;
	bank->num_sectors = bank->size / info->row_size;
	bank->sectors = alloc_block_array(0, info->row_size, bank->num_sectors);
	if (!bank->sectors) {
		LOG_ERROR("Error alloc memory");
		return ERROR_FAIL;
	}
	info->probed = true;

	return ERROR_OK;
}

static int psc3_flash_auto_probe(struct flash_bank *bank)
{
	struct psc3_info *info = (struct psc3_info *)bank->driver_priv;

	if (!info)
		return ERROR_FAIL;

	if (info->probed)
		return ERROR_OK;

	return psc3_flash_probe(bank);
}

/** ***********************************************************************************************
 * @brief Prints error code with error description
 *        Function forms tcl command to fetch device-specific boot statuses,
 *        Executes tcl srcipt from inside the *.c file.
 * @param op - flash operation in use (erase, write etc)
 * @param srom_result - return code from flash api
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static void print_flash_error(const char *op, uint32_t srom_result)
{
    const char *tcl_frame = "echo [format \"Error: psc3 %s operation failed: 0x%08X-%%s\" [get_boot_status_str 0x%08X]]";
    char *tcl_cmd = alloc_printf(tcl_frame, op, srom_result, srom_result);
    int rc = JIM_OK;

    if (tcl_cmd) {
        extern struct command_context *global_cmd_ctx;
        Jim_Interp *interp = global_cmd_ctx->interp;
        rc = Jim_Eval(interp, tcl_cmd);
    }

    if (!tcl_cmd || rc) {
        LOG_ERROR("psc3 %s operation failed: 0x%08X", op, srom_result);
    }

    free(tcl_cmd);
}

#if(USE_ASYNC_ALGOS)
static int psc3_flash_erase(struct flash_bank *bank, unsigned int first,
	unsigned int last)
{
	static const uint8_t erase_algo[] = {
		#include "../../../contrib/loaders/flash/psc3/psc3_erase.inc"
	};

	struct target *target = bank->target;

	struct working_area *wa_algorithm;
	struct working_area *wa_stack;
	struct working_area *wa_buffer;

	if (strstr(bank->name, "super") != NULL) {
		if (!psc3_allow_write_sflash(target)) {
			return ERROR_FAIL;
		}
	}

	uint32_t address_buffer[last - first + 1];
	memset(address_buffer, 0, sizeof(address_buffer));

	/* Allocate buffer for the algorithm */
	int hr = target_alloc_working_area(target, sizeof(erase_algo), &wa_algorithm);
	if (hr != ERROR_OK)
		return hr;

	/* Write the algorithm code */
	hr = target_write_buffer(target, wa_algorithm->address, sizeof(erase_algo), erase_algo);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Allocate buffer for the stack */
	hr = target_alloc_working_area(target, SROMAPI_STACK_SIZE, &wa_stack);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Allocate circular buffer for 16 addresses, this should be sufficient */
	hr = target_alloc_working_area(target, 16 * sizeof(uint32_t) + 8, &wa_buffer);
	if(hr != ERROR_OK)
		goto err_free_wa_stack;

	size_t num_addresses_in_buffer = last - first + 1;
	for(size_t i = 0; i < num_addresses_in_buffer; i++) {
		const uint32_t address = bank->base + (first + i) * FLASH_ROW_SIZE;
		address_buffer[i] = address;
	}

	struct armv7m_algorithm armv7m_algo;
	armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_algo.core_mode = ARM_MODE_THREAD;

	struct reg_param reg_params[4];
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "sp", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, wa_buffer->address);
	buf_set_u32(reg_params[1].value, 0, 32, wa_buffer->address + wa_buffer->size);
	buf_set_u32(reg_params[2].value, 0, 32, num_addresses_in_buffer);
	buf_set_u32(reg_params[3].value, 0, 32, wa_stack->address + wa_stack->size);

	progress_init(0, ERASING);
	hr = target_run_flash_async_algorithm(target, (const uint8_t *)address_buffer, num_addresses_in_buffer,
			sizeof(uint32_t), 0, NULL, ARRAY_SIZE(reg_params), reg_params,
			wa_buffer->address, wa_buffer->size,
			wa_algorithm->address, 0, &armv7m_algo);

	if (hr != ERROR_OK) {
		uint32_t srom_result = buf_get_u32(reg_params[0].value, 0, 32);
		print_flash_error("erase", srom_result);
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	target_free_working_area(target, wa_buffer);

err_free_wa_stack:
	target_free_working_area(target, wa_stack);

err_free_wa_algo:
	target_free_working_area(target, wa_algorithm);

	return hr;
}
#else
static int psc3_flash_erase(struct flash_bank *bank, unsigned int first,
	unsigned int last)
{
	struct target *target = bank->target;

	if (strstr(bank->name, "super") != NULL) {
		if (!psc3_allow_write_sflash(target)) {
			return ERROR_FAIL;
		}
	}

	/* Allocate Working Area for Stack and Flash algorithm */
	struct working_area *stack_area;
	int hr = target_alloc_working_area(target, sizeof(flash_context_t) + 4 + SROMAPI_STACK_SIZE , &stack_area);
	if (hr != ERROR_OK) {
		LOG_ERROR("WA failed (erase op)");
		return hr;
	}

	target_addr_t flash_ctx_offs = stack_area->address;
	target_addr_t bkpt_offs = flash_ctx_offs + sizeof(flash_context_t);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t srom_api_ptr_erase_row = 0;
	hr = target_read_u32(target, SROMAPI_ERASE_ROW, &srom_api_ptr_erase_row);
	if (hr != ERROR_OK || srom_api_ptr_erase_row == 0) {
		LOG_ERROR("Failed to read out address of sromapi");
		return hr;
	}

	struct armv7m_algorithm armv7m_info;
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* Initialize whole struct with zeroes */
	flash_context_t flash_context = {0};
	flash_context.flags = BLOCKING_OPERATION;

	struct reg_param reg_params[4];
	init_reg_param(&reg_params[0], "sp", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "lr", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, stack_area->address + stack_area->size);
	buf_set_u32(reg_params[2].value, 0, 32, flash_ctx_offs);
	buf_set_u32(reg_params[3].value, 0, 32, bkpt_offs | 1u);

	target_write_u32(target, bkpt_offs, DUAL_BKPT_INSTR);

	progress_init(last - first + 1, ERASING);

	for (unsigned sector_idx = first; sector_idx <= last; ++sector_idx) {
		uint32_t flash_addr = bank->base + bank->sectors[sector_idx].offset;

		buf_set_u32(reg_params[1].value, 0, 32, flash_addr);

		hr = target_write_buffer(target, flash_ctx_offs, sizeof(flash_context_t), (const uint8_t *)&flash_context);
		if (hr != ERROR_OK)
			break;

		/* Start the algorithm in the background */
		hr = target_run_algorithm(target, 0, NULL, ARRAY_SIZE(reg_params), reg_params, srom_api_ptr_erase_row, 0, SROMAPI_TIMEOUT_MS, &armv7m_info);
		if (hr != ERROR_OK)
			break;

		keep_alive();
		progress_sofar(sector_idx - first + 1);
	}

	progress_done(hr);

	/* Free resources  */
	target_free_working_area(target, stack_area);
	for (unsigned i = 0; i < ARRAY_SIZE(reg_params); ++i) {
		destroy_reg_param(&reg_params[i]);
	}

	return hr;
}
#endif

#if(USE_ASYNC_ALGOS)
int psc3_flash_program(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	static const uint8_t program_algo[] = {
		#include "../../../contrib/loaders/flash/psc3/psc3_write.inc"
	};

	struct target *target = bank->target;
	struct working_area *wa_algorithm;
	struct working_area *wa_stack;
	struct working_area *wa_buffer;

	if (strstr(bank->name, "super") != NULL) {
		if (!psc3_allow_write_sflash(target)) {
			return ERROR_FAIL;
		}
	}

	/* Allocate buffer for the algorithm */
	int hr = target_alloc_working_area(target, sizeof(program_algo), &wa_algorithm);
	if (hr != ERROR_OK)
		return hr;

	/* Write the algorithm code */
	hr = target_write_buffer(target, wa_algorithm->address, sizeof(program_algo), program_algo);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Allocate buffer for the stack */
	hr = target_alloc_working_area(target, SROMAPI_STACK_SIZE, &wa_stack);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Try to allocate as large RAM Buffer as possible */
	const uint32_t wa_avail = target_get_working_area_avail(target);
	uint32_t num_rows = (wa_avail - 8) / FLASH_ROW_SIZE;
	if (num_rows <= 4) {
		LOG_WARNING("Failed to allocate Circular Buffer");
		hr = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto err_free_wa_stack;
	}

	hr = target_alloc_working_area(target, num_rows * FLASH_ROW_SIZE + 8, &wa_buffer);
	assert(hr == ERROR_OK);

	LOG_DEBUG("Allocated buffer for %d pages (%d bytes)", num_rows, num_rows * FLASH_ROW_SIZE);

	struct armv7m_algorithm armv7m_algo;
	armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_algo.core_mode = ARM_MODE_THREAD;

	struct reg_param reg_params[5];
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, wa_buffer->address);
	buf_set_u32(reg_params[1].value, 0, 32, wa_buffer->address + wa_buffer->size);
	buf_set_u32(reg_params[2].value, 0, 32, bank->base + offset);
	buf_set_u32(reg_params[3].value, 0, 32, count / FLASH_ROW_SIZE);
	buf_set_u32(reg_params[4].value, 0, 32, wa_stack->address + wa_stack->size);

	hr = target_run_flash_async_algorithm(target, buffer, count / FLASH_ROW_SIZE,
			FLASH_ROW_SIZE, 0, NULL, ARRAY_SIZE(reg_params), reg_params,
			wa_buffer->address, wa_buffer->size,
			wa_algorithm->address, 0, &armv7m_algo);

	if (hr != ERROR_OK) {
		uint32_t srom_result = buf_get_u32(reg_params[0].value, 0, 32);
		print_flash_error("program", srom_result);
	}

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	target_free_working_area(target, wa_buffer);

err_free_wa_stack:
	target_free_working_area(target, wa_stack);

err_free_wa_algo:
	target_free_working_area(target, wa_algorithm);

	return hr;
}
#else

int psc3_flash_program(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	/* Allocate Working Area for Stack and Flash algorithm */
	struct working_area *stack_area;
	int hr = target_alloc_working_area(target, sizeof(flash_context_t) + 4 + FLASH_ROW_SIZE + SROMAPI_STACK_SIZE, &stack_area);
	if (hr != ERROR_OK) {
		LOG_ERROR("Failed to allocate WA (op program)");
		return hr;
	}

	if (strstr(bank->name, "super") != NULL) {
		if (!psc3_allow_write_sflash(bank->target)) {
			return ERROR_FAIL;
		}
	}

	target_addr_t data_wa_offs = stack_area->address;
	target_addr_t flash_ctx_offs = data_wa_offs + FLASH_ROW_SIZE;
	target_addr_t bkpt_offs = flash_ctx_offs + sizeof(flash_context_t);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t srom_api_ptr_progr_row = 0;
	hr = target_read_u32(target, SROMAPI_PROGRAM_ROW, &srom_api_ptr_progr_row);
	if (hr != ERROR_OK || srom_api_ptr_progr_row == 0) {
		LOG_ERROR("Failed to read out address of sromapi");
		return hr;
	}

	struct armv7m_algorithm armv7m_info;
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	/* Initialize whole struct with zeroes */
	flash_context_t flash_context = {0};
	flash_context.flags = BLOCKING_OPERATION;

	struct reg_param reg_params[5];
	init_reg_param(&reg_params[0], "sp", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "lr", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, stack_area->address + stack_area->size);
	buf_set_u32(reg_params[2].value, 0, 32, data_wa_offs);
	buf_set_u32(reg_params[3].value, 0, 32, flash_ctx_offs);
	buf_set_u32(reg_params[4].value, 0, 32, bkpt_offs | 1u);

	target_write_u32(target, bkpt_offs, DUAL_BKPT_INSTR);

	/* Start the algorithm in the background */
	progress_init(count, PROGRAMMING);

	uint32_t flash_address = bank->base + offset;
	while (count > 0) {
		buf_set_u32(reg_params[1].value, 0, 32, flash_address);

		hr = target_write_buffer(target, data_wa_offs, FLASH_ROW_SIZE, buffer);
		if (hr != ERROR_OK) {
			LOG_ERROR("Failed to write data to RAM");
			break;
		}

		hr = target_write_buffer(target, flash_ctx_offs, sizeof(flash_context_t), (const uint8_t *)&flash_context);
		if (hr != ERROR_OK) {
			LOG_ERROR("Failed to write flash context to RAM");
			break;
		}

		/* Write registers and launch SROM API*/
		hr = target_run_algorithm(target, 0, NULL, ARRAY_SIZE(reg_params), reg_params, srom_api_ptr_progr_row, 0, SROMAPI_TIMEOUT_MS, &armv7m_info);
		if (hr != ERROR_OK) {
			LOG_ERROR("Failed to execute algorithm");
			break;
		}

		count -= FLASH_ROW_SIZE;
		buffer += FLASH_ROW_SIZE;
		flash_address += FLASH_ROW_SIZE;

		keep_alive();
		progress_left(count);
	}

	progress_done(hr);

	/* This is a freaking workaround to purge read buffers in the chip */
	uint8_t cache_buf[32];
	target_read_buffer(target, bank->base + offset, 32, cache_buf);

	/* Free resources  */
	target_free_working_area(target, stack_area);
	for (unsigned i = 0; i < ARRAY_SIZE(reg_params); ++i) {
		destroy_reg_param(&reg_params[i]);
	}

	return hr;
}
#endif

int psc3_flash_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	(void)bank; (void)set; (void)first; (void)last;

	LOG_DEBUG("psc3_flash_protect()\r\n");

	return ERROR_OK;
}

int psc3_flash_protect_check(struct flash_bank *bank)
{
	LOG_DEBUG("psc3_flash_protect_check()\r\n");

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = false;

	return ERROR_OK;
}

static int psc3_get_lifecycle(struct target *target, uint32_t *lifecycle)
{
	uint32_t lcs;
	int hr = target_read_u32(target, 0x52610180, &lcs);

	if (hr != ERROR_OK) {
		LOG_ERROR("Error reading lifecycle, abort");
		return ERROR_FAIL;
	}

	lcs &= 0xFFFF;

	*lifecycle = lcs;

	return ERROR_OK;
}

COMMAND_HANDLER(psc3_handle_sflash_access)
{
	uint32_t restrictions;
	uint32_t lifecycle;
	int hr;
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], restrictions);
	if(restrictions > 1) {
		LOG_ERROR("SFlash restriction level should be in range 0...1");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (restrictions == 0) {
		g_sflash_restrictions = restrictions;
		LOG_INFO("SFlash access set to prohibited");
		return ERROR_OK;
	}

	struct flash_bank *p;
	if (get_flash_bank_by_num(0, &p) != ERROR_OK) {
		return ERROR_FAIL;
	}

	hr = psc3_get_lifecycle(p->target, &lifecycle);
	if (hr != ERROR_OK) {
		LOG_ERROR("Error reading lifecycle, abort");
		return ERROR_FAIL;
	}

	if ((lifecycle == VIRGIN_LCS || lifecycle == SORT_LCS) &&  (restrictions == 1)) {
		LOG_INFO("SFlash access enabled");
		g_sflash_restrictions = restrictions;
	} else if (restrictions == 1) {
		LOG_INFO("SFlash access in current LCS constantly prohibited");
		return ERROR_OK;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Creates 'second' flash bank out from original bank in dual-bank mode. New bank has a half
 * size of original bank and offset DUAL_BANK_OFFSET
 *
 * @param main_bank - original bank
 * @param probed_by_read - set true, if bank probing done by means of reading instead of fetching
 * from mpn list
 * @return pointer to bank or NULL
 *************************************************************************************************/
static struct flash_bank * psc3_alloc_bank(struct flash_bank *main_bank, bool probed_by_read)
{
	char* pBank_0;
	struct flash_bank *dual_bank = calloc(1, sizeof(struct flash_bank));
	if (!dual_bank) {
		return NULL;
	}

	char *bank_name = strdup(main_bank->name);
	if (!bank_name) {
		free(dual_bank);
		return NULL;
	}

	pBank_0 = strchr(bank_name, '0');
	if (!pBank_0) {
		LOG_ERROR("Fail in bank namings");
		free(dual_bank);
		free(bank_name);
		return NULL;
	}

	*pBank_0 = '1';

	struct psc3_info *info = calloc(1, sizeof(struct psc3_info));
	if (!info) {
		free(dual_bank);
		free(bank_name);
		return NULL;
	}

	info->row_size = FLASH_ROW_SIZE;
	info->probed = false;

	if (probed_by_read)
		/* Main bank already contains correct size*/
		dual_bank->size = main_bank->size;
	else
		/* Flash size fetched from cympn list and must be divided in dual bank mode */
		dual_bank->size = main_bank->size * 0.5;

	dual_bank->target = main_bank->target;
	dual_bank->name = bank_name;
	dual_bank->base = main_bank->base + DUAL_BANK_OFFSET;
	dual_bank->write_start_alignment = dual_bank->write_end_alignment = FLASH_ROW_SIZE;
	dual_bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;
	dual_bank->default_padded_value = dual_bank->erased_value = 0x00;
	dual_bank->is_memory_mapped = true;
	dual_bank->num_sectors = dual_bank->size / FLASH_ROW_SIZE;


	dual_bank->sectors = alloc_block_array(0, info->row_size, dual_bank->num_sectors);
	if (!dual_bank->sectors) {
		return NULL;
	}
	info->dual_bank_mode = true;
	info->probed = true;

	dual_bank->driver = main_bank->driver;
	dual_bank->driver_priv = info;

	flash_bank_add(dual_bank);

	return dual_bank;
}

/** ***********************************************************************************************
 * @brief Creates virtual flash bank out from master bank in dual-bank mode
 * @param master_bank - bank to link to
 * @param secure - if secure/non-secure bank must be created
 * @return pointer to virtual bank or NULL
 *************************************************************************************************/
static struct flash_bank * psc3_alloc_bank_virtual(struct flash_bank *master_bank, bool secure)
{
	struct flash_bank *virtual_bank = calloc(1, sizeof(struct flash_bank));
	if (!virtual_bank) {
		return NULL;
	}

	char *bank_name = calloc(strlen(VIRTUAL_BANK_NAME_BASE) + 3, 1);
	if (!bank_name) {
		free(virtual_bank);
		return NULL;
	}

	strcpy(bank_name, VIRTUAL_BANK_NAME_BASE);
	secure ? strcat(bank_name, "s") : strcat(bank_name, "ns");

	virtual_bank->target = master_bank->target;
	virtual_bank->name = bank_name;
	virtual_bank->base = master_bank->base - VIRTUAL_BANK_OFFSET;
	virtual_bank->write_start_alignment = virtual_bank->write_end_alignment = FLASH_ROW_SIZE;
	virtual_bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;
	virtual_bank->default_padded_value = virtual_bank->erased_value = 0x00;
	virtual_bank->size = master_bank->size;

	const struct flash_driver *driver = flash_driver_find_by_name("virtual");
	if (!driver) {
		free(virtual_bank);
		free(bank_name);
		return NULL;
	}
	virtual_bank->driver = driver;

	virtual_bank->driver_priv = strdup(master_bank->name);

	flash_bank_add(virtual_bank);

	return virtual_bank;
}

/** ***********************************************************************************************
 * @brief Updates original bank in dual bank mode
 *
 * @param main_bank - bank
 * @param probed_by_read - set true, if bank probing done by means of reading instead of fetching
 * from mpn list
 *************************************************************************************************/
static void update_main_bank(struct flash_bank *main_bank, bool probed_by_read)
{
	struct psc3_info *info = (struct psc3_info *)main_bank->driver_priv;

	if (!info)
		return;

	info->probed = false;

	if (!probed_by_read) {
		main_bank->size *= 0.5;
		if (main_bank->sectors){
			free (main_bank->sectors);
		}
		main_bank->num_sectors = main_bank->size / FLASH_ROW_SIZE;
		main_bank->sectors = alloc_block_array(0, FLASH_ROW_SIZE, main_bank->num_sectors);
		if (!main_bank->sectors) {
			return;
		}
	}
	info->dual_bank_mode = true;
	info->probed = true;
}

/** ***********************************************************************************************
 * @brief Updates virtual bank out from master bank in dual bank mode
 *
 * @param bank - virtual bank
 * @param master_bank - master bank
 *************************************************************************************************/
static void update_virtual_bank(struct flash_bank *bank, struct flash_bank *master_bank)
{
	if (!master_bank)
		return;

	bank->size = master_bank->size;
	bank->chip_width = master_bank->chip_width;
	bank->bus_width = master_bank->bus_width;
	bank->erased_value = master_bank->erased_value;
	bank->default_padded_value = master_bank->default_padded_value;
	bank->write_start_alignment = master_bank->write_start_alignment;
	bank->write_end_alignment = master_bank->write_end_alignment;
	bank->minimal_write_gap = master_bank->minimal_write_gap;
	bank->num_sectors = master_bank->num_sectors;
	bank->sectors = master_bank->sectors;
	bank->num_prot_blocks = master_bank->num_prot_blocks;
	bank->prot_blocks = master_bank->prot_blocks;
	bank->is_memory_mapped = master_bank->is_memory_mapped;
}

/** ***********************************************************************************************
 * @brief PSoC C3 supports dual bank mode. Handler checks if dual bank mode activated and creates
 * new banks with offset DUAL_BANK_OFFSET and a half size of the original bank.
 *
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
COMMAND_HANDLER(psc3_handle_dual_bank)
{
	struct flash_bank *bank0_s;
	struct flash_bank *bank0_ns;
	struct flash_bank *bank1_s;
	struct flash_bank *bank1_ns;
	struct flash_bank *v_bank;
	struct target *target;
	struct psc3_info *info;
	uint32_t flash_ctl;
	bool probe_method_read;
	int hr;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target = get_current_target(CMD_CTX);

	hr = get_flash_bank_by_addr(target, FLASH_BANK_BASE_SECURE, true, &bank0_s);
	if (hr != ERROR_OK)
		return ERROR_FAIL;

	info = (struct psc3_info *)bank0_s->driver_priv;
	if (!info)
		return ERROR_FAIL;

	if (info->bank_size_override)
		/* Probe using sequential flash read*/
		probe_method_read = true;
	else
		/* Probe using device mpn list*/
		probe_method_read = false;

	hr = target_read_u32(target, 0x52150000, &flash_ctl);
	if (hr != ERROR_OK)
		return ERROR_FAIL;

	if (flash_ctl & DUAL_BANK_MODE)  {
		LOG_DEBUG("Dual bank mode on");

		/* check if dual-banks were already created*/
		char* check_bank_name = strdup(bank0_s->name);
		if (!check_bank_name)
			return ERROR_FAIL;

		char* pBank_0 = strchr(check_bank_name, '0');
		if (!check_bank_name) {
			LOG_ERROR("Fail in bank namings");
			return ERROR_FAIL;
		}

		*pBank_0 = '1';

		if (get_flash_bank_by_name_noprobe(check_bank_name)) {
			/* Dual mode banks already exist*/
			LOG_DEBUG("Dual mode banks already inited");
			free(check_bank_name);
			return ERROR_OK;
		}
		free(check_bank_name);

		/* Secure flash alias*/
		bank1_s = psc3_alloc_bank(bank0_s, probe_method_read);
		if (!bank1_s)
			return ERROR_FAIL;
		update_main_bank(bank0_s, probe_method_read);

		/* Non-secure flash alias*/
		hr = get_flash_bank_by_addr(target, FLASH_BANK_BASE_NON_SECURE, true, &bank0_ns);
		if (hr != ERROR_OK || !bank0_ns)
			return ERROR_FAIL;
		bank1_ns = psc3_alloc_bank(bank0_ns, probe_method_read);
		if (!bank1_ns)
			return ERROR_FAIL;
		update_main_bank(bank0_ns, probe_method_read);

		/* manage secure virtual banks */
		get_flash_bank_by_addr(target, FLAH_BANK_BASE_VIRTUAL_SECURE, true, &v_bank);
		update_virtual_bank(v_bank, bank0_s);
		psc3_alloc_bank_virtual(bank1_s, SECURE);

		/* manage non-secure virtual banks */
		get_flash_bank_by_addr(target, FLAH_BANK_BASE_VIRTUAL_NON_SECURE, true, &v_bank);
		update_virtual_bank(v_bank, bank0_ns);
		psc3_alloc_bank_virtual(bank1_ns, NON_SECURE);
	}

	return ERROR_OK;
}

static bool psc3_allow_write_sflash(struct target *target)
{
	uint32_t lcs = VIRGIN_LCS;
	int hr;

	if (g_sflash_restrictions != 1) {
		hr = psc3_get_lifecycle(target, &lcs);
		if (hr != ERROR_OK)
			return false;

		if (lcs == VIRGIN_LCS || lcs == SORT_LCS) {
			LOG_WARNING("SFlash operations are prohibited. To enable SFlash access"
				    "use command 'psc3 sflash_restrictions 1'.");
		} else {
			LOG_ERROR("SFlash operations are prohibited in current lifecycle");
		}
		return false;
	} else {
		return true;
	}
}

/** ***********************************************************************************************
 * @brief Handler sets size for flash banks
 *
 * @param size - flash size in KiB to set
 * @param size_max - max main flash size in KiB that present in device serie
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
COMMAND_HANDLER(psc3_set_banks_size)
{
	struct psc3_info *info;
	uint32_t bank_size;
	uint32_t bank_size_max = 0;

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], bank_size);

	if (CMD_ARGC == 2)
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], bank_size_max);

	struct flash_bank *p = get_flash_bank_by_num_noprobe(0);
	if (!p)
		return ERROR_FAIL;

	for (; p; p = p->next) {
		switch (p->base)
		{
		case FLASH_BANK_BASE_SECURE:
		case FLASH_BANK_BASE_NON_SECURE:
			info = (struct psc3_info *)p->driver_priv;
			if (!info)
				return ERROR_FAIL;

			if (p->size == 0) {
				p->size = KiB(bank_size);
				info->bank_size_max = bank_size_max ? KiB(bank_size_max) : 0;
				info->probed = false;
			}
			break;

		default:
			break;
		}
	}

	return ERROR_OK;
}

static const struct command_registration psc3_exec_cmd_handlers[] = {
	{
		.name = "sflash_restrictions",
		.handler = psc3_handle_sflash_access,
		.mode = COMMAND_ANY,
		.usage = "<0|1>",
		.help = "Controls access to write/program/erase SFlash banks:\r\n"
				"0 - prohibit (default)\r\n"
				"1 - enable in Virgin/Sort LCS",
	},
	{
		.name = "handle_flash_bank_mode",
		.handler = psc3_handle_dual_bank,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Checks if target is in dual bank mode"
				"and changes original banks in a proper way",
	},
	{
		.name = "set_banks_size",
		.handler = psc3_set_banks_size,
		.mode = COMMAND_EXEC,
		.usage = "<flash_size> <flash_size_limit>",
		.help = "Sets flash size for flash driver in KiB,\r\n"
				"optionally limits max flash size (for autodetecting flash size feature)",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration psc3_cmd_handlers[] = {
	{
		.name = "psc3",
		.mode = COMMAND_ANY,
		.help = "PSoC C3 flash command group",
		.usage = "",
		.chain = psc3_exec_cmd_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver psc3_flash = {
	.name = "psc3",
	.usage = "flash bank <name> psc3 <base> <size> 0 0 <target#> [page_size = 0x200]",
	.commands = psc3_cmd_handlers,
	.flash_bank_command = psc3_flashbank_command,
	.erase = psc3_flash_erase,
	.protect = psc3_flash_protect,
	.write = psc3_flash_program,
	.read = default_flash_read,
	.probe = psc3_flash_probe,
	.auto_probe = psc3_flash_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = psc3_flash_protect_check,
	.info = NULL,
	.free_driver_priv = default_flash_free_driver_priv,
};
