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
#include "flash/progress.h"
#include "helper/time_support.h"
#include "rtos/rtos.h"
#include "target/algorithm.h"
#include "target/arm_adi_v5.h"
#include "target/breakpoints.h"
#include "target/cortex_m.h"
#include "target/image.h"
#include "target/register.h"
#include "target/target.h"

static struct working_area *g_stack_area;
static struct armv7m_algorithm g_armv7m_info;
static uint32_t g_sflash_restrictions;
static bool g_sromcall_prepare_called;

enum operation {
	PROGRAM,
	ERASE
};

/** ***********************************************************************************************
 * @brief Iterates over all flash banks, locates any bank assigned to a current target with
 * mxs40_bank_info populated and returns the pointer to the structure
 * @param target current target
 * @return pointer to mxs40_bank_info structure or NULL
 *************************************************************************************************/
static struct mxs40_bank_info *mxs40_get_bank_info_by_target(struct target *target)
{
	struct flash_bank *bank_iter;
	struct mxs40_bank_info *info = NULL;

	for (bank_iter = flash_bank_list(); bank_iter; bank_iter = bank_iter->next) {
		if (bank_iter->target != target)
			continue;

		if(strcmp(bank_iter->driver->name, "virtual") == 0) {
			struct flash_bank *master_bank = get_flash_bank_by_name_noprobe(bank_iter->driver_priv);
			info = master_bank->driver_priv;
			if (info)
				break;
		}

		info = bank_iter->driver_priv;
		if (info)
			break;
	}

	return info;
}

/** ***********************************************************************************************
 * @brief Initializes `struct timeout` structure with given timeout value
 * @param to pointer to `struct timeout` structure
 * @param timeout_ms timeout, in milliseconds
 *************************************************************************************************/
void mxs40_timeout_init(struct timeout *to, long timeout_ms)
{
	to->start_time = timeval_ms();
	to->timeout_ms = timeout_ms;
}

/** ***********************************************************************************************
 * @brief Returns true if given timeout has expired
 * @param to pointer to `struct timeout` structure
 * @return true if timeout expired
 *************************************************************************************************/
bool mxs40_timeout_expired(struct timeout *to)
{
	return (timeval_ms() - to->start_time) > to->timeout_ms;
}

/** ***********************************************************************************************
 * @brief Translates Protection status to string
 * @param protection protection value
 * @return pointer to const string describintg protection status
 *************************************************************************************************/
static const char *mxs40_protection_to_str(uint8_t protection)
{
	static const char *prot_states[] = {"UNKNOWN", "VIRGIN", "NORMAL", "SECURE", "DEAD"};
	if (protection > 4)
		protection = 0;
	return prot_states[protection];
}

/** ***********************************************************************************************
 * @brief Waits for expected IPC lock status. MXS40 patform uses IPC structures for inter-core
 * communication. Same IPCs are used to invoke SROM API. IPC structure must be locked prior to
 * invoking any SROM API. This ensures nothing else in the system will use same IPC thus corrupting
 * our data. Locking is performed by ipc_acquire(), this function ensures that IPC is actually
 * in expected state
 *
 * @param bank current flash bank
 * @param lock_expected expected lock status
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int mxs40_ipc_poll_lock_stat(struct flash_bank *bank, bool lock_expected)
{
	uint32_t reg_val;
	struct timeout to;
	struct target *target = bank->target;
	struct mxs40_bank_info *info = bank->driver_priv;
	mxs40_timeout_init(&to, IPC_TIMEOUT_MS);

	while (!mxs40_timeout_expired(&to)) {
		/* Process any server requests */
		keep_alive();

		/* Read IPC Lock status */
		int hr = target_read_u32(target, info->regs->ipc_lock_stat, &reg_val);
		if (hr != ERROR_OK) {
			LOG_ERROR("Unable to read IPC Lock Status register");
			return hr;
		}

		bool is_locked = (reg_val & IPC_LOCK_ACQUIRED_MSK) != 0;

		if (lock_expected == is_locked)
			return ERROR_OK;
	}

	LOG_ERROR("Timeout polling IPC Lock Status");
	return ERROR_TARGET_TIMEOUT;
}

/** ***********************************************************************************************
 * @brief Acquires IPC structure. MXS40 patform uses IPC structures for inter-core communication.
 * Same IPCs are used to invoke SROM API. IPC structure must be locked prior to invoking any SROM API.
 * This ensures nothing else in the system will use same IPC thus corrupting our data.
 * This function locks the IPC.
 *
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int mxs40_ipc_acquire(struct flash_bank *bank)
{
	int hr = ERROR_OK;
	bool is_acquired = false;
	uint32_t reg_val = 0;
	struct timeout to;
	struct target *target = bank->target;
	struct mxs40_bank_info *info = bank->driver_priv;

	mxs40_timeout_init(&to, IPC_TIMEOUT_MS);
	while (!mxs40_timeout_expired(&to)) {
		keep_alive();

		/* Workaround for double-buffered PPU */
		if (info->regs->ppu_flush) {
			enum log_levels level = change_debug_level(LOG_LVL_USER);
			hr = target_read_u32(target, info->regs->ppu_flush, &reg_val);
			change_debug_level(level);
			info->ppu_read_protected = (hr != ERROR_OK);
		}

		/* Acquire the IPC structure */
		hr = target_write_u32(target, info->regs->ipc_acquire, IPC_ACQUIRE_SUCCESS_MSK);
		if (hr != ERROR_OK) {
			LOG_ERROR("Unable to write to IPC Acquire register%s",
					  info->ppu_read_protected ? " (PPU read-protected)" : "");
			return hr;
		}

		/* Check if data is written on first step */
		hr = target_read_u32(target, info->regs->ipc_acquire, &reg_val);
		if (hr != ERROR_OK) {
			LOG_ERROR("Unable to read IPC Acquire register%s",
					  info->ppu_read_protected ? " (PPU read-protected)" : "");
			return hr;
		}

		is_acquired = (reg_val & IPC_ACQUIRE_SUCCESS_MSK) != 0;
		if (is_acquired) {
			/* If IPC structure is acquired, the lock status should be set */
			hr = mxs40_ipc_poll_lock_stat(bank, true);
			break;
		}
	}

	if (!is_acquired)
		LOG_ERROR("Timeout acquiring IPC structure");

	return hr;
}

/** ***********************************************************************************************
 * @brief Helper to detect size of the flash. Reads address space with predefined interval and
 * calculates size of the memory area based on read failures
 *
 * @param target current target
 * @param start_addr starting address of the area
 * @param max_size maximal size of the area
 * @param step interval size
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
uint32_t mxs40_probe_mem_area(struct target *target, target_addr_t start_addr, uint32_t step,
							  uint32_t max_precision)
{
	assert(step && !(step & (step - 1)));
	assert(start_addr % step == 0);
	assert(max_precision && !(max_precision & (max_precision - 1)));
	assert(max_precision >= 4);
	assert(step > max_precision);

	int hr;
	uint32_t offset = 0;
	uint32_t dummy;
	int steps = 0;

	enum log_levels old_lvl = change_debug_level(LOG_LVL_USER);
	while(true) {
		steps++;
		hr = target_read_u32(target, start_addr + offset, &dummy);
		if(hr != ERROR_OK)
			break;

		offset += step;
	}

	while(step > max_precision) {
		steps++;
		step = step >> 1;
		hr = target_read_u32(target, start_addr + offset, &dummy);
		offset = (hr != ERROR_OK) ? offset - step : offset + step;
	}
	change_debug_level(old_lvl);

	const uint32_t area_size = offset + step;
	LOG_INFO("Probed Flash Bank @" TARGET_ADDR_FMT ", size %u.%u KiB (in %d steps)", start_addr,
			 area_size >> 10, area_size % 1024, steps);

	return area_size;
}

/** ***********************************************************************************************
 * @brief Locates CM0 core of this chip (if present)
 * @param this_target current target, name should end with 'sysap'
 * @return pointer to CM0 core (of NULL)
 *************************************************************************************************/
static struct target *mxs40_find_core_by_suffix(struct target *target, const char *suffix)
{
	const size_t this_len = strlen(target->cmd_name);
	size_t last_dot_pos;
	for(last_dot_pos = this_len; last_dot_pos; last_dot_pos--) {
		if(target->cmd_name[last_dot_pos] == '.')
			break;
	}

	assert(last_dot_pos != 0);

	last_dot_pos++;
	char *new_name = calloc(1, last_dot_pos + strlen(suffix) + 1);
	memcpy(new_name, target->cmd_name, last_dot_pos);
	new_name = strcat(new_name, suffix);

	struct target *result = NULL;
	for (result = all_targets; result; result = result->next) {
		if (target_name(result) && strcmp(new_name, target_name(result)) == 0)
			break;
	}

	free(new_name);
	return result;
}

/** ***********************************************************************************************
 * @brief Starts pseudo flash algorithm and leaves it running. Function allocates working area for
 * algorithm code and CPU stack, adjusts stack pointer, uploads and starts the algorithm.
 * Algorithm (a basic infinite loop) runs asynchronously while driver performs Flash operations.
 *
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_sromalgo_prepare(struct flash_bank *bank)
{
	struct target *target = bank->target;

	/* Special core_id for fake SysAP */
	if(target->coreid == 0xFF) {
		struct target *cm0_target = mxs40_find_core_by_suffix(target, "cm0");
		if(!cm0_target)
			return ERROR_OK;

		target = cm0_target;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int hr;

	struct mxs40_bank_info *info = bank->driver_priv;
	if(info->prepare_function) {
		hr = info->prepare_function(bank);
		if (hr != ERROR_OK)
			return hr;
	}

	/* Check if IPC_INTR_MASK contains valid value */
	uint32_t ipc_intr;
	hr = target_read_u32(target, info->regs->ipc_intr, &ipc_intr);
	if (hr != ERROR_OK)
		return hr;

	/* Enable notification interrupt of IPC_INTR_STRUCT for IPC_STRUCT2 */
	if(!(ipc_intr & info->regs->ipc_intr_msk)) {
		hr = target_write_u32(target, info->regs->ipc_intr, ipc_intr | info->regs->ipc_intr_msk);
		if (hr != ERROR_OK)
			return hr;
	}

	/* Initialize Vector Table Offset register (in case FW modified it) */
	hr = target_write_u32(target, NVIC_VTOR, 0x00000000);
	if (hr != ERROR_OK)
		return hr;

	/* Clear pending interrupts. This resolves the issue that IRQ0/IRQ1 can not be
	 * executed when core is HardFault os similar higher-priority handler */
	const struct armv7m_common *cm = target_to_armv7m(target);
	hr = mem_ap_write_atomic_u32(cm->debug_ap, NVIC_AIRCR, AIRCR_VECTKEY | AIRCR_VECTCLRACTIVE);
	if (hr != ERROR_OK)
		return hr;

	/* Allocate Working Area for Stack and Flash algorithm */
	hr = target_alloc_working_area(target, RAM_STACK_WA_SIZE, &g_stack_area);
	if (hr != ERROR_OK)
		return hr;

	g_armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	g_armv7m_info.core_mode = ARM_MODE_THREAD;

	struct reg_param reg_params;
	init_reg_param(&reg_params, "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params.value, 0, 32, g_stack_area->address + g_stack_area->size);

	/* Write 'cpsie	i' + basic infinite loop algorithm to target RAM */
	hr = target_write_u32(target, g_stack_area->address, 0xE7FEB662);
	if (hr != ERROR_OK)
		goto destroy_rp_free_wa;

	hr = target_start_algorithm(target, 0, NULL, 1, &reg_params, g_stack_area->address,
			0, &g_armv7m_info);
	if (hr != ERROR_OK)
		goto destroy_rp_free_wa;

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
 * @brief Stops running flash algorithm and releases associated resources.
 * This function is also used for cleanup in case of errors so g_stack_area may be NULL.
 * These cases have to be handled gracefully.
 *
 * @param target current target
 *************************************************************************************************/
void mxs40_sromalgo_release(struct target *target)
{
	if (g_stack_area) {
		/* Special core_id for fake SysAP */
		if(target->coreid == 0xFF) {
			struct target *cm0_target = mxs40_find_core_by_suffix(target, "cm0");
			if(!cm0_target)
				return;

			target = cm0_target;
		}

		/* Stop flash algorithm if it is running */
		if (target->running_alg) {
			int hr = target_halt(target);
			if (hr != ERROR_OK)
				goto exit_free_wa;

			hr = target_wait_algorithm(target, 0, NULL, 0, NULL, 0,
					IPC_TIMEOUT_MS, &g_armv7m_info);
			if (hr != ERROR_OK)
				goto exit_free_wa;
		}

exit_free_wa:
		/* Free Stack/Flash algorithm working area */
		target_free_working_area(target, g_stack_area);
		g_stack_area = NULL;
	}
}

/** ***********************************************************************************************
 * @brief Invokes SROM API functions which are responsible for Flash operations
 *
 * @param bank current flash bank
 * @param req_and_params requect id of the function to invoke
 * @param working_area address of memory buffer in target's memory space for SROM API parameters
 * @param check_errors true if error check and reporting should be performed
 * @param data_out pointer to variable which will be populated with execution status
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_call_sromapi_inner(struct flash_bank *bank, uint32_t req_and_params, uint32_t working_area,
    bool check_errors, uint32_t *data_out)
{
    LOG_DEBUG("Executing SROM API #0x%08X", req_and_params);

    struct target *target = bank->target;

	if(!g_stack_area && target->coreid != 0xFF) {
		LOG_ERROR("SROM Call: target is not prepared for srom calls");
		return ERROR_FAIL;
	}

	int hr;
	struct mxs40_bank_info *info = bank->driver_priv;
	bool is_data_in_ram = (req_and_params & SROMAPI_DATA_LOCATION_MSK) == 0;

	hr = mxs40_ipc_acquire(bank);
	if (hr != ERROR_OK)
		return hr;

	if (is_data_in_ram)
		hr = target_write_u32(target, info->regs->ipc_data, working_area);
	else
		hr = target_write_u32(target, info->regs->ipc_data, req_and_params);

	if (hr != ERROR_OK)
		return hr;

	hr = target_write_u32(target, info->regs->ipc_notify, 1);
	if (hr != ERROR_OK)
		return hr;

	/* Poll lock status */
	hr = mxs40_ipc_poll_lock_stat(bank, false);
	if (hr != ERROR_OK)
		return hr;

	/* Poll Data byte */
	if (is_data_in_ram)
		hr = target_read_u32(target, working_area, data_out);
	else
		hr = target_read_u32(target, info->regs->ipc_data, data_out);

	if (hr != ERROR_OK) {
		LOG_ERROR("Error reading SROM API Status location");
		return hr;
	}

	bool is_error = (*data_out & SROMAPI_STATUS_MSK) != SROMAPI_STAT_SUCCESS;
	if (check_errors && is_error) {
		LOG_ERROR("SROM API execution failed. Status: 0x%08X", (uint32_t)*data_out);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Invokes SROM API functions which are responsible for Flash operations
 *
 * @param bank current flash bank
 * @param req_and_params requect id of the function to invoke
 * @param working_area address of memory buffer in target's memory space for SROM API parameters
 * @param data_out pointer to variable which will be populated with execution status
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_call_sromapi(struct flash_bank *bank, uint32_t req_and_params,
    uint32_t working_area, uint32_t *data_out)
{
    return mxs40_call_sromapi_inner(bank, req_and_params, working_area, true, data_out);
}

/** ***********************************************************************************************
 * @brief Checks if given flash bank base address is one in the addr_array array
 * @param bank current flash bank
 * @param match pointer to the array of bank_match structures
 * @return true if flash bank matches
 *************************************************************************************************/
bool mxs40_flash_bank_matches(struct flash_bank *bank, const struct bank_match *match)
{
	uint32_t idx = 0;
	while (match[idx].address_mask) {
		if ((bank->base & match[idx].address_mask) == match[idx].address_value)
			return true;
		idx++;
	}

	return false;
}

/** ***********************************************************************************************
 * @brief Checks if given flash row belongs to Safe Supervisory Flash region
 * @param addr row start address to check
 * @param op type of operation to check safety for
 * @return true if flash bank belongs to Safe Supervisory Flash region
 *************************************************************************************************/
static bool mxs40_is_safe_sflash_page(struct mxs40_bank_info *info, uint32_t addr, enum operation op)
{
	assert(addr % 512 == 0);

	/* At restriction level 3 all writes are allowed except during erase */
	if (op == PROGRAM && g_sflash_restrictions == 3)
		return true;

	for (size_t i = 0; i < 4; i++) {
		target_addr_t region_start = info->sflash_regions[i].addr;
		uint32_t region_size = info->sflash_regions[i].size;

		if (addr >= region_start && addr < region_start + region_size) {
			if(op == ERASE) {
				return (info->sflash_regions[i].flags & (1u << (g_sflash_restrictions + 0)));
			} else { /* op == PROGRAM */
				return (info->sflash_regions[i].flags & (1u << (g_sflash_restrictions + 4)));
			}
		}
	}

	return false;
}

/** ***********************************************************************************************
 * @brief Retrieves SiliconID and Protection status of the target device
 * @param bank current flash bank
 * @param si_id pointer to variable, will be populated with SiliconID of the following format:
 *  [31:24]       | [23:16]       | [15:12]       | [11:8]        | [7:0]
 *  Silicon Id Hi | Silicon ID Lo | Major Rev. Id | Minor Rev. Id | Family Id Lo
 * @param protection pointer to variable, will be populated with protection status
 * @param lifecycle pointer to variable, will be populated with lifecycle state
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_get_silicon_id(struct flash_bank *bank, uint32_t *si_id, uint8_t *protection, uint8_t *lifecycle)
{
	int hr;
	uint32_t family_rev, siid_prot;
	struct target *target = bank->target;

	hr = mxs40_sromalgo_prepare(bank);
	if (hr != ERROR_OK)
		goto exit;

	/* SiliconID SROM API returns:
			   | [31:28]           | [23:20]           | [19:16]           | [15:8]       | [7:0]         |
	   ----------------------------------------------------------------------------------------------------
	   Type 0: | Status Code = 0xA | Major Revision Id | Minor Revision Id | Family Id Hi | Family Id Lo  |
	   Type 1: | Status Code = 0xA | Not used          | Protection state  | Silicon Id Hi| Silicon ID Lo |
	   Type 2: | Status Code = 0xA | Not used          | Not used          | SROM FW major| SROM FW minor | */

	/* Type 0: Get Family ID and Revision ID */
	hr = mxs40_call_sromapi(bank, SROMAPI_SIID_REQ_FAMILY_REVISION, 0, &family_rev);
	if (hr != ERROR_OK)
		goto exit;

	/* Type 1: Get Silicon ID, Protection state and Life Cycle stage */
	hr = mxs40_call_sromapi(bank, SROMAPI_SIID_REQ_SIID_PROTECTION, 0, &siid_prot);
	if (hr != ERROR_OK)
		goto exit;

	*si_id  = (siid_prot & 0x0000FFFF) << 16; /* si_id[31:24] = Silicon Id Hi, si_id[23:16] = Silicon ID Lo */
	*si_id |= (family_rev & 0x00FF0000) >> 8; /* si_id[15:12] = Major Rev. Id, si_id[11:8]  = Minor Rev. Id */
	*si_id |= (family_rev & 0x000000FF) >> 0; /* si_id[7:0]   = Family Id Lo */

	*protection = (siid_prot & 0x000F0000) >> 0x10;
	*lifecycle = (siid_prot & 0x00F00000) >> 0x14;

exit:
	mxs40_sromalgo_release(target);
	return hr;
}

/** ***********************************************************************************************
 * @brief Identifies Device Family/Die from Silicon ID using following rules:
 *        Device Family    | Family ID | Si ID Range
 *        ------------------------------------------
 *        PSoC6A-BLE2      | 0x100     | E200-E2FF
 *        TraveoII B-E-1M  | 0x101     | E300-E3FF
 *        PSoC6A-2M        | 0x102     | E400-E4FF
 *        TraveoII B-H-8M  | 0x103     | E500-E5FF
 *        TraveoII B-E-2M  | 0x104     | E600-E6FF
 *        PSoC6A-512K      | 0x105     | E700-E7FF
 *        TraveoII Cluster | 0x106     | E800-E8FF
 *        TraveoII B-H-16M | 0x114     | ED00-ED3F
 * @param silicon_id Silicon ID of the following format:
 *  [31:24]       | [23:16]       | [15:12]       | [11:8]        | [7:0]
 *  Silicon Id Hi | Silicon ID Lo | Major Rev. Id | Minor Rev. Id | Family Id Lo
 * @param die pointer to variable, will be populated with die enum identificator
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_identify_die_from_silicon_id(uint32_t silicon_id, enum mxs40_die *die)
{
	uint8_t family_id_lo =  (silicon_id & 0x000000FF);
	uint8_t silicon_id_hi = (silicon_id & 0xFF000000) >> 24;

	if ((family_id_lo == 0x00) && (silicon_id_hi == 0xE2))
		*die = die_psoc6_ble2;
	else if ((family_id_lo == 0x01) && (silicon_id_hi == 0xE3))
		*die = die_traveo2_1m;
	else if ((family_id_lo == 0x02) && (silicon_id_hi == 0xE4))
		*die = die_psoc6_2m;
	else if ((family_id_lo == 0x03) && (silicon_id_hi == 0xE5))
		*die = die_traveo2_8m;
	else if ((family_id_lo == 0x04) && (silicon_id_hi == 0xE6))
		*die = die_traveo2_2m;
	else if ((family_id_lo == 0x05) && (silicon_id_hi == 0xE7))
		*die = die_psoc6_512k;
	else if ((family_id_lo == 0x06) && (silicon_id_hi == 0xE8))
		*die = die_traveo2_cl;
	else if ((family_id_lo == 0x14) && (silicon_id_hi == 0xED))
		*die = die_traveo2_16m;
	else
		*die = die_unknown;

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Translates Protection status to openocd-friendly boolean value
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_protect_check(struct flash_bank *bank)
{
	int is_protected;
	uint32_t silicon_id;
	uint8_t protection;
	uint8_t lifecycle;

	int hr = mxs40_get_silicon_id(bank, &silicon_id, &protection, &lifecycle);
	if (hr != ERROR_OK)
		return hr;

	switch (protection) {
		case MXS40_CHIP_PROT_VIRGIN:
		case MXS40_CHIP_PROT_NORMAL:
			is_protected = 0;
			break;

		case MXS40_CHIP_PROT_UNKNOWN:
		case MXS40_CHIP_PROT_SECURE:
		case MXS40_CHIP_PROT_DEAD:
		default:
			is_protected = 1;
			break;
	}

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = is_protected;

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Dummy function, device does not support flash bank protection
 * @return ERROR_OK always
 *************************************************************************************************/
int mxs40_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	(void)bank; (void)set; (void)first; (void)last;

	LOG_WARNING("Device does not support flash bank protection");
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief MXS40 patform_get_info Displays human-readable information about acquired device
 * @param bank current flash bank
 * @param buf pointer to buffer for human-readable text
 * @param buf_size size of the buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct mxs40_bank_info *info = bank->driver_priv;

	if (info->is_probed == false)
		return ERROR_FAIL;

	uint32_t silicon_id;
	uint8_t protection;
	uint8_t lifecycle;

	int hr = mxs40_get_silicon_id(bank, &silicon_id, &protection, &lifecycle);
	if (hr != ERROR_OK)
		return hr;

	command_print_sameline(cmd, "Silicon ID: 0x%08X\nProtection: %s",
		silicon_id, mxs40_protection_to_str(protection));

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Probes target device only if it hasn't been probed yet
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_auto_probe(struct flash_bank *bank)
{
	struct mxs40_bank_info *info = bank->driver_priv;
	return info->is_probed ? ERROR_OK : bank->driver->probe(bank);
}

/** ***********************************************************************************************
 * @brief Erases Supervisory Flash by programming it with zeros
 *
 * @param bank current flash bank
 * @param first first sector to erase
 * @param last last sector to erase
 * @return ERROR_OK, all errors are ignored (but reported)
 *************************************************************************************************/
int mxs40_erase_sflash(struct flash_bank *bank, int first, int last)
{
	uint32_t offset = bank->sectors[first].offset;
	uint32_t count = bank->sectors[last].offset + bank->sectors[last].size - offset;
	uint8_t *buffer = calloc(1, count);
	uint8_t *const_buffer = buffer;
	bool erase_skipped = false;
	struct target *target = bank->target;
	struct mxs40_bank_info *info = bank->driver_priv;
	int hr;

	const bool is_sflash = mxs40_flash_bank_matches(bank, info->regs->mem_base_sflash);
	assert(is_sflash == true);

	memset(buffer, bank->erased_value, count);
	hr = mxs40_sromalgo_prepare(bank);
	if (hr != ERROR_OK)
		goto exit;

	progress_init(count / info->page_size, ERASING);
	for (size_t i = 0; i < count / info->page_size; i++) {
		const uint32_t row_addr = bank->base + offset + i * info->page_size;
		if (mxs40_is_safe_sflash_page(info, row_addr, ERASE)) {
			hr = mxs40_program_row(bank, row_addr, buffer, is_sflash);
			if (hr != ERROR_OK)
				LOG_ERROR("Failed to program Flash at address 0x%08X", row_addr);
		} else erase_skipped = true;

		progress_sofar(i + 1);
		buffer += info->page_size;
	}
	hr = ERROR_OK;

exit:
	free(const_buffer);
	mxs40_sromalgo_release(target);
	progress_done(hr);

	if(erase_skipped)
		LOG_WARNING("Some SFlash rows were skipped during erase, see 'sflash_restrictions' command");

	return hr;
}

/** ***********************************************************************************************
 * @brief Erases single Row or Sector on target device
 * @param bank current flash bank
 * @param addr starting address of the flash row
 * @param erase_sector if true will erase sector, erases row otherwise
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_erase_row(struct flash_bank *bank, uint32_t addr, bool erase_sector)
{
	const bool is_flash_ctrl1 = (bank->base & FLASH_CONTROLLER_1_MSK);
	const uint32_t erase_sector_req = is_flash_ctrl1 ? SROMAPI_ERASESECTOR1_REQ : SROMAPI_ERASESECTOR0_REQ;
	struct target *target = bank->target;
	struct working_area *wa;

	LOG_DEBUG("MXS40 patform: eraseing row @%08X", addr);
	uint8_t srom_params[2 * sizeof(uint32_t)];

	int hr = target_alloc_working_area(target, sizeof(srom_params), &wa);
	if (hr != ERROR_OK)
		goto exit;

	buf_set_u32(srom_params + 0x00, 0, 32, erase_sector ? erase_sector_req : SROMAPI_ERASEROW_REQ);
	buf_set_u32(srom_params + 0x04, 0, 32, addr);

	hr = target_write_buffer(target, wa->address, sizeof(srom_params), srom_params);
	if (hr != ERROR_OK)
		goto exit_free_wa;

	uint32_t data_out;
	hr = mxs40_call_sromapi(bank, SROMAPI_ERASEROW_REQ, wa->address, &data_out);

exit_free_wa:
	target_free_working_area(target, wa);

exit:
	return hr;
}

/** ***********************************************************************************************
 * @brief Performs Erase operation using asynchronous flash algorithm
 * @param bank current flash bank
 * @param first first sector to erase
 * @param last last sector to erase
 * @param erase_builder_p pointer to erase_builder function
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_erase_with_algo(struct flash_bank *bank, int first, int last, erase_builder erase_builder_p)
{
	int hr;
	struct target *target = bank->target;
	struct mxs40_bank_info *info = bank->driver_priv;

	const size_t algo_size = info->erase_algo_size;
	const uint8_t *algo_p = info->erase_algo_p;

	struct working_area *wa_algorithm;
	struct working_area *wa_stack;
	struct working_area *wa_buffer;

	uint32_t address_buffer[last - first + 1];
	memset(address_buffer, 0, sizeof(address_buffer));

	if(info->prepare_function) {
		hr = info->prepare_function(bank);
		if (hr != ERROR_OK)
			return hr;
	}

	/* Allocate buffer for the algorithm */
	hr = target_alloc_working_area(target, algo_size, &wa_algorithm);
	if (hr != ERROR_OK)
		return hr;

	/* Write the algorithm code */
	hr = target_write_buffer(target, wa_algorithm->address, algo_size, algo_p);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Allocate buffer for the stack */
	hr = target_alloc_working_area(target, RAM_STACK_WA_SIZE, &wa_stack);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Allocate circular buffer for 16 addresses, this should be sufficient */
	hr = target_alloc_working_area(target, 16 * sizeof(uint32_t) + 8, &wa_buffer);
	if(hr != ERROR_OK)
		goto err_free_wa_stack;

	size_t num_addresses_in_buffer = erase_builder_p(bank, first, last, address_buffer);
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
		if ((srom_result & SROMAPI_STATUS_MSK) != SROMAPI_STAT_SUCCESS) {
			LOG_ERROR("SROM API execution failed. Status: 0x%08X", srom_result);
			hr = ERROR_FAIL;
		}
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

/** ***********************************************************************************************
 * @brief Programs single Flash Row
 * @param bank current flash bank
 * @param addr address of the flash row
 * @param buffer pointer to the buffer with data
 * @param use_writerow true if current flash bank belongs to Supervisory Flash
 * @param data_size - size of data to be programmed
 *   0 – 1 byte     1 - 2 bytes    2 - 4 bytes     3 – 8 bytes     4 – 16 bytes
 *   5 – 32 bytes   6 – 64 bytes   7 - 128 bytes   8 - 256 bytes   9 - 512 bytes
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_program_row_inner(struct flash_bank *bank, uint32_t addr, const uint8_t *buffer,
    bool use_writerow, uint8_t data_size)
{
	const bool is_flash_ctrl1 = (bank->base & FLASH_CONTROLLER_1_MSK);
	const uint32_t program_row_req = is_flash_ctrl1 ? SROMAPI_PROGRAMROW1_REQ : SROMAPI_PROGRAMROW0_REQ;

	struct target *target = bank->target;
	struct working_area *wa;
	const uint32_t sromapi_req = use_writerow ? SROMAPI_WRITEROW_REQ : program_row_req;
	const size_t num_bytes = (1u << data_size);
	uint32_t data_out;
	int hr;

	LOG_DEBUG("MXS40 patform: programming row @%08X", addr);
	uint8_t srom_params[4 * sizeof(uint32_t)];

	hr = target_alloc_working_area(target, sizeof(srom_params) + num_bytes, &wa);
	if (hr != ERROR_OK)
		goto exit;

	buf_set_u32(srom_params + 0x00, 0, 32, sromapi_req);
	buf_set_u32(srom_params + 0x04, 0, 32, 0x100 | data_size);
	buf_set_u32(srom_params + 0x08, 0, 32, addr);
	buf_set_u32(srom_params + 0x0C, 0, 32, wa->address + 0x10);

	hr = target_write_buffer(target, wa->address, sizeof(srom_params), srom_params);
	if (hr != ERROR_OK)
		goto exit_free_wa;

	hr = target_write_buffer(target, wa->address + 0x10, num_bytes, buffer);
	if (hr != ERROR_OK)
		goto exit_free_wa;

	hr = mxs40_call_sromapi(bank, sromapi_req, wa->address, &data_out);

exit_free_wa:
	target_free_working_area(target, wa);

exit:
	return hr;
}

/** ***********************************************************************************************
 * @brief Programs single Flash Row
 * @param bank current flash bank
 * @param addr address of the flash row
 * @param buffer pointer to the buffer with data
 * @param use_writerow true if current flash bank belongs to Supervisory Flash
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_program_row(struct flash_bank *bank, uint32_t addr, const uint8_t *buffer, bool use_writerow)
{
    return mxs40_program_row_inner(bank, addr, buffer, use_writerow, 9);
}

/** ***********************************************************************************************
 * @brief Performs Program operation
 * @param bank current flash bank
 * @param buffer pointer to the buffer with data
 * @param offset starting offset in falsh bank
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_program(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct mxs40_bank_info *info = bank->driver_priv;
	int hr = ERROR_OK;
	bool program_skipped = false;

	assert(offset % info->page_size == 0);
	assert(count % info->page_size == 0);

	const bool is_sflash = mxs40_flash_bank_matches(bank, info->regs->mem_base_sflash);

	if (!is_sflash && target->coreid != 0xFF) { /* Special core_id for fake SysAP */
		hr = mxs40_program_with_algo(bank, buffer, offset, count);
		if(hr == ERROR_OK || hr != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			return hr;
	}

	hr = mxs40_sromalgo_prepare(bank);
	if (hr != ERROR_OK)
		goto exit;

	progress_init(count / info->page_size, PROGRAMMING);
	for (size_t i = 0; i < count / info->page_size; i++) {
		const uint32_t page_addr = bank->base + offset + i * info->page_size;
		const bool is_safe_sflash_page = mxs40_is_safe_sflash_page(info, page_addr, PROGRAM);

		if (!is_sflash || is_safe_sflash_page) {
			hr = mxs40_program_row(bank, page_addr, buffer, is_sflash);
			if (hr != ERROR_OK) {
				LOG_ERROR("Failed to program Flash at address 0x%08X", page_addr);

				/* Ignore possigle errors in case we are dealing with SFlash */
				if (!is_sflash)
					goto exit;
			}
		} else if (is_sflash) program_skipped = true;

		progress_sofar(i + 1);
		buffer += info->page_size;
	}

exit:
	mxs40_sromalgo_release(target);
	progress_done(hr);

	if(program_skipped)
		LOG_WARNING("Some SFlash rows were skipped during programming, see 'sflash_restrictions' command");

	return hr;
}

/** ***********************************************************************************************
 * @brief Performs Program operation
 * @param bank current flash bank
 * @param buffer pointer to the buffer with data
 * @param offset starting offset in falsh bank
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_program_with_algo(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct mxs40_bank_info *info = bank->driver_priv;
	int hr;

	const size_t algo_size = info->program_algo_size;
	const uint8_t *algo_p = info->program_algo_p;

	struct working_area *wa_algorithm;
	struct working_area *wa_stack;
	struct working_area *wa_buffer;

	if(info->prepare_function) {
		hr = info->prepare_function(bank);
		if (hr != ERROR_OK)
			return hr;
	}

	/* Allocate buffer for the algorithm */
	hr = target_alloc_working_area(target, algo_size, &wa_algorithm);
	if (hr != ERROR_OK)
		return hr;

	/* Write the algorithm code */
	hr = target_write_buffer(target, wa_algorithm->address, algo_size, algo_p);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Allocate buffer for the stack */
	hr = target_alloc_working_area(target, RAM_STACK_WA_SIZE, &wa_stack);
	if (hr != ERROR_OK)
		goto err_free_wa_algo;

	/* Try to allocate as large RAM Buffer as possible */
	const uint32_t wa_avail = target_get_working_area_avail(target);
	uint32_t num_rows = (wa_avail - 8) / info->page_size;
	if (num_rows <= 4) {
		LOG_WARNING("Failed to allocate Circular Buffer, falling back to DAP mode");
		hr = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto err_free_wa_stack;
	}

	hr = target_alloc_working_area(target, num_rows * info->page_size + 8, &wa_buffer);
	assert(hr == ERROR_OK);

	LOG_DEBUG("Allocated buffer for %d pages (%d bytes)", num_rows, num_rows * info->page_size);

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
	buf_set_u32(reg_params[3].value, 0, 32, count / info->page_size);
	buf_set_u32(reg_params[4].value, 0, 32, wa_stack->address + wa_stack->size);

	hr = target_run_flash_async_algorithm(target, buffer, count / info->page_size,
			info->page_size, 0, NULL, ARRAY_SIZE(reg_params), reg_params,
			wa_buffer->address, wa_buffer->size,
			wa_algorithm->address, 0, &armv7m_algo);

	if (hr != ERROR_OK) {
		uint32_t srom_result = buf_get_u32(reg_params[0].value, 0, 32);
		if ((srom_result & SROMAPI_STATUS_MSK) != SROMAPI_STAT_SUCCESS) {
			LOG_ERROR("SROM API execution failed. Status: 0x%08X", srom_result);
			hr = ERROR_FAIL;
		}
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

static bool mxs40_is_address_bootable(uint32_t variant, target_addr_t addr)
{
	switch (variant) {
	case MXS40_VARIANT_PSOC6_BLE2:
	case MXS40_VARIANT_PSOC6A_2M:
		return  (addr >= 0x10000000 && addr < 0x10200000) || /* Main Flash, 2 MiB max */
				(addr >= 0x08000000 && addr < 0x08100000) || /* RAM, 1 MiB max */
				(addr >= 0x18000000 && addr < 0x20000000);   /* XIP, 128 MiB max */

	case MXS40_VARIANT_MACAW:
		return  (addr < 0x00008000) ||                       /* ROM, 32 KiB max */
				(addr >= 0x08000000 && addr < 0x08002000);   /* RAM, 8 KiB max */

	case MXS40_VARIANT_TRAVEO_II: /* xmc5xxx, traveo21 (T2G-C-2D-4M) drivers */
		// TODO: Below mapping is for T2G-C-2D-4M, may need other Traveo II variants
		return                        (addr < 0x00010000) || /* CM7_ITCM_INTERNAL, 64 KiB */
				(addr >= 0x10000000 && addr < 0x12208000) || /* FLASH LG & SM regions in Single or Dual bank modes, 4,160 KiB */
				(addr >= 0x28000000 && addr < 0x28060000) || /* SRAM, 384 KiB */
				(addr >= 0x60000000 && addr < 0x80000000) || /* SMIF0_XIP, 512 MiB */
				(addr >= 0x80000000 && addr < 0xA0000000) || /* SMIF1_XIP, 512 MiB */
				(addr >= 0xA0000000 && addr < 0xA0010000);   /* CM7_0_ITCM, 64 KiB */

	case MXS40_VARIANT_TRAVEO_II_8M: /* cat1c (xmc7xxx), traveo22 drivers */
		return  (addr >= 0x10000000 && addr < 0x12418000) || /* FLASH LG & SM regions in Single or Dual bank modes, 8384 KiB */
				(addr >= 0x28000000 && addr < 0x28100000) || /* SRAM, 1 MiB */
				(addr >= 0xA0000000 && addr < 0xA0004000) || /* CM7_0_ITCM, 16 KiB */
				(addr >= 0xA0100000 && addr < 0xA0104000) || /* CM7_1_ITCM, 16 KiB */
				(addr >= 0x60000000 && addr < 0x68000000);   /* SMIF_XIP, 128 MiB */

	case MXS40_VARIANT_TRAVEO_II_16M: /* traveo23 driver */
		return  (addr >= 0x10000000 && addr < 0x12418000) || /* FLASH LG & SM regions in Single or Dual bank modes, 8384 KiB */
				(addr >= 0x18000000 && addr < 0x1C000000) || /* FLASH1 LG & SM regions in Single or Dual bank modes, 8384 KiB */
				(addr >= 0xA0000000 && addr < 0xA0004000) || /* CM7_0_ITCM, 16 KiB */
				(addr >= 0xA0100000 && addr < 0xA0104000) || /* CM7_1_ITCM, 16 KiB */
				(addr >= 0xA0200000 && addr < 0xA0204000) || /* CM7_2_ITCM, 16 KiB */
				(addr >= 0xA0300000 && addr < 0xA0304000) || /* CM7_3_ITCM, 16 KiB */
				(addr >= 0x28000000 && addr < 0x28200000) || /* SRAM, 2 MiB */
				(addr >= 0x60000000 && addr < 0x68000000);   /* SMIF_XIP, 128 MiB */

	default:
		LOG_WARNING("Unsupported MXS40 variant: %" PRIu32 ", reset_halt may fail" , variant);
		return true;
	}
}

/** ***********************************************************************************************
 * @brief Simulates broken Vector Catch
 * Function will try to determine entry point of user application. If it succeeds it will set HW
 * breakpoint at that address, issue SW Reset and remove the breakpoint afterwards.
 * In case of CM0, SYSRESETREQ is used. This allows to reset all peripherals. Boot code will
 * reset CM4 anyway, so using SYSRESETREQ is safe here.
 * In case of CM4, VECTRESET is used instead of SYSRESETREQ to not disturb CM0 core.
 *
 * @param target current target
 * @param mode overrides default reset type for the core
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int mxs40_reset_halt(struct target *target, enum reset_halt_mode mode)
{
	const struct mxs40_bank_info *info = mxs40_get_bank_info_by_target(target);
	if (info == NULL) {
		LOG_ERROR("Unable to locate mxs40_bank_info structure for target %s",
			target->cmd_name);
		return ERROR_FAIL;
	}

	int hr;
	const long timeout_ms = (target->coreid == 0) ? CM0_VTOR_TIMEOUT_MS : OTHER_VTOR_TIMEOUT_MS;
	LOG_INFO("%s: Waiting up to %ld.%ld sec for valid Vector Table address...",
			 target_name(target), timeout_ms / 1000u, timeout_ms % 1000u);

	uint32_t vt_base;
	uint32_t reset_addr;
	bool vt_found = false;
	struct timeout to;
	struct armv7m_common *armv7m =target_to_armv7m(target);
	mxs40_timeout_init(&to, timeout_ms);
	while (!mxs40_timeout_expired(&to)) {
		keep_alive();

		/* Read Vector Table Offset register */
		hr = mem_ap_read_atomic_u32(armv7m->debug_ap, info->regs->vtbase[target->coreid], &vt_base);
		if (hr != ERROR_OK)
			continue;

		if ((target->coreid == 0) && (vt_base & 0xFFFF0000) == 0xFFFF0000) {
			LOG_INFO("%s: Application is invalid (VTOR = 0x%08X), reset_halt skipped",
					 target_name(target), vt_base);
			goto exit_halt_cpu;
		}

		/* Vector Table Offset must point to bootable region */
		if (!mxs40_is_address_bootable(info->regs->variant, vt_base))
			continue;

		/* Read Reset_Handler address */
		hr = mem_ap_read_atomic_u32(armv7m->debug_ap, vt_base + 4, &reset_addr);
		if (hr != ERROR_OK)
			continue;

		/* Read Reset_Handler must belong to bootable region */
		if (mxs40_is_address_bootable(info->regs->variant, reset_addr)) {
			vt_found = true;
			break;
		}
	}

	if (!vt_found) {
		LOG_INFO("%s: Vector Table not found (core not started?), reset_halt skipped", target_name(target));
		goto exit_halt_cpu;
	}

	LOG_INFO("%s: Vector Table found at 0x%08X", target_name(target), vt_base);

	/* Set breakpoint at User Application entry point */
	hr = breakpoint_add(target, reset_addr, 2, BKPT_HARD);
	if (hr != ERROR_OK)
		return hr;

	/* MXS40 platform reboots immediately after issuing SYSRESETREQ / VECTRESET
	 * this disables SWD/JTAG pins momentarily and may break communication
	 * Ignoring return value of mem_ap_write_atomic_u32 seems to be ok here */

	char *mode_str = "SYSRESETREQ";
	uint32_t rst_mask = AIRCR_SYSRESETREQ;

	if (mode == mode_default) {
		if (target->coreid) {
			mode_str = "VECTRESET";
			rst_mask = AIRCR_VECTRESET;
		}
	} else if (mode == mode_vectreset) {
		mode_str = "VECTRESET";
		rst_mask = AIRCR_VECTRESET;
	}

	/* Reset the CM0 by asserting SYSRESETREQ. This will also reset CM4 */
	LOG_INFO("%s: bkpt @0x%08X, issuing %s", target_name(target), reset_addr, mode_str);

	/* Workaround for PT-2019, both cores enter LOCKUP state with slow JTAG clock
	 * This happens probably because JTAG pins gets disconnected momentarily when
	 * SYSRESETREQ bit is written causing invalid JTAG state when pins gets connected
	 * back to the DAP by the boot code. */
#if(0)
	mem_ap_write_atomic_u32(armv7m->debug_ap, NVIC_AIRCR,
		AIRCR_VECTKEY | rst_mask);
#else
	mem_ap_write_u32(armv7m->debug_ap, NVIC_AIRCR, AIRCR_VECTKEY | rst_mask);
	struct adiv5_dap *dap = armv7m->debug_ap->dap;
	if(dap->ops->sync) {
		dap->ops->sync(dap);
	} else {
		/* SWD DAP does not support sync() method, use run() instead */
		dap->ops->run(dap);
	}
#endif

	jtag_sleep(jtag_get_nsrst_delay() * 1000u);

	/* Target is now under RESET */
	target->state = TARGET_RESET;

	/* Register cache is now invalid */
	register_cache_invalidate(target->reg_cache);

	/* Wait for debug interface to be ready */
	mxs40_timeout_init(&to, IPC_TIMEOUT_MS);
	while(!mxs40_timeout_expired(&to)) {
		if( dap_dp_init(armv7m->debug_ap->dap) != ERROR_OK ||
			target_examine_one(target) != ERROR_OK         ||
			target_poll(target) != ERROR_OK)
			continue;
		break;
	}

	/* Finally wait for the target to halt on break point */
	target_wait_state(target, TARGET_HALTED, IPC_TIMEOUT_MS);

	/* Remove the break point */
	breakpoint_remove(target, reset_addr);

	/* Wipe-out previous RTOS state, if any */
	if(target->rtos_wipe_on_reset_halt)
		rtos_wipe(target);

	return ERROR_OK;

exit_halt_cpu:
	hr = target_poll(target);
	if(hr != ERROR_OK)
		return hr;

	/* Poll again if we came here right from Reset
	 * or target_halt() will not skip writing C_HALT bit */
	if(target->state == TARGET_RESET) {
		hr = target_poll(target);
		if(hr != ERROR_OK)
			return hr;
	}

	/* Halt the target if it is running */
	if(target->state == TARGET_RUNNING) {
		hr = target_halt(target);
		if(hr != ERROR_OK)
			return hr;

		hr = target_wait_state(target, TARGET_HALTED, IPC_TIMEOUT_MS);
		if(hr != ERROR_OK)
			return hr;
	}

	return hr;
}

/** ***********************************************************************************************
 * @brief Performs Mass Erase operation
 * @param bank flash bank index to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
COMMAND_HANDLER(mxs40_handle_mass_erase_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int hr = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (hr != ERROR_OK)
		return hr;

	hr = flash_driver_erase(bank, 0, bank->num_sectors - 1);
	return hr;
}

COMMAND_HANDLER(mxs40_handle_reset_halt)
{
	enum reset_halt_mode mode;

	if (CMD_ARGC == 0)
		mode = mode_default;
	else if (CMD_ARGC == 1) {
		if (strcmp(CMD_ARGV[0], "sysresetreq") == 0)
			mode = mode_sysresetreq;
		else if (strcmp(CMD_ARGV[0], "vectreset") == 0)
			mode = mode_vectreset;
		else if (strcmp(CMD_ARGV[0], "default") == 0)
			mode = mode_default;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	return mxs40_reset_halt(target, mode);
}

COMMAND_HANDLER(mxs40_handle_sflash_restrictions)
{
	uint32_t restrictions;
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], restrictions);
	if(restrictions > 3) {
		LOG_ERROR("SFlash restriction level should be in range 0...3");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	g_sflash_restrictions = restrictions;

	if (g_sflash_restrictions)
		LOG_WARNING("SFlash programming allowed for regions: %s",
					g_sflash_restrictions == 1 ? "USER, TOC, KEY" :
					g_sflash_restrictions == 2 ? "USER, TOC, KEY, NAR" :
												 "Whole SFlash region");
	else LOG_INFO("SFlash programming disallowed, see 'sflash_restrictions' command");

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Deallocates private driver structures
 * @param bank - the bank being destroyed
 *************************************************************************************************/
void mxs40_free_driver_priv(struct flash_bank *bank)
{
	free(bank->driver_priv);
	bank->driver_priv = NULL;
}

COMMAND_HANDLER(mxs40_handle_set_region_size)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t sectors = 0;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], sectors);

	struct flash_bank *bank = get_flash_bank_by_name_noprobe(CMD_ARGV[0]);
	if (bank == NULL){
		LOG_ERROR("Invalid Flash Bank for mxs40_handle_set_region_size: %s",
				  CMD_ARGV[0]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct mxs40_bank_info *info = bank->driver_priv;
	info->size_override = sectors;

	return ERROR_OK;
}

static struct flash_bank *mxs40_get_any_bank(struct target *target)
{
	for(struct flash_bank *b = flash_bank_list(); b; b = b->next) {
		if(b->target == target) {
			b->driver->auto_probe(b);
			return b;
		}
	}

	LOG_ERROR("Unable to find ant flash bank for the target %s", target_name(target));
	return NULL;
}

COMMAND_HANDLER(mxs40_handle_sromcall_prepare)
{
	if(g_sromcall_prepare_called) {
		LOG_ERROR("SROM Call: prepare/release not in sequence");
		return ERROR_FAIL;
	}

	g_sromcall_prepare_called = true;
	struct flash_bank *bank = mxs40_get_any_bank(get_current_target(CMD_CTX));
	if(!bank)
		return ERROR_FAIL;

	return mxs40_sromalgo_prepare(bank);
}

COMMAND_HANDLER(mxs40_handle_sromcall)
{
	if(!CMD_ARGC) {
		LOG_ERROR("At least one argument required");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t sromapi_params[CMD_ARGC];

	for(size_t i = 0; i < CMD_ARGC; i++) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i], sromapi_params[i]);
		if(i == 0 && (sromapi_params[i] & 0x01) && CMD_ARGC > 1) {
			LOG_ERROR("Additional SROM API parameters can be passed via RAM buffer only, "
					  "check bit #0 of your SROM API request.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	struct target *target = get_current_target(CMD_CTX);
	struct working_area *wa = NULL;

	int hr;
	bool data_in_ram = (sromapi_params[0] & SROMAPI_DATA_LOCATION_MSK) == 0;

	if(data_in_ram) {
		hr = target_alloc_working_area(target, CMD_ARGC * sizeof(uint32_t), &wa);
		if (hr != ERROR_OK)
			goto exit;

		hr = target_write_buffer(target, wa->address, CMD_ARGC * sizeof(uint32_t), (uint8_t *)sromapi_params);
		if(hr != ERROR_OK)
			goto exit_free_wa;
	}

	struct flash_bank *bank = mxs40_get_any_bank(get_current_target(CMD_CTX));
	if(!bank) {
		hr = ERROR_FAIL;
		goto exit_free_wa;
	}

	uint32_t data_out;
	hr = mxs40_call_sromapi(bank, sromapi_params[0], wa ? wa->address : 0, &data_out);
	if(hr == ERROR_OK && data_out != 0xA0000000)
		command_print(CMD, "0x%08X", data_out);

exit_free_wa:
	if(data_in_ram)
		target_free_working_area(target, wa);

exit:
	return hr;
}

COMMAND_HANDLER(mxs40_handle_sromcall_release)
{
	if(!g_sromcall_prepare_called) {
		LOG_ERROR("SROM Call: prepare/release not in sequence");
		return ERROR_FAIL;
	}

	mxs40_sromalgo_release(get_current_target(CMD_CTX));
	g_sromcall_prepare_called = false;
	return ERROR_OK;
}

const struct command_registration mxs40_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = mxs40_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank>",
		.help = "Erases entire flash bank",
	},
	{
		.name = "reset_halt",
		.handler = mxs40_handle_reset_halt,
		.mode = COMMAND_EXEC,
		.usage = "[mode (sysresetreq, vectreset), by default core-dependent reset is used]",
		.help = "Tries to simulate broken Vector Catch",
	},
	{
		.name = "sflash_restrictions",
		.handler = mxs40_handle_sflash_restrictions,
		.mode = COMMAND_ANY,
		.usage = "<0|1|2|3>",
		.help = "Controls SFlash programming restrictions: 0:Writes disallowed, "
				"1:USER+TOC+KEY, 2:USER+TOC+KEY+NAR, 3:Whole region",
	},
	{
		.name = "set_region_size",
		.handler = mxs40_handle_set_region_size,
		.mode = COMMAND_ANY,
		.usage = "<region_name> <region_sectors>",
		.help = "Sets sectors for specified region",
	},
	{
		.name = "sromcall_prepare",
		.handler = mxs40_handle_sromcall_prepare,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Prepares mxs40 driver for direct srom calls",
	},
	{
		.name = "sromcall",
		.handler = mxs40_handle_sromcall,
		.mode = COMMAND_ANY,
		.usage = "<call_id> [param1] [param2] ...",
		.help = "Calls SROM API function <call_id> with arbitrary number of additional parameters",
	},
	{
		.name = "sromcall_release",
		.handler = mxs40_handle_sromcall_release,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Releases resources allocated by 'sromcall_prepare'",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration macaw_exec_command_handlers[] = {
	{
		.name = "reset_halt",
		.handler = mxs40_handle_reset_halt,
		.mode = COMMAND_EXEC,
		.usage = "[mode (sysresetreq, vectreset), by default core-dependent reset is used]",
		.help = "Tries to simulate broken Vector Catch",
	},
	{
		.name = "sromcall_prepare",
		.handler = mxs40_handle_sromcall_prepare,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Prepares mxs40 driver for direct srom calls",
	},
	{
		.name = "sromcall",
		.handler = mxs40_handle_sromcall,
		.mode = COMMAND_ANY,
		.usage = "<call_id> [param1] [param2] ...",
		.help = "Calls SROM API function <call_id> with arbitrary number of additional parameters",
	},
	{
		.name = "sromcall_release",
		.handler = mxs40_handle_sromcall_release,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Releases resources allocated by 'sromcall_prepare'",
	},
	COMMAND_REGISTRATION_DONE
};
