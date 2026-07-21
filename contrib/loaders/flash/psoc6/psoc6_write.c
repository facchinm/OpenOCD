// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2018-2025, Infineon Technologies AG, or an affiliate of *
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

#include "device.h"
#include "flashboot.h"

/** *************************************************************************************
 * @brief Represents Circular Buffer structure passed to this loader by the OpenOCD
 ***************************************************************************************/
struct circular_buffer {
	uint32_t wp;
	uint32_t rp;
	uint8_t data[];
};

/** *************************************************************************************
 * @brief Equivalent for the CLZ assembly instruction for Cortex-M0 (no Thumb2 support)
 * @param data Value to count leading zeros
 * @return number of leading zeros in 'data'
 ***************************************************************************************/
__inline __attribute__((always_inline))
static uint32_t soft_clz(uint32_t data)
{
	uint32_t count = 0u;
	uint32_t mask = 0x80000000u;

	while((data & mask) == 0) {
		count += 1u;
		mask = mask >> 1u;
	}

	return count;
}

/** *************************************************************************************
 * @brief Performs programming of a single Flash Row
 * @param src Address of the RAM buffer to be programmed
 * @param dst Target Flash address
 ***************************************************************************************/
__inline __attribute__((always_inline))
static uint32_t program_row(uint32_t src, uint32_t dst, uint32_t write_param)
{
	volatile uint32_t req[4];
#if (MXS40_VARIANT == MXS40_VARIANT_TRAVEO_II)
	const bool is_wflash0 = (dst & 0xFF000000) == 0x14000000;
	const bool is_wflash1 = (dst & 0xFF000000) == 0x1C000000;
	const bool is_fc1 = (dst & 0x08000000);
	if ((is_wflash0 || is_wflash1) && write_param != 0x102) {
		/* Use bulk request only for Work Flash and only if program page
		 * size is larger than 32 bits (write_param != 0x102) */
		req[0] = is_fc1 ? SROMAPI_PROGRAMWORKFLASH1_REQ : SROMAPI_PROGRAMWORKFLASH0_REQ;
	} else {
		/* Use regular request for Main Flash and for 32-bit writes to Work Flash */
		req[0] = is_fc1 ? SROMAPI_PROGRAMROW1_REQ : SROMAPI_PROGRAMROW0_REQ;
	}
#else
	/* Always use regular request for PSoC6 targets */
	req[0] = SROMAPI_PROGRAMROW0_REQ;
#endif
	req[1] = write_param;
	req[2] = dst;
	req[3] = src;

	call_sromapi((uint32_t)&req[0]);

	return req[0];
}

/** *************************************************************************************
 * @brief Returns largest possible programming page size
 ***************************************************************************************/
__inline __attribute__((always_inline))
static uint32_t get_page_size(uint32_t address)
{
#if (MXS40_VARIANT == MXS40_VARIANT_TRAVEO_II)
	if ((address & 0xFF000000) == 0x14000000) {
		uint32_t fb_ver_major = read_io(FLASHBOOT_VER_HI_ADDR) >> 28u;
		if(fb_ver_major < 2)
			return 4;

		uint32_t fb_build = read_io(FLASHBOOT_VER_LO_ADDR) & 0xFFFFu;
		return (fb_build < FLASHBOOT_BUILD_BULK_SUPPORTED) ? 4 : 512;
	}
#endif
	(void)address;
	return 512;
}

__attribute__((flatten, noreturn))
void write(volatile struct circular_buffer *work_area, uint32_t fifo_end,
	uint32_t target_address, uint32_t count)
{
	__asm volatile ("cpsie i" : : : "memory");
	const uint32_t page_size = get_page_size(target_address);
	const uint32_t write_param = 0x100 | (31 - soft_clz(page_size));

	const int ipc_id = get_ipc_id();
	const uint32_t ipc_intr_mask = read_io(REG_IPC_INTR_MASK(IPC_INTR_ID));
	const uint32_t ipc_intr_mask_ok = 1u << (16 + ipc_id);
	if((ipc_intr_mask & ipc_intr_mask_ok) == 0)
		write_io(REG_IPC_INTR_MASK(IPC_INTR_ID), ipc_intr_mask | ipc_intr_mask_ok);

	while (count) {
		/* Wait for some data in the FIFO */
		while (work_area->rp == work_area->wp);

		uint32_t result = program_row(work_area->rp, target_address, write_param);
		if ((result & SROMAPI_STATUS_MSK) != SROMAPI_STAT_SUCCESS) {
			work_area->rp = 0;
			__asm volatile ("mov r0, %[value]\n\t"
			"bkpt 0"  :  : [value] "r" (result) : );
		}

		target_address += page_size;

		uint32_t read_ptr = work_area->rp;
		read_ptr += page_size;
		if (read_ptr >= fifo_end)
			read_ptr = ((uint32_t)&work_area->data);

		work_area->rp = read_ptr;

		count--;
	}

	for (;;)
		__asm("bkpt 0");
}
