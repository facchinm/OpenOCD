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
 * @brief Performs programming of a single Flash Row
 * @param src Address of the RAM buffer to be programmed
 * @param dst Target Flash address
 ***************************************************************************************/
__inline __attribute__((always_inline))
static uint32_t erase_row_or_sector(uint32_t flash_addr)
{
	volatile uint32_t req[2];
	switch (flash_addr & 0x03) {
	case 0:
		req[0] = SROMAPI_ERASEROW_REQ;
		break;
	case 1:
#if (MXS40_VARIANT == MXS40_VARIANT_TRAVEO_II)
		req[0] = (flash_addr & 0x08000000) ? SROMAPI_ERASESECTOR1_REQ : SROMAPI_ERASESECTOR0_REQ;
#else
		req[0] = SROMAPI_ERASESECTOR0_REQ;
#endif
		break;
	case 2:
		req[0] = SROMAPI_ERASESUBSECTOR_REQ;
		break;
	case 3:
	default:
		return 0xF00DBAAD;
	}
	req[1] = flash_addr & ~3u;

	call_sromapi((uint32_t)&req[0]);

	return req[0];
}

#include <string.h>

__attribute__((flatten, noreturn))
void write(volatile struct circular_buffer *work_area, uint32_t fifo_end, uint32_t count)
{
	__asm volatile ("cpsie i" : : : "memory");

	const int ipc_id = get_ipc_id();
	const uint32_t ipc_intr_mask = read_io(REG_IPC_INTR_MASK(IPC_INTR_ID));
	const uint32_t ipc_intr_mask_ok = 1u << (16 + ipc_id);
	if((ipc_intr_mask & ipc_intr_mask_ok) == 0)
		write_io(REG_IPC_INTR_MASK(IPC_INTR_ID), ipc_intr_mask | ipc_intr_mask_ok);

	while (count) {
		/* Wait for some data in the FIFO */
		while (work_area->rp == work_area->wp);

		uint32_t u32_value = *((uint32_t *)work_area->rp);
		uint32_t result = erase_row_or_sector(u32_value);
		if ((result & SROMAPI_STATUS_MSK) != SROMAPI_STAT_SUCCESS) {
			work_area->rp = 0;
			__asm volatile ("mov r0, %[value]\n\t"
			"bkpt 0"  :  : [value] "r" (result) : );
		}

		uint32_t read_ptr = work_area->rp;
		read_ptr += 4;
		if (read_ptr >= fifo_end)
			read_ptr = ((uint32_t)&work_area->data);
		work_area->rp = read_ptr;

		count--;
	}

	for (;;)
		__asm("bkpt 0");
}
