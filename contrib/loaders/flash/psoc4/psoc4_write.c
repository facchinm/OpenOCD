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

#include "psoc4_defs.h"
#include <string.h>

/** *************************************************************************************
 * @brief Represents ping-pong biffer
 ***************************************************************************************/
struct ping_pong_buffer {
	uint32_t num_rows;
	uint32_t cmd;
	uint8_t data[];
};

#define CMD_BUFFER_EMPTY 0
#define CMD_BUFFER_FULL  1
#define CMD_BUFFER_DONE  2

/** *************************************************************************************
 * @brief Optimized version of memcpy for 32-bit values, buffers should be 32-bit aliged
 ***************************************************************************************/
__inline __attribute__((always_inline))
static void memcpy32(uint32_t *dst, uint32_t *src, uint32_t count)
{
	while (count--)
		*dst++ = *src++;
}

/** *************************************************************************************
 * @brief Checks if flash row is erased
 ***************************************************************************************/
__inline __attribute__((always_inline))
static bool is_erased(uint32_t *addr, uint32_t count)
{
	while (count--) {
		if(*addr++)
			return false;
	}

	return true;
}

/** *************************************************************************************
 * @brief Waits for SROMAPI completion, checks and returns the status
 ***************************************************************************************/
__inline __attribute__((always_inline))
static uint32_t poll_sromapi()
{
	/* Some NOPs after writing to CPUSS_SYSREQ, as recommended in the TRM */
	__asm volatile("nop \n nop \n nop \n");

	const uint32_t status = read_io(PSOC4_CPUSS_SYSARG);
	if ((status & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED)
		return status;

	return 0;
}

/** *************************************************************************************
 * @brief Performs programming of a single Flash Row
 * @param src Address of the RAM buffer to be programmed
 * @param dst Target Flash address
 ***************************************************************************************/
__inline __attribute__((always_inline))
static uint32_t program_row(uint32_t src, uint32_t row_id, uint32_t macro_idx, uint32_t row_size)
{
	uint32_t sromapi_buf[2 + (row_size >> 2)];

	bool is_sflash_bank = !!(row_id & SUPERVISORY_FLASH_BIT_MASK);
	row_id &= ~(SUPERVISORY_FLASH_BIT_MASK);

	sromapi_buf[0] = PSOC4_SROM_KEY1 | ((PSOC4_SROM_KEY2 + PSOC4_CMD_LOAD_LATCH) << 8) |
		(macro_idx << 24);
	sromapi_buf[1] = row_size - 1;
	memcpy32(&sromapi_buf[2], (uint32_t *)src, row_size >> 2);

	write_io(PSOC4_CPUSS_SYSARG, sromapi_buf);
	write_io(PSOC4_CPUSS_SYSREQ, PSOC4_SROM_SYSREQ_BIT | PSOC4_CMD_LOAD_LATCH);
	uint32_t status = poll_sromapi();
	if (status)
		return status;

	if(is_sflash_bank) {
		uint32_t srom_req = PSOC4_CMD_WRITE_SFLASH_ROW;

		sromapi_buf[0] = PSOC4_SROM_KEY1 | ((PSOC4_SROM_KEY2 + srom_req) << 8);
		sromapi_buf[1] = row_id;
		write_io(PSOC4_CPUSS_SYSARG, sromapi_buf);
		write_io(PSOC4_CPUSS_SYSREQ, PSOC4_SROM_SYSREQ_BIT | srom_req);
		status = poll_sromapi();
		if (status)
			return status;

	} else {
		uint32_t srom_req = is_erased((uint32_t *)(row_id * row_size), row_size >> 2) ?
					PSOC4_CMD_PROGRAM_ROW : PSOC4_CMD_WRITE_ROW;

		sromapi_buf[0] = PSOC4_SROM_KEY1 | ((PSOC4_SROM_KEY2 + srom_req) << 8) |
				(row_id << 16);
		write_io(PSOC4_CPUSS_SYSARG, sromapi_buf);
		write_io(PSOC4_CPUSS_SYSREQ, PSOC4_SROM_SYSREQ_BIT | srom_req);
		status = poll_sromapi();
		if (status)
			return status;
	}
	return 0;
}

/** *************************************************************************************
 * @brief Entry point of the algorithm.
 * All communication is done via ping-pong buffer passed as first parameter.
 * @param buf pointer to the ping-pong buffer
 * @param row_idx index of the first flash row to be programmed
 * @param row_size size of the row
 * @param rows_per_macro number of rows per flash macro
 ***************************************************************************************/
__attribute__((flatten, noreturn))
void psoc4_write(volatile struct ping_pong_buffer *buf, uint32_t row_idx,
			  const uint32_t row_size, uint32_t const rows_per_macro)
{
	/* rows_per_macro can be 256 or 512 only, translate it to 8 or 9 */
	const uint32_t shift_count = 8 + (rows_per_macro >> 9);

	while(true) {
		if(buf->cmd == CMD_BUFFER_DONE)
			__asm("bkpt #0");

		if(buf->cmd == CMD_BUFFER_EMPTY)
			continue;

		if(buf->cmd != CMD_BUFFER_FULL)
			continue;

		uint32_t data_addr = (uint32_t)&buf->data[0];
		while(buf->num_rows) {
			/* macro_idx = row_idx / rows_per_macro == /256 or /512 == >>8 or >>9 */
			uint32_t macro_idx = row_idx >> shift_count;
			uint32_t result = program_row(data_addr, row_idx, macro_idx, row_size);
			if (result) {
				buf->cmd = result;
				__asm("bkpt #0");
			}

			row_idx++;
			buf->num_rows--;
			data_addr += row_size;
		}

		buf->cmd = CMD_BUFFER_EMPTY;
	}
}
