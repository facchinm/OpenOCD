// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2019-2025, Infineon Technologies AG, or an affiliate of *
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

#include <stdbool.h>
#include <stdint.h>

typedef volatile uint32_t * reg32_t;

#define PSOC4_CMD_LOAD_LATCH		0x04
#define PSOC4_CMD_WRITE_ROW			0x05
#define PSOC4_CMD_PROGRAM_ROW		0x06
#define PSOC4_CMD_WRITE_SFLASH_ROW	0x18
#define PSOC4_SROM_KEY1				0xB6
#define PSOC4_SROM_KEY2				0xD3
#define PSOC4_SROM_SYSREQ_BIT		(1u << 31)
#define PSOC4_SROM_STATUS_SUCCEEDED 0xA0000000
#define PSOC4_SROM_STATUS_MASK		0xF0000000
#define SFLASH_PROGRAMMING_MASK		0x00000100

#define read_io(addr)		  (*((volatile uint32_t *)(addr)))
#define write_io(addr, value) (*((volatile uint32_t *)(addr)) = (uint32_t)(value))

struct chunk {
	uint32_t param[2];
	/* uint8_t data[row_szie] */
};

struct header {
	reg32_t  sysreq_reg;
	reg32_t  sysarg_reg;
	uint32_t row_size;
	uint32_t num_chunks;
	/* struct chunk chunks[num_buffers] */
};

#define FLATTERN __inline __attribute__((always_inline)) static

/** *************************************************************************************
 * @brief checks if given row contents is all-zeros. Depending on results algorithm will
 * choose what SROM API to use for programming - either ProgramRow or WriteRow.
 * @param addr start address of the row, must be 32-bit aligned
 * @param count number of bytes to verify
 * @return true if row is erased
 ****************************************************************************************/
FLATTERN bool is_erased_psoc4(reg32_t addr, uint32_t count)
{
	count >>= 2;
	while (count--)
		if (*addr++)
			return false;

	return true;
}

FLATTERN bool is_erased_psoc4hv(uint8_t fc_idx, reg32_t addr, uint32_t count)
{
	const reg32_t fc_ctrl =   fc_idx ? (reg32_t)0x401000A8 : (reg32_t)0x40100030;
	const reg32_t berr_ctrl = fc_idx ? (reg32_t)0x40100504 : (reg32_t)0x40100500;

	uint32_t fc_ctrl_val = read_io(fc_ctrl);
	write_io(fc_ctrl, fc_ctrl_val | (1 << 21));
	write_io(berr_ctrl, 0xFFFFFFFFu);

	bool empty = is_erased_psoc4(addr, count);
	if (read_io(berr_ctrl) & 2u) {
		write_io(berr_ctrl, 0xFFFFFFFFu);
		empty = false;
	}

	write_io(fc_ctrl, fc_ctrl_val);
	return empty;
}

/** *************************************************************************************
 * @brief calls SROM API function and checks return status.
 * @param header structure with MCU-specific data and number of data blocks in the buffer
 * @param chunk pointer to data block to be programmed
 * @param req SROM API to call - ProgramRow/WriteRow/WriteSflashRow
 ****************************************************************************************/
FLATTERN void call_srom(volatile struct header *header, struct chunk *chunk, uint32_t req)
{
	write_io(header->sysarg_reg, chunk);
	write_io(header->sysreq_reg, PSOC4_SROM_SYSREQ_BIT | req);
	__asm volatile("nop \n nop \n");
	uint32_t status = read_io(header->sysarg_reg);
	if ((status & PSOC4_SROM_STATUS_MASK) != PSOC4_SROM_STATUS_SUCCEEDED) {
		header->num_chunks = status;
		__asm("bkpt #0");
	}
}

/** *************************************************************************************
 * @brief parses record header and calls LoadFlashBytes Program/Write(SFlash)Row APIs.
 * @param header structure with MCU-specific data and number of data blocks in the buffer
 * @param flash_addr address of flash row to be programmed
 * @param chunk pointer to data block to be programmed
 *
 * Each data chunk consists of a header (two 32-bit values) followed by row data.
 ****************************************************************************************/
FLATTERN void process_chunk(volatile struct header *header, reg32_t flash_addr, struct chunk *chunk)
{
	/* Backup chunk's header, the following code will overwrite it */
	const uint32_t p0_backup = chunk->param[0];
	const uint32_t p1_backup = chunk->param[1];

	/* Call LoadFlashBytes API */
	uint32_t load_param = p0_backup & 0xFFFF0000;
	chunk->param[0] = load_param | PSOC4_SROM_KEY1 | ((PSOC4_SROM_KEY2 + PSOC4_CMD_LOAD_LATCH) << 8);
	chunk->param[1] = p0_backup & 0x000000FF;
	call_srom(header, chunk, PSOC4_CMD_LOAD_LATCH);

#if defined(PSOC4_VARIANT_PSOC4)
	bool is_erased = is_erased_psoc4(flash_addr, header->row_size);
#elif defined(PSOC4_VARIANT_PSOC4HV)
	bool is_erased = is_erased_psoc4hv(!!(p0_backup & 0x80000000u), flash_addr, header->row_size);
#else
  #error "Unknown PSOC4 Variant"
#endif

	/* Call one of the ProgramRow/WriteRow/WriteSflashRow APIs */
	uint32_t write_param = p1_backup & 0xFFFF0000;
	uint8_t req = p1_backup & SFLASH_PROGRAMMING_MASK   ? PSOC4_CMD_WRITE_SFLASH_ROW
											: is_erased ? PSOC4_CMD_PROGRAM_ROW
														: PSOC4_CMD_WRITE_ROW;
	chunk->param[0] = write_param | PSOC4_SROM_KEY1 | ((PSOC4_SROM_KEY2 + req) << 8);
	chunk->param[1] = p1_backup & 0x000000FF;
	call_srom(header, chunk, req);
}

/** *************************************************************************************
 * @brief entry point of the algorithm. This function supports Main, Work and SFlash
 * programming on all PSoC4 flavors - legacy, regular and automotive (hv).
 * @param header structure with MCU-specific data followed by data chunks
 * @param flash_addr address of the first flash row to be programmed
 *
 * Struct header is immediately followed by 'header.num_chunks' chunks of data. Each
 * data chunk in turn has it's own header containing SROM API parameters followed by
 * 'header.row_size' bytes of data. The size of chunk's header is two 32-bit values.
 *
 * Chunk's header contains parameters for SROM API encoded as follows:
 * PARAM[0] - parameters for LoadFlashBytes API
 *   [ControllerID]:1, [MacroID]:7,  [UNUSED]:16, [RowSize-1]:8
 * PARAM[1] - parameters for ProgramRow/WriteRow/WriteSflashRow APIs
 *   [ControllerID]:1,.[RowID]:15,   [UNUSED]:7,  [WriteSflash]:1  [RowID]:8
 ***************************************************************************************/
__attribute__((used, flatten, noreturn)) void psoc4hv_write(volatile struct header *header, reg32_t flash_addr)
{
	while (true) {
		/* Wait for the OpenOCD to fill data buffers */
		while (header->num_chunks == 0)
			continue;

		/* Proceed with flash programming */
		uint8_t *chunk_p = (uint8_t *)(&header[1]);
		for (uint32_t i = 0; i < header->num_chunks; i++) {
			process_chunk(header, flash_addr, (struct chunk *)chunk_p);
			chunk_p += header->row_size + 8;
			flash_addr += (header->row_size / 4);
		}

		/* All chunks programmed, report completion to the OpenOCD  */
		header->num_chunks = 0;
	}
}
