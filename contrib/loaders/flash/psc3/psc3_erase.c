// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2023-2025, Infineon Technologies AG, or an affiliate of *
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

#define SROMAPI_ERASE_ROW    0x1080FFE0u
#define CYBOOT_FLASH_SUCCESS 0x0D50B002
#define BLOCKING_OPERATION   0

struct circular_buffer {
  uint32_t wp;
  uint32_t rp;
  uint8_t data[];
};

typedef struct
{
  uint32_t min_count;
  uint32_t max_count;
  uint32_t min_page_addr;
  uint32_t scratch_row_idx;
} cyboot_flash_refresh_t;

typedef void (*cyboot_flash_callback_t)(void * ctx);

struct cyboot_flash_context {
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
};

typedef uint32_t (*cyboot_flash_erase_row_t)(uint32_t flash_address, struct cyboot_flash_context *ctx);

__attribute__((flatten, noreturn)) void erase(volatile struct circular_buffer *work_area, uint32_t fifo_end,
											  uint32_t count) {
  struct cyboot_flash_context ctxt;
  cyboot_flash_erase_row_t api = (cyboot_flash_erase_row_t)(*(uint32_t *)SROMAPI_ERASE_ROW);

  while (count) {
	/* Wait for some data in the FIFO */
	while (work_area->rp == work_area->wp)
	  continue;

	uint32_t u32_value = *((uint32_t *)work_area->rp);

	ctxt.flags          = BLOCKING_OPERATION;
	ctxt.hv_params_addr = 0;
	ctxt.refresh        = 0;

	uint32_t result = api(u32_value, &ctxt);
	if (result != CYBOOT_FLASH_SUCCESS) {
	  work_area->rp = 0;
	  __asm volatile(
		  "mov r0, %[value]\n\t"
		  "bkpt 0"
		  :
		  : [value] "r"(result)
		  :);
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
