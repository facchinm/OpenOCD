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
#include <stddef.h>
#include <stdint.h>

#define FLASH_ROW_SIZE       512
#define SROMAPI_PROGRAM_ROW  0x1080FFE4u
#define CYBOOT_FLASH_SUCCESS 0x0D50B002
#define BLOCKING_OPERATION   0

#define FLASH_BANK_BASE_SECURE 0x32000000
#define FLASH_BANK_BASE_NON_SECURE 0x22000000
#define FLAH_BANK_BASE_VIRTUAL_SECURE 0x12000000
#define FLAH_BANK_BASE_VIRTUAL_NON_SECURE 0x02000000

#define FLASH_BOOT_VER_ADDR 0x13401408
#define FLASH_BOOT_VER_A0 12031 /*PSC3 GENERIC ES100 VER*/

#define FAMILY_ID_ADDR    0x03400004
#define FAMILY_ID_MASK    0x0000FFFF
#define FAMILY_ID_PSC3    0x118
#define FAMILY_ID_PSC3_P8 0x119

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

/* returns true if it is generic psc3 (not p8) */
__attribute__((always_inline)) static inline bool is_psc3_generic(void)
{
	bool ret;
	uint32_t* pFamily_id = (uint32_t*)FAMILY_ID_ADDR;

	ret = ((*pFamily_id & FAMILY_ID_MASK) == FAMILY_ID_PSC3) ? true : false;

	return ret;
}

/* returns flashboot version */
__attribute__((always_inline)) static inline uint32_t get_flash_boot_ver(void)
{
	uint32_t* pFb_ver = (uint32_t*)FLASH_BOOT_VER_ADDR;
	return *pFb_ver;
}

/* stops fl execution and sets result to display on oocd side*/
__attribute__((always_inline)) static inline void stop_execution(volatile struct circular_buffer *wa, uint32_t result)
{
	wa->rp = 0;
	__asm volatile(
		"mov r0, %[value]\n\t"
		"bkpt 0"
		:
		: [value] "r"(result)
		:);
}

__attribute__((always_inline)) static inline void my_memcpy(uint32_t *src, uint32_t *dst, size_t len) {
  while (len--)
	*dst++ = *src++;
}

typedef uint32_t (*cyboot_flash_program_row)(uint32_t flash_address, const void *data,
											 struct cyboot_flash_context *ctx);

__attribute__((flatten, noreturn)) void write(volatile struct circular_buffer *work_area, uint32_t fifo_end,
											  uint32_t target_address, uint32_t count) {
  struct cyboot_flash_context ctxt;
  uint32_t result;
  cyboot_flash_program_row api = (cyboot_flash_program_row)(*(uint32_t *)SROMAPI_PROGRAM_ROW);

  uint32_t page_buffer[FLASH_ROW_SIZE / 4 + 4];

  while (count) {
	/* Wait for some data in the FIFO */
	while (work_area->rp == work_area->wp)
	  continue;

	ctxt.flags          = BLOCKING_OPERATION;
	ctxt.hv_params_addr = 0;
	ctxt.refresh        = 0;

	if (is_psc3_generic() && (get_flash_boot_ver() <= FLASH_BOOT_VER_A0)) {
		if (target_address == FLASH_BANK_BASE_SECURE ||
			target_address == FLASH_BANK_BASE_NON_SECURE ||
			target_address == FLAH_BANK_BASE_VIRTUAL_SECURE ||
			target_address == FLAH_BANK_BASE_VIRTUAL_NON_SECURE) {

			/* erase row 0 to prevent ECC incorrect operation on early flash boot */
			#define SROMAPI_ERASE_ROW   0x1080FFE0u
			typedef uint32_t (*cyboot_flash_erase_row_t)(uint32_t flash_address, struct cyboot_flash_context *ctx);
			cyboot_flash_erase_row_t erase_api = (cyboot_flash_erase_row_t)(*(uint32_t *)SROMAPI_ERASE_ROW);
			result = erase_api(target_address, &ctxt);
			if (result != CYBOOT_FLASH_SUCCESS) {
				stop_execution(work_area, result);
			}
		}
	}

	my_memcpy((uint32_t *)work_area->rp, page_buffer, FLASH_ROW_SIZE / 4);
	result = api(target_address, (void *)page_buffer, &ctxt);
	if (result != CYBOOT_FLASH_SUCCESS) {
		stop_execution(work_area, result);
	}

	target_address += FLASH_ROW_SIZE;

	uint32_t read_ptr = work_area->rp;
	read_ptr += FLASH_ROW_SIZE;
	if (read_ptr >= fifo_end)
	  read_ptr = ((uint32_t)&work_area->data);

	work_area->rp = read_ptr;
	count--;
  }

  for (;;)
	__asm("bkpt 0");
}
