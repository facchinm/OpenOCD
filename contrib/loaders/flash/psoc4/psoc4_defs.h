/* SPDX-License-Identifier: GPL-2.0-or-later */

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

#include <stdbool.h>
#include <stdint.h>

#define PSOC4_VARIANT_LEGACY  1
#define PSOC4_VARIANT_CURRENT 2

#ifndef PSOC4_VARIANT
#define PSOC4_VARIANT PSOC4_VARIANT_CURRENT
#endif

#ifndef PSOC4_VARIANT
#error "PSOC4_VARIANT not defined"
#endif

#define PSOC4_CMD_LOAD_LATCH          0x04
#define PSOC4_CMD_WRITE_ROW           0x05
#define PSOC4_CMD_PROGRAM_ROW         0x06
#define PSOC4_CMD_WRITE_SFLASH_ROW    0x18

#define PSOC4_SROM_KEY1               0xB6
#define PSOC4_SROM_KEY2               0xD3
#define PSOC4_SROM_SYSREQ_BIT         (1u << 31)
#define PSOC4_SROM_HMASTER_BIT        (1u << 30)
#define PSOC4_SROM_PRIVILEGED_BIT     (1u << 28)
#define PSOC4_DIS_RESET_VECT_REL_BIT  (1u << 27)
#define PSOC4_SROM_STATUS_SUCCEEDED   0xA0000000
#define PSOC4_SROM_STATUS_FAILED      0xF0000000
#define PSOC4_SROM_STATUS_MASK        0xF0000000

#define PSOC4_ROWS_PER_MACRO          512

#if (PSOC4_VARIANT == PSOC4_VARIANT_LEGACY)
#define PSOC4_CPUSS_SYSREQ            0x40000004
#define PSOC4_CPUSS_SYSARG            0x40000008
#elif (PSOC4_VARIANT == PSOC4_VARIANT_CURRENT)
#define PSOC4_CPUSS_SYSREQ            0x40100004
#define PSOC4_CPUSS_SYSARG            0x40100008
#else
#error "PSOC4_VARIANT unknown"
#endif

/* These flags are passed in MSB of row_id parameter */
#define SUPERVISORY_FLASH_BIT_MASK (1u << 31) /* Perform SFlash programming */

#define read_io(addr)                  (*((volatile uint32_t *)(addr)))
#define write_io(addr, value)          (*((volatile uint32_t *)(addr)) = (uint32_t)(value))
