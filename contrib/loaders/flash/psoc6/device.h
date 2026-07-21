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

#include <stdint.h>
#include <stdbool.h>

#define MXS40_VARIANT_PSOC6A_BLE        1
#define MXS40_VARIANT_PSOC6A_2M         2
#define MXS40_VARIANT_TRAVEO_II         3

#ifndef MXS40_VARIANT
#define MXS40_VARIANT MXS40_VARIANT_TRAVEO_II
#endif

#ifndef MXS40_VARIANT
  #error "MXS40_VARIANT not defined"
#endif

#if (MXS40_VARIANT == MXS40_VARIANT_PSOC6A_BLE)
  #define MEM_BASE_IPC                    0x40230000u
  #define MEM_BASE_IPC_INTR               0x40231000u
  #define IPC_LOCK_STATUS_OFFSET          0x10u
  #define MXS40_CPUSS_IDENTITY            0x40210400
#elif (MXS40_VARIANT == MXS40_VARIANT_PSOC6A_2M || MXS40_VARIANT == MXS40_VARIANT_TRAVEO_II)
  #define MEM_BASE_IPC                    0x40220000u
  #define MEM_BASE_IPC_INTR               0x40221000u
  #define IPC_LOCK_STATUS_OFFSET          0x1Cu
  #define MXS40_CPUSS_IDENTITY            0x40200000
#else
  #error "MXS40_VARIANT unknown"
#endif

#define MXS40_CPUSS_IDENTITY_MS_MSK       0x00000F00
#define MXS40_CPUSS_IDENTITY_MS_M0        0x000
#define IPC_INTR_ID                       0

#define IPC_STRUCT_SIZE                 0x20u
#define IPC_INTR_STRUCT_SIZE            0x20u

#define MEM_IPC(n)                      (MEM_BASE_IPC + (n) * IPC_STRUCT_SIZE)
#define MEM_IPC_INTR(n)                 (MEM_BASE_IPC_INTR + (n) * IPC_INTR_STRUCT_SIZE)

#define REG_IPC_ACQUIRE(n)              (MEM_IPC(n) + 0x00u)
#define REG_IPC_NOTIFY(n)               (MEM_IPC(n) + 0x08u)
#define REG_IPC_DATA(n)                 (MEM_IPC(n) + 0x0Cu)
#define REG_IPC_LOCK_STATUS(n)          (MEM_IPC(n) + IPC_LOCK_STATUS_OFFSET)
#define REG_IPC_INTR_MASK(n)            (MEM_IPC_INTR(n) + 0x08u)

#define IPC_ACQUIRE_SUCCESS_MSK         0x80000000u
#define IPC_LOCK_ACQUIRED_MSK           0x80000000u

#define SROMAPI_PROGRAMROW0_REQ         0x06000100u
#define SROMAPI_PROGRAMWORKFLASH0_REQ   0x30000100u
#define SROMAPI_ERASESECTOR0_REQ        0x14000100u

#define SROMAPI_PROGRAMROW1_REQ         0x09000100u
#define SROMAPI_PROGRAMWORKFLASH1_REQ   0x31000100u
#define SROMAPI_ERASESECTOR1_REQ        0x1C000100u

#define SROMAPI_ERASESUBSECTOR_REQ      0x1D000100u
#define SROMAPI_ERASEROW_REQ            0x1C000100u

#define SROMAPI_STATUS_MSK              0xF0000000u
#define SROMAPI_STAT_SUCCESS            0xA0000000u

#define read_io(addr)                  (*((volatile uint32_t *)(addr)))
#define write_io(addr, value)          (*((volatile uint32_t *)(addr)) = (uint32_t)(value))

__inline __attribute__((always_inline))
static int get_ipc_id(void)
{
	int is_cm0;
	uint32_t id = read_io(MXS40_CPUSS_IDENTITY);
	is_cm0 = (id & MXS40_CPUSS_IDENTITY_MS_MSK) == MXS40_CPUSS_IDENTITY_MS_M0;
	return is_cm0 ? 0 : 1;
}

/** *************************************************************************************
 * @brief Invoke SROM API Function
 * @param req_buf Address of the buffer with SROM API Request and parameters
 ***************************************************************************************/
__inline __attribute__((always_inline))
static void call_sromapi(uint32_t req_buf)
{
	int ipc_id = get_ipc_id();

	/* Acquire the IPC structure */
	for (;;) {
		uint32_t val = read_io(REG_IPC_ACQUIRE(ipc_id));
		bool is_acquired = (val & IPC_ACQUIRE_SUCCESS_MSK) != 0;
		if (is_acquired)
			break;
	}

	/* Invoke SROM API by posting an NMI via IPC */
	write_io(REG_IPC_DATA(ipc_id), req_buf);
	write_io(REG_IPC_NOTIFY(ipc_id), 1);

	/* Poll the IPC structure for release */
	for (;;) {
		uint32_t lock_stat = read_io(REG_IPC_LOCK_STATUS(ipc_id));
		bool is_locked = (lock_stat & IPC_LOCK_ACQUIRED_MSK) != 0;
		if (!is_locked)
			break;
	}
}
