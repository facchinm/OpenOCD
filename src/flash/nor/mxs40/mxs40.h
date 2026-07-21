/* SPDX-License-Identifier: GPL-2.0-or-later */

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

#ifndef MXS40_H
#define MXS40_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "flash/nor/core.h"
#include "target/target.h"

struct bank_match {
	uint32_t address_mask;
	uint32_t address_value;
};

#define BANK_MATCH_END {0, 0}

struct mxs40_regs {
	uint32_t variant;
	uint32_t ipc_acquire;
	uint32_t ipc_notify;
	uint32_t ipc_data;
	uint32_t ipc_lock_stat;
	uint32_t ipc_intr;
	uint32_t ipc_intr_msk;
	uint32_t ppu_flush;
	uint32_t vtbase[5];
	struct bank_match mem_base_main[5];
	struct bank_match mem_base_work[5];
	struct bank_match mem_base_sflash[5];
};

#define MXS40_VARIANT_PSOC6_BLE2            1
#define MXS40_VARIANT_PSOC6A_2M             2
#define MXS40_VARIANT_MACAW                 3
#define MXS40_VARIANT_TRAVEO_II             4
#define MXS40_VARIANT_TRAVEO_II_8M          5
#define MXS40_VARIANT_TRAVEO_II_16M         6

#define IPC_INTR_MASK_1_CORE                (0x02u << 16u)
#define IPC_INTR_MASK_2_CORE                (0x04u << 16u)
#define IPC_INTR_MASK_3_CORE                (0x08u << 16u)

#define SROMAPI_SIID_REQ                    0x00000001u
#define SROMAPI_SIID_REQ_FAMILY_REVISION    (SROMAPI_SIID_REQ | 0x000u)
#define SROMAPI_SIID_REQ_SIID_PROTECTION    (SROMAPI_SIID_REQ | 0x100u)

#define SROMAPI_WRITEROW_REQ                0x05000100u
#define SROMAPI_ERASEROW_REQ                0x1C000100u

#define TV2_MAIN_FLASH0_BASE                0x10000000
#define TV2_MAIN_FLASH1_BASE                0x18000000
#define TV2_WORK_FLASH0_BASE                0x14000000
#define TV2_WORK_FLASH1_BASE                0x1C000000
#define FLASH_CONTROLLER_1_MSK              0x08000000u

#define SROMAPI_PROGRAMROW0_REQ             0x06000100u
#define SROMAPI_ERASESECTOR0_REQ            0x14000100u
#define SROMAPI_PROGRAMROW1_REQ             0x09000100u
#define SROMAPI_ERASESECTOR1_REQ            0x1C000100u

#define SROMAPI_BLANK_CHECK_REQ             0x2A000000u
#define SROMAPI_READ_FUSE_BYTE              0x03000001u
#define SROMAPI_CHECK_FACTORY_HASH          0x27000001u
#define SROMAPI_GENERATE_HASH_CODE          0x1E000000u

#define IPC_ACQUIRE_SUCCESS_MSK             0x80000000u
#define IPC_LOCK_ACQUIRED_MSK               0x80000000u

#define SROMAPI_STATUS_MSK                  0xF0000000u
#define SROMAPI_STAT_SUCCESS                0xA0000000u
#define SROMAPI_DATA_LOCATION_MSK           0x00000001u

#define IPC_TIMEOUT_MS                      20000
#define CM0_VTOR_TIMEOUT_MS                 600
#define OTHER_VTOR_TIMEOUT_MS               10000

#define MXS40_CHIP_PROT_UNKNOWN             0x00u
#define MXS40_CHIP_PROT_VIRGIN              0x01u
#define MXS40_CHIP_PROT_NORMAL              0x02u
#define MXS40_CHIP_PROT_SECURE              0x03u
#define MXS40_CHIP_PROT_DEAD                0x04u
#define MXS40_CHIP_PROT_MASK                0x0Fu

#define NVIC_VTOR                           0xE000ED08
#define RAM_STACK_WA_SIZE                   2048u

/* PSOC6A1M registers */
extern const struct mxs40_regs psoc6_ble2_regs;
/* PSoC6A2M registers */
extern const struct mxs40_regs psoc6_2m_regs;
/* Macaw registers */
extern const struct mxs40_regs macaw_regs;
/* TRAVEO™II registers */
extern const struct mxs40_regs traveo2_regs;
/* TRAVEO™II 8M registers */
extern const struct mxs40_regs traveo2_8m_regs;
/* TRAVEO™II 16M registers */
extern const struct mxs40_regs traveo2_16m_regs;

enum reset_halt_mode {
	mode_default,
	mode_sysresetreq,
	mode_vectreset
};

/* MXS40 Device Family/Die */
enum mxs40_die {
							 /* Device Family    | Family ID | Si ID Range */
	die_psoc6_ble2  = 0x100u, /* PSoC6A-BLE2      | 0x100     | E200-E2FF   */
	die_traveo2_1m  = 0x101u, /* TraveoII B-E-1M  | 0x101     | E300-E3FF   */
	die_psoc6_2m    = 0x102u, /* PSoC6A-2M        | 0x102     | E400-E4FF   */
	die_traveo2_8m  = 0x103u, /* TraveoII B-H-8M  | 0x103     | E500-E5FF   */
	die_traveo2_2m  = 0x104u, /* TraveoII B-E-2M  | 0x104     | E600-E6FF   */
	die_psoc6_512k  = 0x105u, /* PSoC6A-512K      | 0x105     | E700-E7FF   */
	die_traveo2_cl  = 0x106u, /* TraveoII Cluster | 0x106     | E800-E8FF   */
	die_macaw       = 0x109u, /* Macaw            | 0x109     | EA40-EA7F   */
	die_traveo2_16m = 0x114u, /* TraveoII B-H-16M | 0x114     | ED00-ED3F   */
	die_unknown     = 0xFFFu  /* Unknown or onsupported                     */
};

// defines efuse regions that are programmed in a special way
struct efuse_regions {
	uint32_t    lifecycle_offset;
	uint32_t    lifecycle_size;
	uint32_t    dead_access_offset;
	uint32_t    dead_access_size;
	uint32_t    secure_access_offset;
	uint32_t    secure_access_size;
} ;

struct sflash_region {
	uint32_t addr;
	uint32_t size;
	uint8_t flags;
};

struct mxs40_bank_info {
	bool is_probed;
	bool ppu_read_protected;
	bool has_erase_subsector_bug;
	uint32_t size_override;
	uint32_t page_size;
	size_t program_algo_size;
	size_t erase_algo_size;
	const uint8_t *program_algo_p;
	const uint8_t *erase_algo_p;
	const struct mxs40_regs *regs;
	const struct efuse_regions *efuse_regions;
	const struct sflash_region *sflash_regions;
	int (*prepare_function) (struct flash_bank *bank);
};

struct timeout {
	int64_t start_time;
	long timeout_ms;
};

extern const struct command_registration mxs40_exec_command_handlers[];
extern const struct command_registration macaw_exec_command_handlers[];

typedef size_t (*erase_builder)(struct flash_bank *bank, int first, int last, uint32_t *address_buffer);

/** ***********************************************************************************************
 * @brief Initializes `struct timeout` structure with given timeout value
 * @param to pointer to `struct timeout` structure
 * @param timeout_ms timeout, in milliseconds
 *************************************************************************************************/
void mxs40_timeout_init(struct timeout *to, long timeout_ms);

/** ***********************************************************************************************
 * @brief Returns true if given timeout has expired
 * @param to pointer to `struct timeout` structure
 * @return true if timeout expired
 *************************************************************************************************/
bool mxs40_timeout_expired(struct timeout *to);

/** ***********************************************************************************************
 * @brief Checks if given flash bank base address is one in the addr_array array
 * @param bank current flash bank
 * @param match pointer to the array of bank_match structures
 * @return true if flash bank matches
 *************************************************************************************************/
bool mxs40_flash_bank_matches(struct flash_bank *bank, const struct bank_match *match);

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
							  uint32_t max_precision);

/** ***********************************************************************************************
 * @brief Starts pseudo flash algorithm and leaves it running. Function allocates working area for
 * algorithm code and CPU stack, adjusts stack pointer, uploads and starts the algorithm.
 * Algorithm (a basic infinite loop) runs asynchronously while driver performs Flash operations.
 *
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_sromalgo_prepare(struct flash_bank *bank);

/** ***********************************************************************************************
 * @brief Stops running flash algorithm and releases associated resources.
 * This function is also used for cleanup in case of errors so g_stack_area may be NULL.
 * These cases have to be handled gracefully.
 *
 * @param target current target
 *************************************************************************************************/
void mxs40_sromalgo_release(struct target *target);

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
                             bool check_errors, uint32_t *data_out);
/** ***********************************************************************************************
 * @brief Invokes SROM API functions which are responsible for Flash operations
 *
 * @param bank current flash bank
 * @param req_and_params requect id of the function to invoke
 * @param working_area address of memory buffer in target's memory space for SROM API parameters
 * @param data_out pointer to variable which will be populated with execution status
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_call_sromapi(struct flash_bank *bank, uint32_t req_and_params, uint32_t working_area,
    uint32_t *data_out);

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
int mxs40_get_silicon_id(struct flash_bank *bank, uint32_t *si_id, uint8_t *protection, uint8_t *lifecycle);

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
 *        Macaw            | 0x109     | EA40-EA7F
 *        TraveoII B-H-16M | 0x114     | ED00-ED3F
 * @param silicon_id Silicon ID of the following format:
 *  [31:24]       | [23:16]       | [15:12]       | [11:8]        | [7:0]
 *  Silicon Id Hi | Silicon ID Lo | Major Rev. Id | Minor Rev. Id | Family Id Lo
 * @param die pointer to variable, will be populated with die enum identificator
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_identify_die_from_silicon_id(uint32_t silicon_id, enum mxs40_die *die);

/** ***********************************************************************************************
 * @brief Translates Protection status to openocd-friendly boolean value
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_protect_check(struct flash_bank *bank);

/** ***********************************************************************************************
 * @brief Dummy function, device does not support flash bank protection
 * @return ERROR_OK always
 *************************************************************************************************/
int mxs40_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last);

/** ***********************************************************************************************
 * @brief MXS40 patform_get_info Displays human-readable information about acquired device
 * @param bank current flash bank
 * @param buf pointer to buffer for human-readable text
 * @param buf_size size of the buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_get_info(struct flash_bank *bank, struct command_invocation *cmd);

/** ***********************************************************************************************
 * @brief Probes target device only if it hasn't been probed yet
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_auto_probe(struct flash_bank *bank);

/** ***********************************************************************************************
 * @brief Erases Supervisory Flash by programming it with zeros
 *
 * @param bank current flash bank
 * @param first first sector to erase
 * @param last last sector to erase
 * @return ERROR_OK, all errors are ignored (but reported)
 *************************************************************************************************/
int mxs40_erase_sflash(struct flash_bank *bank, int first, int last);

/** ***********************************************************************************************
 * @brief Erases single Row or Sector on target device
 * @param bank current flash bank
 * @param addr starting address of the flash row
 * @param erase_sector if true will erase sector, erases row otherwise
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_erase_row(struct flash_bank *bank, uint32_t addr, bool erase_sector);

/** ***********************************************************************************************
 * @brief Performs Erase operation using asynchronous flash algorithm
 * @param bank current flash bank
 * @param first first sector to erase
 * @param last last sector to erase
 * @param erase_builder_p pointer to erase_builder function
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_erase_with_algo(struct flash_bank *bank, int first, int last, erase_builder erase_builder_p);

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
							bool use_writerow, uint8_t data_size);

/** ***********************************************************************************************
 * @brief Programs single Flash Row
 * @param bank current flash bank
 * @param addr address of the flash row
 * @param buffer pointer to the buffer with data
 * @param use_writerow true if current flash bank belongs to Supervisory Flash
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_program_row(struct flash_bank *bank, uint32_t addr, const uint8_t *buffer, bool use_writerow);

/** ***********************************************************************************************
 * @brief Performs Program operation
 * @param bank current flash bank
 * @param buffer pointer to the buffer with data
 * @param offset starting offset in falsh bank
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_program(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count);

/** ***********************************************************************************************
 * @brief Performs Program operation using Flash Algorithm
 * @param bank current flash bank
 * @param buffer pointer to the buffer with data
 * @param offset starting offset in falsh bank
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int mxs40_program_with_algo(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count);

/** ***********************************************************************************************
 * @brief Performs Erase operation. Corrently not supported.
 *
 * @param bank current flash bank
 * @param first first sector to erase
 * @param last last sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int efuse_erase(struct flash_bank *bank, unsigned int first, unsigned int last);

/** ***********************************************************************************************
 * @brief Programs EFuse region
 * @param bank current flash bank
 * @param addr address of the flash row
 * @param buffer pointer to the buffer with program data
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int efuse_program_bank(struct flash_bank *bank,  const uint8_t *buffer, uint32_t offset, uint32_t count);

/** ***********************************************************************************************
 * @brief Performs Read operation of EFuse region.
 *
 * @param bank The bank to read
 * @param buffer The data bytes read
 * @param offset The offset into the chip to read
 * @param count The number of bits to read. Note: single efuse bit is represented by bye in a
 * programming file
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int efuse_read_bank(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count);

/** ***********************************************************************************************
 * @brief Probes the device and populates related data structures with target flash geometry data.
 * This is done in non-intrusive way, no SROM API calls are involved so GDB can safely attach to a
 * running target. Function assumes that size of Work Flash is 32kB (true for all current part numbers)
 *
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int efuse_probe(struct flash_bank *bank);

/** ***********************************************************************************************
 * @brief Deallocates private driver structures
 * @param bank - the bank being destroyed
 *************************************************************************************************/
void mxs40_free_driver_priv(struct flash_bank *bank);

/** ***********************************************************************************************
 * @brief Prepares T2G devices for executing SROM calls
 * @param bank - the flash bank
 *************************************************************************************************/
int traveo2_prepare(struct flash_bank *bank);

#endif //#ifndef MXS40_H
