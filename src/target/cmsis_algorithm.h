#if(0)

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "flash/nor/core.h"

enum cmsis_algo_operation {
	CMSIS_OP_ERASE = 1,
	CMSIS_OP_PROGRAM = 2,
	CMSIS_OP_VERIFY = 3,
};

struct cmsis_algorithm;

/** ***********************************************************************************************
 * @brief Parses optional arguments to 'flash bank' tcl command and determines if current bank has
 * CMSIS flash algorithm assigned. Function is intended to be used within FLASH_BANK_COMMAND_HANDLER.
 * Algorithm can be assigned to any flash bank by supplying three additional arguments to the
 * 'flash bank' command as follows:
 * flash bank <required arguments> cmsis_algorithm <elf filename> <stack size>
 * In case algorithm has been assigned, function will open the image, resolve required symbols,
 * locate and load FlashDevice structure and calculate total size of the image. All data will be
 * stored in newly allocated cmsis_algorithm structure. Pointer to this structure can lieve
 * somewhere in bank->driver_priv so each bank can have individual algorithm assigned.
 *
 * @param argc CMD_ARGC value
 * @param argv CMD_ARGV value
 * @param cmsis_algo will be populated with cmsis_algorithm poiner (or NULL if algorithm is not assigned)
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_open(int argc, const char **argv, struct cmsis_algorithm **algo_pp);

/** ***********************************************************************************************
 * @brief Releases resurces allocated by cmsis_algo_open
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_close(struct cmsis_algorithm **cmsis_algo);

/** ***********************************************************************************************
 * @brief Allocates working area for the algorithm, stack and breakpoint instruction and writes
 * contents of Flash Algorithm image to the target memory
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_load(struct cmsis_algorithm *cmsis_algo, struct target *target);

/** ***********************************************************************************************
 * @brief Releases resources allocated by cmsis_algo_load
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_unload(struct cmsis_algorithm *cmsis_algo, struct target *target);

/** ***********************************************************************************************
 * @brief Parses FlashDevice structure in algorithm image and returns total number of Flash sectors
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param num_sectors will be populated with total number of sectors
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_get_sector_num(struct cmsis_algorithm *cmsis_algo, int *num_sectors);

/** ***********************************************************************************************
 * @brief Parses FlashDevice structure in algorithm image and returns geometry of the given sector
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param sector_idx index of the sector
 * @param offset will be populated with the offset of the sector within flash bank
 * @param size will be populated with the size of the sector
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_get_sector_info(struct cmsis_algorithm *cmsis_algo, size_t sector_idx,
	uint32_t *sect_offset, uint32_t *sect_size);

/** ***********************************************************************************************
 * @brief Parses FlashDevice structure in algorithm image and returns information about flash bank
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param base will be populated with the base address of the flash bank
 * @param size will be populated with size of the flash bank
 * @param empty_val will be populated with empty value
 * @param block_size will be populated with page size
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_get_bank_info(struct cmsis_algorithm *cmsis_algo, uint32_t *base, uint32_t *size,
	uint8_t *empty_val, uint32_t *block_size);

/** ***********************************************************************************************
 * @brief Calls Init() function of the CMSIS-PACK FlashLoader
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param address base address of the next operation
 * @param clock clock speed for the target device
 * @param function type of next operation
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_init(struct cmsis_algorithm *cmsis_algo, struct target *target,
	uint32_t address, uint32_t clock, enum cmsis_algo_operation function);

/** ***********************************************************************************************
 * @brief Calls UnInit() function of the CMSIS-PACK FlashLoader
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param function type of next operation
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_uninit(struct cmsis_algorithm *cmsis_algo, struct target *target, uint32_t function);

/** ***********************************************************************************************
 * @brief Calls EraseSector() function of the CMSIS-PACK FlashLoader
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param addr address of the sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_erase_sector(struct cmsis_algorithm *cmsis_algo, struct target *target,
	uint32_t addr);

/** ***********************************************************************************************
 * @brief Calls ProgramPage() function of the CMSIS-PACK FlashLoader
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param addr address of the sector to erase
 * @param size size of the data to program
 * @param buf_addr adress of the buffer with data
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_program_page(struct cmsis_algorithm *cmsis_algo, struct target *target,
	uint32_t addr, uint32_t size, uint32_t buf_addr);

/** ***********************************************************************************************
 * @brief Calls BlankCheck() function of the CMSIS-PACK FlashLoader
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param addr address of the flash
 * @param size size of the area to check
 * @param pattern pattern to compare
 * @param is_blank will be set to true if the area content is equal to pattern, set to false otherwise
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_blank_check(struct cmsis_algorithm *cmsis_algo, struct target *target,
	uint32_t addr, uint32_t size, uint8_t pattern, bool *is_blank);

/** ***********************************************************************************************
 * @brief Calls EraseChip() function of the CMSIS-PACK FlashLoader
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_erase_chip(struct cmsis_algorithm *cmsis_algo, struct target *target);

/** ***********************************************************************************************
 * @brief Calls Verify() function of the CMSIS-PACK FlashLoader
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param addr address of the area to verify
 * @param size size of the data
 * @param buf_addr adress of the buffer with data
 * @param failed_addr will be set to (addr + size) on success, any other number is the address of failure
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_verify(struct cmsis_algorithm *cmsis_algo, struct target *target,
	uint32_t addr, uint32_t size, uint8_t buf_addr, uint32_t *failed_addr);

/** ***********************************************************************************************
 * @brief Convinience function, intended to be used within probe() function of the Flash Driver.
 * Populates flash_bank structure with values stored in FlashDevice structure of the Flash Algorithm
 * @param bank flash_bank structure to populate
 * @param algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param call_init if true, load algorithm and call Init(VERIFY) e.g. to map external flash to main memory space
 * @param page_size will be populated with szPage velue (may be NULL if not required)
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_probe(struct flash_bank *bank, struct cmsis_algorithm *algo, bool call_init,
	uint32_t *page_size);

/** ***********************************************************************************************
 * @brief Convinience function, intended to be used within erase() function of the Flash Driver.
 * Performs Erase operation for the range of sectors.
 * @param bank current flash bank
 * @param algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param first first sector to erase
 * @param last last sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_erase(struct flash_bank *bank, struct cmsis_algorithm *algo, int first, int last);
#endif
