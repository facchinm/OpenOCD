#if(0)

#include <stdarg.h>
#include "cmsis_algorithm.h"
#include "algorithm.h"
#include "binarybuffer.h"
#include "image.h"
#include "log.h"
#include "target/armv7m.h"


static struct symbol cmsis_symbols[] = {
	{"Init", 0},
	{"UnInit", 0},
	{"EraseSector", 0},
	{"ProgramPage", 0},
	{"BlankCheck", 0},
	{"EraseChip", 0},
	{"Verify", 0},
	{"FlashDevice", 0},
	{"PrgData", 0},
	{NULL, 0},
};

struct cmsis_flash_sectors {
	uint32_t size;
	uint32_t addr;
};

struct cmsis_flash_device {
	uint16_t vers;			/* Version Number and Architecture*/
	uint8_t dev_name[128];	/* Device Name and Description*/
	uint16_t dev_type;		/* Device Type: ONCHIP, EXT8BIT, EXT16BIT, ...*/
	uint32_t dev_addr;		/* Default Device Start Address*/
	uint32_t sz_dev;		/* Total Size of Device*/
	uint32_t sz_page;		/* Programming Page Size*/
	uint32_t _resvd;		/* Reserved for future Extension*/
	uint8_t val_empty;		/* Content of Erased Memory*/
	uint32_t timeout_prog;	/* Time Out of Program Page Function*/
	uint32_t timeout_erase;	/* Time Out of Erase Sector Function*/
	struct cmsis_flash_sectors sectors[512];
};

struct cmsis_algorithm {
	struct image image;
	struct cmsis_flash_device flash_dev;
	struct working_area *work_area;
	uint32_t footprint;
	uint32_t stack_size;
	uint32_t of_init;
	uint32_t of_uninit;
	uint32_t of_erase_sector;
	uint32_t of_program_page;
	uint32_t of_blank_check;
	uint32_t of_erase_chip;
	uint32_t of_verify;
	uint32_t of_flash_device;
	uint32_t of_prg_data;
};

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
int cmsis_algo_open(int argc, const char **argv, struct cmsis_algorithm **algo_pp)
{
	const char *algo_url = NULL;
	int hr;

	*algo_pp = NULL;
	uint32_t stack_size;

	/* Iterate over all arguments and try to locate 'cmsis_algorithm' */
	for (int i = 0; i < argc; i++) {
		if (strcmp(argv[i], "cmsis_algorithm") != 0)
			continue;

		/* Ensure 'cmsis_algorithm' is followed by at least two parameters */
		if (i > argc - 3) {
			LOG_ERROR("cmsis_algorithm: required parameters are missing "
				"(usage: cmsis_algorithm <elf filename> <stack size>)");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		/* Get algorithm url */
		algo_url = argv[i + 1];
		stack_size = strtol(argv[i + 2], NULL, 0);
		break;
	}

	/* Exit if this flash bank has no algorithm assigned */
	if (!algo_url)
		return ERROR_OK;

	if (!stack_size) {
		LOG_ERROR("cmsis_algorithm: invalid stack size");
		return ERROR_FAIL;
	}

	struct cmsis_algorithm *algo = calloc(1, sizeof(struct cmsis_algorithm));
	if (!algo) {
		LOG_ERROR("cmsis_algorithm: out of memory");
		return ERROR_FAIL;
	}

	algo->stack_size = stack_size;


	char *pkgdata_algo_url = alloc_printf("%s/%s", "..", algo_url);
	if(!pkgdata_algo_url) {
		free(algo);
		LOG_ERROR("cmsis_algorithm: out of memory");
		return ERROR_FAIL;
	}

	/* Open Flash Loader image */
	hr = image_open(&algo->image, pkgdata_algo_url, "elf");
	if (hr != ERROR_OK)
		goto free_algo;

	/* Initialize all symbol offsets to some invalid value */
	for (size_t i = 0; i < sizeof(cmsis_symbols) / sizeof(cmsis_symbols[0]); i++)
		cmsis_symbols[i].offset = UINT32_MAX;

	/* Resolve all required and optional symbols */
	hr = image_resolve_symbols(&algo->image, cmsis_symbols);
	if (hr != ERROR_OK)
		goto close_free_algo;

	algo->of_init = cmsis_symbols[0].offset;
	algo->of_uninit = cmsis_symbols[1].offset;
	algo->of_erase_sector= cmsis_symbols[2].offset;
	algo->of_program_page = cmsis_symbols[3].offset;
	algo->of_blank_check = cmsis_symbols[4].offset;
	algo->of_erase_chip = cmsis_symbols[5].offset;
	algo->of_verify = cmsis_symbols[6].offset;
	algo->of_flash_device = cmsis_symbols[7].offset;
	algo->of_prg_data = cmsis_symbols[8].offset;

	/* Ensure all required symbols are resolved */
	if (algo->of_init == UINT32_MAX ||
		algo->of_uninit == UINT32_MAX ||
		algo->of_erase_sector == UINT32_MAX ||
		algo->of_program_page == UINT32_MAX ||
		algo->of_flash_device == UINT32_MAX ||
		algo->of_prg_data == UINT32_MAX) {
		LOG_ERROR("cmsis_algorithm: One or more API functions "
			"required by CMSIS standard can not be found");
		hr = ERROR_IMAGE_FORMAT_ERROR;
		goto close_free_algo;
	}

	/* Locate elf section containing FlashDevice structure */
	for (int i = 0; i < algo->image.num_sections; i++) {
		target_addr_t base = algo->image.sections[i].base_address;
		size_t size = algo->image.sections[i].size;

		if (base <= algo->of_flash_device &&
			base + size >= algo->of_flash_device + sizeof(struct cmsis_flash_device) ) {

			/* Section with FlashDevice structure found, populate algo->flash_dev */
			uint8_t buffer[size];

			size_t read;
			hr = image_read_section(&algo->image, i, 0, size, buffer, &read);
			if (hr != ERROR_OK)
				goto close_free_algo;

			if (size != read) {
				hr = ERROR_FAIL;
				goto close_free_algo;
			}

			memcpy(&algo->flash_dev, &buffer[algo->of_flash_device - base],
				sizeof(struct cmsis_flash_device));
		}
	}

	/* Calculate total footprint of the elf image */
	algo->footprint = 0;
	for (int i = 0; i < algo->image.num_sections; i++) {
		struct imagesection *section = &algo->image.sections[i];

		/* Try to skip 'DevDscr' section, usually it is not required */
		if (section->base_address == algo->of_flash_device &&
			section->size == sizeof(struct cmsis_flash_device))
			continue;

		if (section->base_address + section->size > algo->footprint)
			algo->footprint = section->base_address + section->size;
	}

	*algo_pp = algo;
	free(pkgdata_algo_url);
	return ERROR_OK;

close_free_algo:
	image_close(&algo->image);

free_algo:
	free(pkgdata_algo_url);
	free(algo);
	return hr;
}

/** ***********************************************************************************************
 * @brief Releases resurces allocated by cmsis_algo_open
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_close(struct cmsis_algorithm **cmsis_algo)
{
	if (*cmsis_algo) {
		image_close(&(*cmsis_algo)->image);
		free(*cmsis_algo);
		*cmsis_algo = NULL;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Allocates working area for the algorithm, stack and breakpoint instruction and writes
 * contents of Flash Algorithm image to the target memory
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_load(struct cmsis_algorithm *cmsis_algo, struct target *target)
{
	int hr;
	const uint32_t wa_size = (cmsis_algo->footprint + cmsis_algo->stack_size + 0x03u) & ~0x03u;

	hr = target_alloc_working_area(target, wa_size, &cmsis_algo->work_area);
	if (hr != ERROR_OK)
		return hr;

	for (int i = 0; i < cmsis_algo->image.num_sections; i++) {
		struct imagesection *section = &cmsis_algo->image.sections[i];

		/* Try to skip 'DevDscr' section, usually it is not required */
		if (section->base_address == cmsis_algo->of_flash_device &&
			section->size == sizeof(struct cmsis_flash_device))
			continue;

		uint8_t buffer[section->size];
		size_t read;

		hr = image_read_section(&cmsis_algo->image, i, 0, section->size, buffer, &read);
		if (hr != ERROR_OK)
			return hr;

		if (section->size != read)
			return ERROR_FAIL;

		const target_addr_t tgt_buf = cmsis_algo->work_area->address +
			section->base_address;

		hr = target_write_buffer(target, tgt_buf, section->size, buffer);
		if (hr != ERROR_OK)
			return hr;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Releases resources allocated by cmsis_algo_load
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_unload(struct cmsis_algorithm *cmsis_algo, struct target *target)
{
	int hr = ERROR_OK;
	if (cmsis_algo->work_area) {
		hr = target_free_working_area(target, cmsis_algo->work_area);
		cmsis_algo->work_area = NULL;
	}
	return hr;
}

/** ***********************************************************************************************
 * @brief Parses FlashDevice structure in algorithm image and returns total number of Flash sectors
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param num_sectors will be populated with total number of sectors
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_get_sector_num(struct cmsis_algorithm *cmsis_algo, int *num_sectors)
{
	assert(cmsis_algo && num_sectors);

	*num_sectors = 0;
	size_t idx = 0;
	const struct cmsis_flash_sectors *sect = cmsis_algo->flash_dev.sectors;
	const struct cmsis_flash_device *dev = &cmsis_algo->flash_dev;

	while (sect[idx + 1].addr != UINT32_MAX) {
		if (idx >= 512 || sect[idx].size == 0) {
			LOG_ERROR("cmsis_algorithm: FlashDevice structure is invalid");
			return ERROR_IMAGE_FORMAT_ERROR;
		}

		*num_sectors += (sect[idx + 1].addr - sect[idx].addr) / sect[idx].size;
		idx++;
	}

	if (sect[idx].size == 0) {
		LOG_ERROR("cmsis_algorithm: FlashDevice structure is invalid");
		return ERROR_IMAGE_FORMAT_ERROR;
	}

	*num_sectors += (dev->sz_dev - sect[idx].addr) / sect[idx].size;
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Parses FlashDevice structure in algorithm image and returns geometry of the given sector
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param sector_idx index of the sector
 * @param offset will be populated with the offset of the sector within flash bank
 * @param size will be populated with the size of the sector
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_get_sector_info(struct cmsis_algorithm *cmsis_algo, size_t sector_idx,
	uint32_t *sect_offset, uint32_t *sect_size)
{
	assert(cmsis_algo && sect_offset && sect_size);

	const struct cmsis_flash_sectors *sect = cmsis_algo->flash_dev.sectors;
	uint32_t size = sect[0].size;
	uint32_t offset = 0;
	size_t fd_sec_idx = 0;

	for (size_t i = 0; i < sector_idx; i++) {

		offset += sect[fd_sec_idx].size;

		if (offset == sect[fd_sec_idx + 1].addr)
			fd_sec_idx++;

		size = sect[fd_sec_idx].size;

		if (sect[fd_sec_idx].size == UINT32_MAX || sect[fd_sec_idx].addr == UINT32_MAX ||
			offset + size > cmsis_algo->flash_dev.sz_dev) {
			LOG_ERROR("cmsis_algorithm: FlashDevice structure is invalid");
			return ERROR_IMAGE_FORMAT_ERROR;
		}
	}

	*sect_offset = offset;
	*sect_size = size;
	return ERROR_OK;
}

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
	uint8_t *empty_val, uint32_t *block_size)
{
	assert(cmsis_algo && base && size && empty_val && block_size);

	*base = cmsis_algo->flash_dev.dev_addr;
	*size = cmsis_algo->flash_dev.sz_dev;
	*empty_val = cmsis_algo->flash_dev.val_empty;
	*block_size = cmsis_algo->flash_dev.sz_page;

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Executes FlashLoader functions. Function takes variable number of arguments and stores
 * them in r0...r3 registers, address to PrgData section is stored in r9, SP is set to the end of
 * working area for the algorithm. Function also writes BKPT #0 instruction right after algorithm
 * code space and stores address of this instruction in lr.
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param fn_offset offset of the entry point of the algorithm (relative to the working area address)
 * @param timeout execution timeout, in ms
 * @param result value is populated with exit code of the algorithm (value in r0 register)
 * @param argc number of arguments followed by variable number of arguments (values for r0...r3)
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_algo_execute(struct cmsis_algorithm *cmsis_algo, struct target *target,
	uint32_t fn_offset, size_t timeout, uint32_t *result,
	size_t argc, ...)
{
	assert(argc <= 4);

	/* TODO: only ARMv6M/v7M are currently supported. I have no experience with other ARM architectures
	 * so I can not make this code completely generic. This will be fixed in the future */
	if (!is_armv7m(target_to_armv7m(target))) {
		LOG_ERROR("cmsis algorithm: only armv7m targets are supported");
		return ERROR_TARGET_INVALID;
	}

	static char *param_regs[4] = { "r0", "r1", "r2", "r3" };
	struct reg_param regs[7];

	struct armv7m_algorithm armv7m;
	armv7m.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m.core_mode = ARM_MODE_THREAD;
	const struct working_area *wa = cmsis_algo->work_area;
	int hr;

	size_t param_cnt = 0;
	va_list args;

	va_start(args, argc);
	if (argc) {
		/* If number of arguments is greater than zero r0 should be PARAM_IN_OUT */
		for (size_t i = 0; i < argc; i++) {
			/* loop through arguments and populate r0...r3 registers */
			enum param_direction dir = (i == 0) ? PARAM_IN_OUT : PARAM_OUT;
			init_reg_param(&regs[i], param_regs[i], 32, dir);
			buf_set_u32(regs[i].value, 0, 32,  va_arg(args, uint32_t));
			param_cnt++;
		}
	} else {
		/* No arguments have been supplied, r0 is PARAM_IN in this case */
		init_reg_param(&regs[0], param_regs[0], 32, PARAM_IN);
		param_cnt++;
	}
	va_end(args);

	/* Calculate address of the 'BKPT #0' instruction */
	uint32_t bkpt_adddr = (wa->address + cmsis_algo->footprint + 0x03u) & ~0x03u;

	/* Populate all common registers */
	init_reg_param(&regs[param_cnt + 0], "r9", 32, PARAM_OUT);
	init_reg_param(&regs[param_cnt + 1], "sp", 32, PARAM_OUT);
	init_reg_param(&regs[param_cnt + 2], "lr", 32, PARAM_OUT);

	buf_set_u32(regs[param_cnt + 0].value, 0, 32, wa->address + cmsis_algo->of_prg_data);
	buf_set_u32(regs[param_cnt + 1].value, 0, 32, wa->address + wa->size);
	buf_set_u32(regs[param_cnt + 2].value, 0, 32, bkpt_adddr | 0x01);

	param_cnt += 3;

	/* Write 'BKPT #0' instruction */
	hr = target_write_u32(target, bkpt_adddr, 0xBE00BE00);
	if (hr != ERROR_OK)
		goto cleanup;

	/* Start the algorithm */
	hr = target_run_algorithm(target, 0, NULL, param_cnt, regs,
			wa->address + fn_offset, 0, timeout, &armv7m);
	if (hr != ERROR_OK)
		goto cleanup;

	/* Retrieve algorithm exit code */
	*result = buf_get_u32(regs[0].value, 0, 32);

cleanup:
	for (size_t i = 0; i < param_cnt; i++)
		destroy_reg_param(&regs[i]);

	return hr;
}

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
	uint32_t address, uint32_t clock, enum cmsis_algo_operation function)
{
	uint32_t result = 0;
	int hr = cmsis_algo_execute(cmsis_algo, target, cmsis_algo->of_init, 1000, &result,
			3, address, clock, function);

	if (hr != ERROR_OK)
		return hr;

	if (result) {
		LOG_ERROR("cmsis algorithm: Init operation failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Calls UnInit() function of the CMSIS-PACK FlashLoader
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param function type of next operation
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_uninit(struct cmsis_algorithm *cmsis_algo, struct target *target, uint32_t function)
{
	uint32_t result = 0;
	int hr = cmsis_algo_execute(cmsis_algo, target, cmsis_algo->of_uninit, 1000, &result,
			1, function);

	if (hr != ERROR_OK)
		return hr;

	if (result) {
		LOG_ERROR("cmsis algorithm: UnInit operation failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Calls EraseSector() function of the CMSIS-PACK FlashLoader
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param addr address of the sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_erase_sector(struct cmsis_algorithm *cmsis_algo, struct target *target,
	uint32_t addr)
{
	uint32_t result = 0;
	int hr = cmsis_algo_execute(cmsis_algo, target, cmsis_algo->of_erase_sector,
			cmsis_algo->flash_dev.timeout_erase, &result,
			1, addr);

	if (hr != ERROR_OK)
		return hr;

	if (result) {
		LOG_ERROR("cmsis algorithm: EraseSector operation failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

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
	uint32_t addr, uint32_t size, uint32_t buf_addr)
{
	uint32_t result = 0;
	int hr = cmsis_algo_execute(cmsis_algo, target, cmsis_algo->of_program_page,
			cmsis_algo->flash_dev.timeout_prog, &result,
			3, addr, size, buf_addr);

	if (hr != ERROR_OK)
		return hr;

	if (result) {
		LOG_ERROR("cmsis algorithm: ProgramPage operation failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

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
	uint32_t addr, uint32_t size, uint8_t pattern, bool *is_blank)
{
	if (cmsis_algo->of_blank_check != UINT32_MAX) {
		uint32_t result = 0;
		int hr = cmsis_algo_execute(cmsis_algo, target, cmsis_algo->of_blank_check,
				1000, &result,
				3, addr, size, pattern);

		if (hr != ERROR_OK)
			return hr;

		*is_blank = (result == 0);
		return ERROR_OK;
	} else
		return ERROR_COMMAND_NOTFOUND;
}

/** ***********************************************************************************************
 * @brief Calls EraseChip() function of the CMSIS-PACK FlashLoader
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_erase_chip(struct cmsis_algorithm *cmsis_algo, struct target *target)
{
	if (cmsis_algo->of_erase_chip != UINT32_MAX) {
		uint32_t result = 0;
		int hr = cmsis_algo_execute(cmsis_algo, target, cmsis_algo->of_erase_chip,
				cmsis_algo->flash_dev.timeout_erase, &result, 0);

		if (hr != ERROR_OK)
			return hr;

		if (result) {
			LOG_ERROR("cmsis algorithm: EraseChip operation failed");
			return ERROR_FAIL;
		}

		return ERROR_OK;
	} else
		return ERROR_COMMAND_NOTFOUND;
}

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
	uint32_t addr, uint32_t size, uint8_t buf_addr, uint32_t *failed_addr)
{
	if (cmsis_algo->of_verify != UINT32_MAX) {
		uint32_t result = 0;
		int hr = cmsis_algo_execute(cmsis_algo, target, cmsis_algo->of_verify,
				1000, &result,
				3, addr, size, buf_addr);

		if (hr != ERROR_OK)
			return hr;

		*failed_addr = result;
		return ERROR_OK;
	} else
		return ERROR_COMMAND_NOTFOUND;
}

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
	uint32_t *page_size)
{
	int hr;
	uint32_t base, size;

	hr = cmsis_algo_get_bank_info(algo, &base, &size, &bank->erased_value, page_size);
	if (hr != ERROR_OK)
		return hr;

	if (!bank->base)
		bank->base = base;

	if (!bank->size)
		bank->size = size;

	hr = cmsis_algo_get_sector_num(algo, &bank->num_sectors);
	if (hr != ERROR_OK)
		return hr;

	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	int real_sec_num = 0;
	for (int i = 0; i < bank->num_sectors; i++) {
		hr = cmsis_algo_get_sector_info(algo, i, &bank->sectors[i].offset,
				&bank->sectors[i].size);

		if (hr != ERROR_OK)
			return hr;

		if (bank->sectors[i].offset >= bank->size)
			break;

		real_sec_num++;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	if (bank->num_sectors != real_sec_num) {
		bank->num_sectors = real_sec_num;
		bank->sectors = realloc(bank->sectors, real_sec_num * sizeof(struct flash_sector));
	}

	bank->write_start_alignment = algo->flash_dev.sz_page;
	bank->write_end_alignment = algo->flash_dev.sz_page;
	bank->minimal_write_gap = algo->flash_dev.sz_page;
	bank->erased_value = algo->flash_dev.val_empty;
	bank->default_padded_value = algo->flash_dev.val_empty;

	if (call_init) {
		hr = cmsis_algo_load(algo, bank->target);
		if (hr != ERROR_OK)
			return hr;

		hr = cmsis_algo_init(algo, bank->target, bank->base, 0, CMSIS_OP_VERIFY);
		if (hr != ERROR_OK)
			return hr;

		hr = cmsis_algo_unload(algo, bank->target);
	}

	return hr;
}

/** ***********************************************************************************************
 * @brief Convinience function, intended to be used within erase() function of the Flash Driver.
 * Performs Erase operation for the range of sectors.
 * @param bank current flash bank
 * @param algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param first first sector to erase
 * @param last last sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
int cmsis_algo_erase(struct flash_bank *bank, struct cmsis_algorithm *algo, int first, int last)
{
	struct target *target = bank->target;
	int hr;

	hr = cmsis_algo_load(algo, target);
	if (hr != ERROR_OK)
		return hr;

	hr = cmsis_algo_init(algo, target, bank->base, 0, CMSIS_OP_ERASE);
	if (hr != ERROR_OK)
		return hr;

	for (int i = first; i <= last; i++) {
		LOG_INFO("Erasing SECTOR @%08X", bank->base + bank->sectors[i].offset);

		hr = cmsis_algo_erase_sector(algo, target,
				bank->base + bank->sectors[i].offset);

		if (hr != ERROR_OK)
			return hr;
	}

	hr = cmsis_algo_uninit(algo, target, CMSIS_OP_ERASE);
	if (hr != ERROR_OK)
		return hr;

	hr = cmsis_algo_init(algo, target, bank->base, 0, CMSIS_OP_VERIFY);
	if (hr != ERROR_OK)
		return hr;

	hr = cmsis_algo_unload(algo, target);
	return hr;
}
#endif
