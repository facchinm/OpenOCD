// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2026                                                   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/time_support.h>

#define PIC32CK_FLASH_PFM_BASE		0x0C000000u

#define PIC32CK_DSU_BASE		0x44000000u
#define PIC32CK_DSU_DID		(PIC32CK_DSU_BASE + 0x120u)

#define PIC32CK_FCW_BASE		0x44004000u
#define PIC32CK_FCW_CTRLA		(PIC32CK_FCW_BASE + 0x00u)
#define PIC32CK_FCW_INTFLAG		(PIC32CK_FCW_BASE + 0x14u)
#define PIC32CK_FCW_STATUS		(PIC32CK_FCW_BASE + 0x18u)
#define PIC32CK_FCW_KEY			(PIC32CK_FCW_BASE + 0x1Cu)
#define PIC32CK_FCW_ADDR		(PIC32CK_FCW_BASE + 0x20u)
#define PIC32CK_FCW_SRCADDR		(PIC32CK_FCW_BASE + 0x24u)

#define PIC32CK_FCW_STATUS_BUSY		(1u << 0)

#define PIC32CK_FCW_INTFLAG_DONE	(1u << 0)
#define PIC32CK_FCW_INTFLAG_WPERR	(1u << 5)
#define PIC32CK_FCW_INTFLAG_ERR_MASK	0x000031FEu

#define PIC32CK_FCW_CTRLA_NVMOP_MASK	0xFu
#define PIC32CK_FCW_CTRLA_PREPG	(1u << 7)

#define PIC32CK_FCW_OP_NOP		0x0u
#define PIC32CK_FCW_OP_ROW_PROGRAM	0x3u
#define PIC32CK_FCW_OP_PAGE_ERASE	0x4u

#define PIC32CK_FCW_UNLOCK_WRKEY	0x91C32C01u

#define PIC32CK_PAGE_SIZE		4096u
#define PIC32CK_ROW_SIZE		1024u

struct pic32ck_part {
	uint32_t did;
	const char *name;
	uint32_t flash_kb;
};

static const struct pic32ck_part pic32ck_parts[] = {
	{ 0x09537053u, "PIC32CK0512xG00064", 512 },
	{ 0x09538053u, "PIC32CK0512xG00100", 512 },
	{ 0x09532053u, "PIC32CK0512xG01100", 512 },
	{ 0x0951F053u, "PIC32CK1025xG00064", 1024 },
	{ 0x09520053u, "PIC32CK1025xG00100", 1024 },
	{ 0x09519053u, "PIC32CK1025xG01064", 1024 },
	{ 0x0951A053u, "PIC32CK1025xG01100", 1024 },
	{ 0x0951C053u, "PIC32CK1025SG01064", 1024 },
	{ 0x0951D053u, "PIC32CK1025SG01100", 1024 },
	{ 0x09523053u, "PIC32CK1025SG00100", 1024 },
	{ 0x0950A053u, "PIC32CK2051SG00064", 2048 },
	{ 0x0950B053u, "PIC32CK2051SG00100", 2048 },
	{ 0x0950C053u, "PIC32CK2051SG00144", 2048 },
	{ 0x09507053u, "PIC32CK2051GC00064", 2048 },
	{ 0x09509053u, "PIC32CK2051GC00144", 2048 },
	{ 0x09503053u, "PIC32CK2051GC01144", 2048 },
	{ 0x09506053u, "PIC32CK2051xG01", 2048 },
};

struct pic32ck_flash_bank {
	bool probed;
	uint32_t did;
	const char *part_name;
	uint32_t page_size;
	uint32_t row_size;
};

static const struct pic32ck_part *pic32ck_find_part(uint32_t did)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(pic32ck_parts); i++) {
		if (pic32ck_parts[i].did == did)
			return &pic32ck_parts[i];
	}

	return NULL;
}

static uint32_t pic32ck_guess_flash_kb(uint32_t did)
{
	/* On PIC32CK DID, series field distinguishes 0512/1025/2051 densities. */
	uint32_t series = (did >> 16) & 0x3Fu;

	switch (series) {
	case 0x13:
		return 512;
	case 0x11:
	case 0x12:
		return 1024;
	case 0x10:
		return 2048;
	default:
		return 0;
	}
}

static int pic32ck_wait_ready(struct target *target, int timeout_ms)
{
	int64_t start = timeval_ms();
	uint32_t status;

	do {
		int res = target_read_u32(target, PIC32CK_FCW_STATUS, &status);
		if (res != ERROR_OK)
			return res;

		if ((status & PIC32CK_FCW_STATUS_BUSY) == 0)
			return ERROR_OK;

		alive_sleep(1);
		keep_alive();
	} while ((timeval_ms() - start) < timeout_ms);

	LOG_ERROR("PIC32CK FCW timeout waiting for BUSY to clear");
	return ERROR_FLASH_OPERATION_FAILED;
}

static int pic32ck_exec_nvmop(struct target *target, uint32_t address, uint32_t op)
{
	int res;
	uint32_t intflag;

	res = target_write_u32(target, PIC32CK_FCW_INTFLAG, 0xFFFFFFFFu);
	if (res != ERROR_OK)
		return res;

	res = target_write_u32(target, PIC32CK_FCW_ADDR, address);
	if (res != ERROR_OK)
		return res;

	res = target_write_u32(target, PIC32CK_FCW_KEY, PIC32CK_FCW_UNLOCK_WRKEY);
	if (res != ERROR_OK)
		return res;

	res = target_write_u32(target, PIC32CK_FCW_CTRLA,
			PIC32CK_FCW_CTRLA_PREPG | (op & PIC32CK_FCW_CTRLA_NVMOP_MASK));
	if (res != ERROR_OK)
		return res;

	res = pic32ck_wait_ready(target, 1000);
	if (res != ERROR_OK)
		return res;

	res = target_read_u32(target, PIC32CK_FCW_INTFLAG, &intflag);
	if (res != ERROR_OK)
		return res;

	if (intflag & PIC32CK_FCW_INTFLAG_ERR_MASK) {
		LOG_ERROR("PIC32CK FCW operation failed, INTFLAG=0x%08" PRIx32, intflag);
		res = (intflag & PIC32CK_FCW_INTFLAG_WPERR) ? ERROR_FLASH_PROTECTED :
			ERROR_FLASH_OPERATION_FAILED;
	}

	/* write-one-to-clear */
	(void)target_write_u32(target, PIC32CK_FCW_INTFLAG, intflag);

	return res;
}

FLASH_BANK_COMMAND_HANDLER(pic32ck_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pic32ck_flash_bank *chip = calloc(1, sizeof(*chip));
	if (!chip)
		return ERROR_FAIL;

	chip->part_name = "unknown";
	chip->page_size = PIC32CK_PAGE_SIZE;
	chip->row_size = PIC32CK_ROW_SIZE;

	bank->driver_priv = chip;

	return ERROR_OK;
}

static int pic32ck_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct pic32ck_flash_bank *chip = bank->driver_priv;
	const struct pic32ck_part *part;
	uint32_t flash_kb;
	int res;

	if (chip->probed)
		return ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	res = target_read_u32(target, PIC32CK_DSU_DID, &chip->did);
	if (res != ERROR_OK) {
		LOG_ERROR("Unable to read PIC32CK DSU DID");
		return res;
	}

	part = pic32ck_find_part(chip->did);
	if (part) {
		chip->part_name = part->name;
		flash_kb = part->flash_kb;
	} else {
		flash_kb = pic32ck_guess_flash_kb(chip->did);
		if (flash_kb == 0) {
			LOG_ERROR("Unknown PIC32CK DID 0x%08" PRIx32, chip->did);
			return ERROR_FLASH_BANK_NOT_PROBED;
		}
		LOG_WARNING("Unknown PIC32CK DID 0x%08" PRIx32 ", using inferred flash size %" PRIu32 "KB",
			chip->did, flash_kb);
	}

	bank->size = flash_kb * 1024u;
	bank->num_sectors = bank->size / chip->page_size;

	free(bank->sectors);
	bank->sectors = alloc_block_array(0, chip->page_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* FCW exposes several protection mechanisms; unsupported for now. */
	bank->num_prot_blocks = 0;
	free(bank->prot_blocks);
	bank->prot_blocks = NULL;

	chip->probed = true;

	if (bank->base != PIC32CK_FLASH_PFM_BASE) {
		LOG_WARNING("PIC32CK PFM is typically at 0x%08" PRIx32 ", configured bank base is 0x%08" PRIx32,
			PIC32CK_FLASH_PFM_BASE, (uint32_t)bank->base);
	}

	LOG_INFO("PIC32CK: %s DID=0x%08" PRIx32 " flash=%" PRIu32 "KB",
		chip->part_name, chip->did, flash_kb);

	return ERROR_OK;
}

static int pic32ck_auto_probe(struct flash_bank *bank)
{
	struct pic32ck_flash_bank *chip = bank->driver_priv;

	if (chip->probed)
		return ERROR_OK;

	return pic32ck_probe(bank);
}

static int pic32ck_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct pic32ck_flash_bank *chip = bank->driver_priv;
	struct target *target = bank->target;
	int res;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (!chip->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	for (unsigned int s = first; s <= last; s++) {
		uint32_t address = bank->base + bank->sectors[s].offset;
		res = pic32ck_exec_nvmop(target, address, PIC32CK_FCW_OP_PAGE_ERASE);
		if (res != ERROR_OK)
			return res;
	}

	return ERROR_OK;
}

static int pic32ck_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct pic32ck_flash_bank *chip = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *source = NULL;
	uint8_t row_buf[PIC32CK_ROW_SIZE];
	int res;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (!chip->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	res = target_alloc_working_area(target, chip->row_size, &source);
	if (res != ERROR_OK) {
		LOG_ERROR("No working area available for PIC32CK row programming");
		return res;
	}

	while (count) {
		uint32_t abs_addr = bank->base + offset;
		uint32_t row_base = abs_addr & ~(chip->row_size - 1u);
		uint32_t row_off = abs_addr - row_base;
		uint32_t n = chip->row_size - row_off;

		if (n > count)
			n = count;

		/* FCW row programming writes an entire row from SRCADDR. */
		memset(row_buf, 0xFF, sizeof(row_buf));
		memcpy(row_buf + row_off, buffer, n);

		res = target_write_memory(target, source->address, 4,
				chip->row_size / 4u, row_buf);
		if (res != ERROR_OK)
			goto out;

		res = target_write_u32(target, PIC32CK_FCW_SRCADDR, source->address);
		if (res != ERROR_OK)
			goto out;

		res = pic32ck_exec_nvmop(target, row_base, PIC32CK_FCW_OP_ROW_PROGRAM);
		if (res != ERROR_OK)
			goto out;

		offset += n;
		buffer += n;
		count -= n;
	}

out:
	target_free_working_area(target, source);
	return res;
}

static int pic32ck_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	(void)bank;
	(void)set;
	(void)first;
	(void)last;

	return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int pic32ck_protect_check(struct flash_bank *bank)
{
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = 0;

	return ERROR_OK;
}

static int pic32ck_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct pic32ck_flash_bank *chip = bank->driver_priv;

	if (!chip->probed) {
		command_print_sameline(cmd, "PIC32CK flash bank not probed");
		return ERROR_OK;
	}

	command_print_sameline(cmd,
		"PIC32CK part %s DID=0x%08" PRIx32 ", flash %" PRIu32 "KB, page %" PRIu32 "B, row %" PRIu32 "B",
		chip->part_name, chip->did, bank->size / 1024u, chip->page_size, chip->row_size);

	return ERROR_OK;
}

const struct flash_driver pic32ck_flash = {
	.name = "pic32ck",
	.flash_bank_command = pic32ck_flash_bank_command,
	.erase = pic32ck_erase,
	.protect = pic32ck_protect,
	.write = pic32ck_write,
	.read = default_flash_read,
	.probe = pic32ck_probe,
	.auto_probe = pic32ck_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = pic32ck_protect_check,
	.info = pic32ck_info,
	.free_driver_priv = default_flash_free_driver_priv,
};