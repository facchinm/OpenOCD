// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022-2025, Infineon Technologies AG, or an affiliate of *
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

#include "flash/nor/imp.h"
#include "target/target.h"
#include "flash/progress.h"

struct dummy_info {
	unsigned int block_size;
	bool probed;
};

FLASH_BANK_COMMAND_HANDLER(dummy_flashbank_command)
{
	unsigned int block_size = 4096;

	if (CMD_ARGC < 6 || CMD_ARGC > 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 7)
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[6], block_size);

	if (bank->size % block_size) {
		LOG_ERROR("Dummy bank size is not a multiple of block size");
		return ERROR_FAIL;
	}

	struct dummy_info *info = calloc(1, sizeof(struct dummy_info));
	if (!info)
		return ERROR_FAIL;

	info->block_size = block_size;
	bank->driver_priv = info;

	return ERROR_OK;
}

static int dummy_flash_probe(struct flash_bank *bank)
{
	struct dummy_info *info = (struct dummy_info *)bank->driver_priv;

	if (bank->sectors)
		free (bank->sectors);

	bank->erased_value = bank->default_padded_value = 0;
	bank->num_sectors = bank->size / info->block_size;
	bank->sectors = alloc_block_array(0, info->block_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	info->probed = true;
	return ERROR_OK;
}

static int dummy_flash_auto_probe(struct flash_bank *bank)
{
	struct dummy_info *info = (struct dummy_info *)bank->driver_priv;
	if (info->probed)
		return ERROR_OK;

	return dummy_flash_probe(bank);
}

static int dummy_flash_erase(struct flash_bank *bank, unsigned int first,
	unsigned int last)
{
	struct dummy_info *info = (struct dummy_info *)bank->driver_priv;
	int hr = ERROR_OK;

	uint8_t *zeros = calloc(1, info->block_size);
	if (!zeros)
		return ERROR_FAIL;

	progress_init(last - first + 1, ERASING);
	for (unsigned s = first; s <= last; s++) {
		const uint32_t address = bank->base + bank->sectors[s].offset;
		hr = target_write_buffer(bank->target, address, info->block_size, zeros);
		if (hr != ERROR_OK)
			goto exit;
		progress_sofar(s - first);
	}

exit:
	progress_done(hr);
	free(zeros);
	return hr;
}

int dummy_flash_program(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct dummy_info *info = (struct dummy_info *)bank->driver_priv;
	int hr = ERROR_OK;

	progress_init(count, PROGRAMMING);
	uint32_t address = bank->base + offset;
	while (count) {
		hr = target_write_buffer(bank->target, address, info->block_size, buffer);
		if (hr != ERROR_OK)
			goto exit;
		progress_left(count);

		buffer += info->block_size;
		address += info->block_size;
		count -= info->block_size;
	}

exit:
	progress_done(hr);
	return hr;
}

int dummy_flash_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	(void)bank; (void)set; (void)first; (void)last;

	LOG_WARNING("Dummy flash does not support flash bank protection");
	return ERROR_OK;
}

int dummy_flash_protect_check(struct flash_bank *bank)
{
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = false;

	return ERROR_OK;
}

const struct flash_driver dummy_flash = {
	.name = "dummy_flash",
	.usage = "flash bank <name> dummy_flash <base> <size> 0 0 <target#> [page_size = 4096]",
	.commands = NULL,
	.flash_bank_command = dummy_flashbank_command,
	.erase = dummy_flash_erase,
	.protect = dummy_flash_protect,
	.write = dummy_flash_program,
	.read = default_flash_read,
	.probe = dummy_flash_probe,
	.auto_probe = dummy_flash_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = dummy_flash_protect_check,
	.info = NULL,
	.free_driver_priv = default_flash_free_driver_priv,
};
