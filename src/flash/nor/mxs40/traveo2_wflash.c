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

#include "flash/nor/mxs40/mxs40.h"
#include "flash/nor/imp.h"
#include "flash/progress.h"
#include "helper/binarybuffer.h"
#include "target/image.h"
#include "target/target.h"

struct wflash_word {
	bool is_valid;
	uint32_t data;
};

static int wflash_read_sector(struct flash_bank *bank, struct flash_sector *sector, struct wflash_word *words)
{
	struct target *target = bank->target;
	struct working_area *wa;

	uint8_t srom_params[3u * sizeof(uint32_t)];
	int hr = target_alloc_working_area(target, sizeof(srom_params), &wa);
	if (hr != ERROR_OK)
		return hr;

	hr = mxs40_sromalgo_prepare(bank);
	if (hr != ERROR_OK)
		goto exit_free_wa;

	const uint32_t sector_start_addr = bank->base + sector->offset;
	const uint32_t sector_end_addr = sector_start_addr + sector->size;
	uint32_t check_addr = sector_start_addr;

	while (check_addr < sector_end_addr) {
		buf_set_u32(srom_params + 0x00u, 0u, 32u, SROMAPI_BLANK_CHECK_REQ);
		buf_set_u32(srom_params + 0x04u, 0u, 32u, check_addr);
		buf_set_u32(srom_params + 0x08u, 0u, 32u, (sector_end_addr - check_addr) / 4u - 1u);

		hr = target_write_buffer(target, wa->address, sizeof(srom_params), srom_params);
		if (hr != ERROR_OK)
			goto exit_release_algo;

		uint32_t data_out;
		hr = mxs40_call_sromapi_inner(bank, SROMAPI_BLANK_CHECK_REQ, wa->address, false, &data_out);
		if (hr != ERROR_OK)
			goto exit_release_algo;

		bool is_valid = (data_out & SROMAPI_STATUS_MSK) != SROMAPI_STAT_SUCCESS;
		if (is_valid) {
			const uint32_t failed_word_num = (data_out & 0x00FFFF00) >> 8u;
			const uint32_t word_addr = check_addr + failed_word_num * sizeof(uint32_t);
			const uint32_t word_idx = ((check_addr - sector_start_addr) / sizeof(uint32_t)) + failed_word_num;

			assert(word_idx < sector->size / 4u);
			words[word_idx].is_valid = true;
			hr = target_read_u32(target, word_addr, &words[word_idx].data);
			if (hr != ERROR_OK)
				goto exit_release_algo;

			LOG_DEBUG("Valid word @0x%08X (word index in sector: %d), value 0x%08X", word_addr, word_idx, words[word_idx].data);
			check_addr += failed_word_num * 4u + 4u;
			continue;
		}

		break;
	}

exit_release_algo:
	mxs40_sromalgo_release(target);

exit_free_wa:
	target_free_working_area(target, wa);
	return hr;
}

static void wflash_validate_image(struct flash_bank *bank, struct image *image)
{
	const target_addr_t bank_start = bank->base;
	const target_addr_t bank_end = bank_start + bank->size;

	for (unsigned int i = 0u; i < image->num_sections; i++) {
		const target_addr_t sec_start = image->sections[i].base_address;
		const target_addr_t sec_end = sec_start + image->sections[i].size;

		target_addr_t real_start = (sec_start + 3u) & ~3ull;
		target_addr_t real_end = sec_end & ~3ull;

		if (real_start < bank_start)
			real_start = bank_start;

		if (real_end > bank_end)
			real_end = bank_end;

		if (real_end < bank_start || real_start >= bank_end || real_start == real_end) {
			LOG_WARNING("Section [" TARGET_ADDR_FMT  ", " TARGET_ADDR_FMT ") will be skipped",
					  sec_start, sec_end);
			continue;
		}

		if (sec_start != real_start || sec_end != real_end) {
			LOG_WARNING("Section [" TARGET_ADDR_FMT  ", " TARGET_ADDR_FMT ") will be truncated to ["
					  TARGET_ADDR_FMT  ", " TARGET_ADDR_FMT ")", sec_start, sec_end, real_start, real_end);
			continue;
		}

		LOG_DEBUG("Section [" TARGET_ADDR_FMT  ", " TARGET_ADDR_FMT ") will be programmed",
					sec_start, sec_end);
	}
}

static int erase_sector(struct flash_bank *bank, unsigned int sector_idx)
{
	int hr = mxs40_sromalgo_prepare(bank);
	if (hr != ERROR_OK)
		return hr;

	hr = mxs40_erase_row(bank, bank->base + bank->sectors[sector_idx].offset, true);

	mxs40_sromalgo_release(bank->target);
	return hr;
}

static int program_words(struct flash_bank *bank, unsigned int sector_idx, struct wflash_word *words)
{
	struct flash_sector *sector = &bank->sectors[sector_idx];
	const uint32_t max_words = sector->size / 4u;

	int hr = mxs40_sromalgo_prepare(bank);
	if (hr != ERROR_OK)
		return hr;

	for (size_t w = 0; w < max_words; w++) {
		if (!words[w].is_valid)
			continue;

		const uint32_t page_addr = bank->base + sector->offset + w * 4u;
		LOG_DEBUG("Programming 0x%08X to 0x%08X", words[w].data, page_addr);

		hr = mxs40_program_row_inner(bank, page_addr, (uint8_t *)&words[w].data, false, 2);
		if (hr != ERROR_OK)
			goto exit;
	}

exit:
	mxs40_sromalgo_release(bank->target);
	return hr;
}

int wflash_program_sector(struct flash_bank *bank, unsigned int sector_idx, struct image *image)
{
	int hr = ERROR_OK;
	struct flash_sector *sector = &bank->sectors[sector_idx];
	const uint32_t start_addr = bank->base + sector->offset;
	const uint32_t end_addr = start_addr + sector->size;
	struct wflash_word *words = NULL;
	uint8_t *section_data = NULL;

	for (unsigned int s_idx = 0u; s_idx < image->num_sections; s_idx++) {
		struct imagesection *section = &image->sections[s_idx];

		int64_t subsect_start = MAX(section->base_address, start_addr);
		int64_t subsect_end = MIN(section->base_address + section->size, end_addr);

		if (subsect_start % 4u)
			subsect_start = (subsect_start + 3) & ~3ull;

		if (subsect_end % 4u)
			subsect_end = subsect_end & ~3ull;

		if (subsect_start >= subsect_end)
			continue;

		if (words == NULL) {
			LOG_DEBUG("Lazy-reading sector @" TARGET_ADDR_FMT, bank->base + sector->offset);
			words = calloc(sector->size / 4u, sizeof(struct wflash_word));
			if (words == NULL)
				return ERROR_FAIL;

			hr = wflash_read_sector(bank, sector, words);
			if (hr != ERROR_OK)
				goto exit;

			bool sector_has_data = false;
			for (uint32_t i = 0; i < sector->size / 4u; i++) {
				if (words[i].is_valid) {
					sector_has_data = true;
					break;
				}
			}

			if (sector_has_data) {
				LOG_DEBUG("Lazy-erasing sector %d", sector_idx);
				hr = erase_sector(bank, sector_idx);
				if (hr != ERROR_OK)
					goto exit;
			} else {
				LOG_DEBUG("No data, erase skipped");
			}
		}

		section_data = malloc(section->size);
		if (section_data == NULL) {
			hr = ERROR_FAIL;
			goto exit;
		}

		size_t bytes_read;
		hr = image_read_section(image, (int)s_idx, 0u, section->size, section_data, &bytes_read);
		assert(bytes_read == section->size);
		if (hr != ERROR_OK)
			goto exit_free_data;

		for (uint32_t addr = subsect_start; addr < subsect_end; addr += 4u) {
			const uint32_t hex_offset = addr - section->base_address;
			const uint32_t word_idx = (addr - start_addr) / 4u;
			uint32_t data = buf_get_u32(section_data + hex_offset, 0u, 32u);
			words[word_idx].is_valid = true;
			words[word_idx].data = data;
		}

		free(section_data);
	}

	if (words)
		hr = program_words(bank, sector_idx, words);

exit:
	free(words);
	return hr;

exit_free_data:
	free(section_data);
	free(words);
	return hr;
}

COMMAND_HANDLER(wflash_blank_map_command)
{
	if (CMD_ARGC > 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target_addr_t wflash_base = TV2_WORK_FLASH0_BASE;

	if (CMD_ARGC >= 1) {
		if(strcmp("fc0", CMD_ARGV[0]) == 0) {
			wflash_base = TV2_WORK_FLASH0_BASE;
			CMD_ARGC--;
			CMD_ARGV++;
		} else if(strcmp("fc1", CMD_ARGV[0]) == 0) {
			wflash_base = TV2_WORK_FLASH1_BASE;
			CMD_ARGC--;
			CMD_ARGV++;
		}
	}

	struct target *target = get_current_target(CMD_CTX);
	struct flash_bank *bank;

	int hr = get_flash_bank_by_addr(target, wflash_base, true, &bank);
	if (hr != ERROR_OK)
		return hr;

	/* Handle also 'virtual' banks */
	if (strcmp(bank->driver->name, "virtual") == 0)
		bank = get_flash_bank_by_name_noprobe(bank->driver_priv);

	if (bank == NULL)
		return ERROR_FLASH_BANK_INVALID;

	unsigned int first_sector = 0u;
	unsigned int last_sector = bank->num_sectors - 1u;

	if (CMD_ARGC > 0) {
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], first_sector);
		last_sector = first_sector;
	}

	if (CMD_ARGC > 1) {
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], first_sector);

		if (strcmp(CMD_ARGV[1], "last") != 0)
			COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], last_sector);
	}

	if (first_sector > last_sector) {
		command_print(CMD, "ERROR: first sector must be <= last");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (last_sector >= bank->num_sectors) {
		command_print(CMD, "ERROR: last sector must be <= %u", bank->num_sectors - 1);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	LOG_USER_N("WorkFlash (%s) word validity map:", bank->name);
	for (unsigned int sector = first_sector; sector <= last_sector; sector++) {
		const size_t num_words = bank->sectors[sector].size / 4;
		struct wflash_word *words = calloc(num_words, sizeof(struct wflash_word));
		if (words == NULL)
			return ERROR_FAIL;

		hr = wflash_read_sector(bank, &bank->sectors[sector], words);
		if (hr != ERROR_OK) {
			free(words);
			goto exit;
		}

		for (size_t i = 0; i < num_words; i++) {
			uint32_t word_addr = bank->base + bank->sectors[sector].offset + i * 4u;

			if (i % 64u == 0)
				LOG_USER_N("\n0x%08x (#%03d): ", word_addr, sector);

			LOG_USER_N(words[i].is_valid ? "+" : "-");
		}

		free(words);
	}

exit:
	LOG_USER_N("\n");
	return hr;
}

COMMAND_HANDLER(wflash_write_image_command)
{
	target_addr_t wflash_base = TV2_WORK_FLASH0_BASE;

	if (CMD_ARGC >= 1) {
		if(strcmp("fc0", CMD_ARGV[0]) == 0) {
			wflash_base = TV2_WORK_FLASH0_BASE;
			CMD_ARGC--;
			CMD_ARGV++;
		} else if(strcmp("fc1", CMD_ARGV[0]) == 0) {
			wflash_base = TV2_WORK_FLASH1_BASE;
			CMD_ARGC--;
			CMD_ARGV++;
		}
	}

	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	struct flash_bank *bank;
	int hr = get_flash_bank_by_addr(target, wflash_base, true, &bank);
	if (hr != ERROR_OK)
		return hr;

	/* Handle also 'virtual' banks */
	if (strcmp(bank->driver->name, "virtual") == 0)
		bank = get_flash_bank_by_name_noprobe(bank->driver_priv);

	if (bank == NULL)
		return ERROR_FLASH_BANK_INVALID;

	struct image image;
	memset(&image, 0, sizeof(struct image));

	if (CMD_ARGC == 2) {
		target_addr_t base_address;
		COMMAND_PARSE_ADDRESS(CMD_ARGV[1], base_address);
		image.base_address_set = true;
		image.base_address = base_address;
	}

	hr = image_open(&image, CMD_ARGV[0], NULL);
	if (hr != ERROR_OK)
		return hr;

	wflash_validate_image(bank, &image);

	progress_init(bank->num_sectors, PROGRAMMING);
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		hr = wflash_program_sector(bank, sector, &image);
		if (hr != ERROR_OK)
			goto exit;
		progress_sofar(sector);
	}

exit:
	progress_done(hr);
	image_close(&image);

	return hr;
}

COMMAND_HANDLER(wflash_write_words_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;


	struct target *target = get_current_target(CMD_CTX);

	target_addr_t section_base;
	COMMAND_PARSE_ADDRESS(CMD_ARGV[0], section_base);
	if (section_base & 3ull) {
		LOG_ERROR("Address is not 32-bit aligned");
		return ERROR_FAIL;
	}

	struct flash_bank *bank;
	int hr = get_flash_bank_by_addr(target, section_base, true, &bank);
	if (hr != ERROR_OK)
		return hr;

	/* Handle also 'virtual' banks */
	if (strcmp(bank->driver->name, "virtual") == 0)
		bank = get_flash_bank_by_name_noprobe(bank->driver_priv);

	if (bank == NULL)
		return ERROR_FLASH_BANK_INVALID;

	struct image image;
	memset(&image, 0, sizeof(struct image));

	CMD_ARGV++;
	CMD_ARGC--;

	uint32_t data[CMD_ARGC];
	for (unsigned int i = 0; i < CMD_ARGC; i++)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i], data[i]);

	hr = image_open(&image, "", "build");
	if (hr != ERROR_OK)
		return hr;

	hr = image_add_section(&image, section_base, CMD_ARGC * sizeof(uint32_t), 0, (uint8_t *)data);
	if (hr != ERROR_OK)
		goto exit;

	wflash_validate_image(bank, &image);
	progress_init(bank->num_sectors, PROGRAMMING);
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		hr = wflash_program_sector(bank, sector, &image);
		if (hr != ERROR_OK)
			goto exit;
		progress_sofar(sector);
	}

exit:
	progress_done(hr);
	image_close(&image);
	return hr;
}

const struct command_registration tv2_wflash_command_handlers[] = {
	{
		.name = "blank_map",
		.handler = wflash_blank_map_command,
		.mode = COMMAND_EXEC,
		.usage = "[fc0|fc1] [first_sector [last_sector | 'last']]",
		.help = "Displays per-word erase state of WFlash",
	},
	{
		.name = "write_image",
		.handler = wflash_write_image_command,
		.mode = COMMAND_EXEC,
		.usage = "[fc0|fc1] filename [offset]",
		.help = "Writes data from given image into WFlash bank",
	},
	{
		.name = "write_words",
		.handler = wflash_write_words_command,
		.mode = COMMAND_EXEC,
		.usage = "address word_0 [word_1] ... [word_n]",
		.help = "Programs given words into WFlash individually",
	},
	COMMAND_REGISTRATION_DONE
};
