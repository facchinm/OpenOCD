// SPDX-License-Identifier: GPL-2.0-or-later

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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <transport/transport.h>
#include <jtag/swd.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/tcl.h>
#include <flash/nor/core.h>

#include <jtag/drivers/cmsis_dap.h>
#include <helper/time_support.h>

#define KP3_TARGET_VOLTAGE_MAX_ERROR      100
#define KP3_TARGET_VOLTAGE_MAX_ERROR_PC   5

#define KP3_USB_TIMEOUT                   1000u
#define KP3_REQUEST_VERSION               0x80
#define KP3_REQUEST_ACQUIRE               0x85
#define KP3_REQUEST_ACQUIRE_PARAM         0x91
#define KP3_REQUEST_POWER                 0x84
#define KP3_REQUEST_POWER_SET             0x10
#define KP3_REQUEST_POWER_GET             0x11
#define KP3_REQUEST_LED_CONTROL           0x83

#define KP3_LED_STATE_READY               0x00u
#define KP3_LED_STATE_PROGRAMMING         0x01u
#define KP3_LED_STATE_SUCCESS             0x02u
#define KP3_LED_STATE_ERROR               0x03u

#define KP3_ACQUIRE_PARAM_TIMEOUT         0x00
#define KP3_ACQUIRE_PARAM_SEQ_TYPE        0x01
#define KP3_ACQUIRE_PARAM_AP_NUM          0x02

#define KP3_ACQUIRE_TARGET_CUSTOM         0xFE
#define KP3_ACQUIRE_TARGET_PSOC4          0x00
#define KP3_ACQUIRE_TARGET_PSOC5          0x01
#define KP3_ACQUIRE_TARGET_PSOC6          0x02
#define KP3_ACQUIRE_TARGET_TVII           0x03
#define KP3_ACQUIRE_TARGET_CYW20829       0x04

#define INVOKE_CMSIS_DAP(name, ...)       cmsis_dap_adapter_driver.name(__VA_ARGS__)

extern struct jtag_interface kitprog3_dap_interface ;

#define MAX_NUM_CUSTOM_COMMANDS 4

struct acquire_config {
	bool configured;
	bool enabled;
	uint8_t target_type;
	uint8_t acquire_mode;
	uint8_t attempts;
	uint8_t acquire_timeout;
	uint8_t acquire_ap;
	struct {
		size_t cmd_len;
		uint8_t *cmd;
	} custom_commands[MAX_NUM_CUSTOM_COMMANDS];
};

struct power_config {
	bool configured;
	uint16_t voltage;
};

struct kp3_version {
	char *directory;
	size_t major;
	size_t minor;
	size_t build;
};

bool g_kp3_ver_valid;
static struct kp3_version g_kp3_ver;
static struct kp3_version g_kp3_latest_ver;
static struct acquire_config acquire_config;
static struct power_config power_config;
static uint16_t initial_voltage = UINT16_MAX;

/** ***********************************************************************************************
 * @brief Returns target name from the type byte parameter for KP3 acquisition command
 * @return Target name.
 *************************************************************************************************/
static const char* kitprog3_target_type_to_name(uint8_t target_type)
{
	switch (target_type) {
        case KP3_ACQUIRE_TARGET_PSOC4:    return "PSOC 4";
        case KP3_ACQUIRE_TARGET_PSOC5:    return "PSOC 5";
        case KP3_ACQUIRE_TARGET_PSOC6:    return "PSOC 6";
        case KP3_ACQUIRE_TARGET_TVII:     return "TraveoII/XMC7xxx/XMC5xxx";
        case KP3_ACQUIRE_TARGET_CYW20829: return "CYW20829";
        case KP3_ACQUIRE_TARGET_CUSTOM:   return "Custom";
        default: return "Unknown";
    }
}

/** ***********************************************************************************************
 * @brief Compares KitProg3 versions represented by kp3_version structure
 * @return -1 (a < b), 0 (a == b) or +1 (a > b)
 *************************************************************************************************/
static int kitprog3_compare_versions(struct kp3_version *a, struct kp3_version *b) {
	if (a->major < b->major) return -1;
	if (a->major > b->major) return 1;
	if (a->minor < b->minor) return -1;
	if (a->minor > b->minor) return 1;
	if (a->build < b->build) return -1;
	if (a->build > b->build) return 1;
	return 0;
}

/** ***********************************************************************************************
 * @brief Performs USB transaction. Data to be sent must be stored in dap->command.
 * @param timeout timeout value, in milliseconds
 * @param has_cmd_stat true if response to the current request contains status sield in second byte
 * @return ERROR_OK in case of success, ERROR_FAIL otherwise.
 * Received data is stored in dap->response.
 *************************************************************************************************/
static int kitprog3_usb_request(int timeout, bool has_cmd_stat)
{
	struct cmsis_dap *dap = cmsis_dap_handle;
	size_t packet_size = dap->packet_size;
	int hr;

	hr = dap->backend->write(dap, packet_size, timeout);
	if (hr < 0)
		return hr;

	do {
		/* Avoid warning during PSoC64 acquisition */
		keep_alive();

		hr = dap->backend->read(dap, timeout, NULL);
		if (hr < 0)
			return hr;
	} while (dap->response[1] == 1);

	if(has_cmd_stat)
		return dap->response[1] == 0 ? ERROR_OK : ERROR_FAIL;

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Shows "Please update the firmware message"
 *************************************************************************************************/
static void kitprog3_show_update_fw_message(bool is_fatal)
{
	const char * const message = {
		"\n*******************************************************************************************"
		"\n* KitProg3 firmware is out of date. Update to the latest detected version (%zu.%zu.%zu)"
		"\n* found in %s"
		"\n* using ModusToolbox Programmer or Advanced KitProg3 Programming launch config option"
		"\n* in the Quick Panel of Eclipse IDE or fw-loader CLI tool"
		"\n*******************************************************************************************"
	};

	const struct kp3_version *v = &g_kp3_latest_ver;
	if(is_fatal)
		LOG_ERROR(message, v->major, v->minor, v->build, v->directory);
	else
		LOG_WARNING(message, v->major, v->minor, v->build, v->directory);
}

/** ***********************************************************************************************
 * @brief Controls LED status
 * @param status Type of LED indication. Possible values:
 * - KP3_LED_STATE_READY
 * - KP3_LED_STATE_PROGRAMMING
 * - KP3_LED_STATE_SUCCESS
 * - KP3_LED_STATE_ERROR
 * @return ERROR_OK in case of success, ERROR_FAIL otherwise.
 *************************************************************************************************/
static int kitprog3_led_control(uint8_t status)
{
	struct cmsis_dap *dap = cmsis_dap_handle;
	dap->command[0] = KP3_REQUEST_LED_CONTROL;
	dap->command[1] = status;

	int hr = kitprog3_usb_request(KP3_USB_TIMEOUT, true);
	if (hr != ERROR_OK)
		LOG_ERROR("kitprog3: led_control command failed");

	return hr;
}

/** ***********************************************************************************************
 * @brief Measures target voltage
 * @param voltage is populated with the target voltage value in millivolts
 * @return ERROR_OK in case of success, ERROR_FAIL otherwise.
 *************************************************************************************************/
static int kitprog3_get_power(uint16_t *voltage)
{
	struct cmsis_dap *dap = cmsis_dap_handle;
	dap->command[0] = KP3_REQUEST_POWER;
	dap->command[1] = KP3_REQUEST_POWER_GET;

	int hr = kitprog3_usb_request(KP3_USB_TIMEOUT, true);
	if (hr != ERROR_OK) {
		LOG_ERROR("kitprog3: get_power command failed");
		return hr;
	}

	*voltage = (uint16_t)(dap->response[3] + (dap->response[4] << 8));
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Controls power sypply of the target device. Target voltage must be configured using
 * 'power_config' before calling this function.
 * @return ERROR_OK in case of success, error code otherwise.
 *************************************************************************************************/
static int kitprog3_power_control(void)
{
	if (!power_config.configured) {
		LOG_ERROR("kitprog3: target voltage not configured, (use 'kitprog3 power_config')");
		return ERROR_FAIL;
	}

	struct cmsis_dap *dap = cmsis_dap_handle;
	dap->command[0] = KP3_REQUEST_POWER;
	dap->command[1] = KP3_REQUEST_POWER_SET;
	if (power_config.voltage == 0) {
		LOG_INFO("kitprog3: powering down target device using KitProg3");
		dap->command[2] = 0x00;
		dap->command[3] = 0x00;
		dap->command[4] = 0x00;
	} else if (power_config.voltage == UINT16_MAX) {
		LOG_INFO("kitprog3: powering up target device using KitProg3 (default voltage)");
		dap->command[2] = 0x01;
		dap->command[3] = 0;
		dap->command[4] = 0;
	} else {
		LOG_INFO("kitprog3: powering up target device using KitProg3 (VTarg = %d mV)", power_config.voltage);
		dap->command[2] = power_config.voltage ? 0x02 : 0x00;
		dap->command[3] = power_config.voltage & 0xFF;
		dap->command[4] = power_config.voltage >> 8;
	}

	int hr = kitprog3_usb_request(KP3_USB_TIMEOUT, true);
	if (hr != ERROR_OK)
		LOG_ERROR("kitprog3: power_control command failed (target already powered?)");

	/* We do not know what the default voltage is,
	 * just sleep for a while to allow voltage to settle */
	if (power_config.voltage == UINT16_MAX) {
		alive_sleep(250);
		return kitprog3_get_power(&initial_voltage);
	}

	/* Wait up to KP3_USB_TIMEOUT for target voltage to stabilise */
	int64_t start = timeval_ms();
	uint16_t prev_voltage = UINT16_MAX;
	while(timeval_ms() - start <= KP3_USB_TIMEOUT) {
		hr = kitprog3_get_power(&initial_voltage);
		if(hr != ERROR_OK)
			return hr;

		if(abs(initial_voltage - prev_voltage) <= KP3_TARGET_VOLTAGE_MAX_ERROR)
			break;

		prev_voltage = initial_voltage;
		alive_sleep(50);
	}

	if(timeval_ms() - start > KP3_USB_TIMEOUT)
		LOG_WARNING("Target voltage (%u.%03u V) is not within %d mV of requested",
					initial_voltage / 1000, initial_voltage % 1000,
					KP3_TARGET_VOLTAGE_MAX_ERROR);
	return hr;
}

/** ***********************************************************************************************
 * @brief Acquires target device. Acquisition mode and parameters must be configured with
 * 'acquire_config' before calling this function
 * @return ERROR_OK in case of success, error code otherwise.
 *************************************************************************************************/
static int kitprog3_acquire_psoc(void)
{
	if (!acquire_config.configured) {
		LOG_ERROR("kitprog3: acquire procedure not configured, (use 'kitprog3 acquire_config')");
		goto error;
	}

	if (kitprog3_led_control(KP3_LED_STATE_PROGRAMMING) != ERROR_OK) {
		LOG_ERROR("kitprog3: failed to set the LED status");
		goto error;
	}

	/* Set acquisition timeout. Common for both, custom and predefined methods */
	struct cmsis_dap *dap = cmsis_dap_handle;
	if (acquire_config.acquire_timeout) {
		dap->command[0] = KP3_REQUEST_ACQUIRE_PARAM;
		dap->command[1] = KP3_ACQUIRE_PARAM_TIMEOUT;
		dap->command[2] = acquire_config.acquire_timeout;
		if (kitprog3_usb_request(KP3_USB_TIMEOUT, true) != ERROR_OK) {
			LOG_WARNING("KitProg3: Acquire timeout not configurable, please update the firmware");
		}
	}

	if (acquire_config.target_type == KP3_ACQUIRE_TARGET_CUSTOM) {
		/* Acquire target using the custom sequence, defined in config script */
		LOG_INFO("kitprog3: acquiring device in Test Mode using custom target sequence...");
		bool success = true;

		for (size_t cmd_idx = 0; cmd_idx < MAX_NUM_CUSTOM_COMMANDS; cmd_idx++) {
			if (!acquire_config.custom_commands[cmd_idx].cmd_len) {
				break;
			}

			for (size_t cmd_len = 0; cmd_len < acquire_config.custom_commands[cmd_idx].cmd_len; cmd_len++) {
				dap->command[cmd_len] = acquire_config.custom_commands[cmd_idx].cmd[cmd_len];
			}

			if (kitprog3_usb_request(KP3_USB_TIMEOUT, true) != ERROR_OK) {
				LOG_ERROR("kitprog3: acquire_psoc request failed");
				goto error;
			}

			if(dap->command[0] == KP3_REQUEST_ACQUIRE) {
				success = dap->command[2] == 0x01;
			}
		}
		if (!success) {
			goto error;
		}

	} else {
		/* Acquire target using one of predefined in kitprog3 method */
		LOG_INFO("kitprog3: acquiring device in Test Mode using '%s' target and '%s' reset...",
			kitprog3_target_type_to_name(acquire_config.target_type),
			acquire_config.acquire_mode ? "power-cycle" : "xres");

		/* Set acquisition Access Port. Used along with timeout. */
		if (acquire_config.acquire_timeout) {
			dap->command[0] = KP3_REQUEST_ACQUIRE_PARAM;
			dap->command[1] = KP3_ACQUIRE_PARAM_AP_NUM;
			dap->command[2] = acquire_config.acquire_ap;
			if (kitprog3_usb_request(KP3_USB_TIMEOUT, true) != ERROR_OK) {
				LOG_WARNING("KitProg3: Acquire AP not configurable, please update the firmware");
			}
		}

		dap->command[0] = KP3_REQUEST_ACQUIRE;
		dap->command[1] = acquire_config.target_type;
		dap->command[2] = acquire_config.acquire_mode;
		dap->command[3] = acquire_config.attempts;
		if (kitprog3_usb_request(acquire_config.attempts * KP3_USB_TIMEOUT, true) != ERROR_OK || dap->response[2] == 0) {
			LOG_ERROR("kitprog3: acquire_psoc request failed");
			goto error;
		}
	}

	kitprog3_led_control(KP3_LED_STATE_SUCCESS);
	return ERROR_OK;

error:
	LOG_ERROR("kitprog3: failed to acquire the device");
	kitprog3_led_control(KP3_LED_STATE_ERROR);
	return ERROR_FAIL;
}

/** ***********************************************************************************************
 * @brief Compares KitProg version with the latest version available (if set) and warns the user
 * in case firmware update is needed
 * @return ERROR_OK in case of success, error code otherwise.
 *************************************************************************************************/
static int kitprog3_check_version(void)
{
	struct cmsis_dap *dap = cmsis_dap_handle;

	dap->command[0] = 0;
	dap->command[1] = 4;

	int hr = kitprog3_usb_request(KP3_USB_TIMEOUT, false);
	if (hr != ERROR_OK) {
		LOG_ERROR("kitprog3: get_version command failed");
		return hr;
	}

	int is_kp3 = strcmp("1.10", (char *)&dap->response[2]);

	if(!is_kp3) {
		kitprog3_show_update_fw_message(true);
		return ERROR_FAIL;
	}

	dap->command[0] = KP3_REQUEST_VERSION;

	hr = kitprog3_usb_request(KP3_USB_TIMEOUT, true);
	if (hr != ERROR_OK) {
		LOG_ERROR("kitprog3: get_version command failed");
		return hr;
	}

	g_kp3_ver.major = dap->response[2] + (dap->response[3] << 8);
	g_kp3_ver.minor = dap->response[4] + (dap->response[5] << 8);
	g_kp3_ver.build = dap->response[10] + (dap->response[11] << 8);
	g_kp3_ver_valid = true;

	LOG_INFO("KitProg3: FW version: %zu.%zu.%zu", g_kp3_ver.major, g_kp3_ver.minor, g_kp3_ver.build);

#if BUILD_CMSIS_DAP_HID == 1
	/* The rest of the code is related to BULK backends only*/
	const bool is_hid_backend = strcmp("hid", cmsis_dap_handle->backend->name) == 0;
	if (is_hid_backend)
		return ERROR_OK;
#endif

#if BUILD_CMSIS_DAP_USB == 1
	/* Workaround for KP3 FW < 1.20.591 in BULK mode */
	if(kitprog3_compare_versions(&g_kp3_ver, &(struct kp3_version){NULL, 1, 20, 591}) < 0) {
		cmsis_dap_handle->packet_count = 1;
		LOG_INFO("KitProg3: Pipelined transfers disabled, please update the firmware");
	} else {
		LOG_INFO("KitProg3: Pipelined transfers enabled");
	}

	/* Workaround for KP3 FW < 2.30.1081 - hangs with asynchronous BULK backend */
	if(kitprog3_compare_versions(&g_kp3_ver, &(struct kp3_version){NULL, 2, 30, 1081}) < 0) {
		cmsis_dap_handle->quirk_mode = true;
		LOG_INFO("KitProg3: Asynchronous USB transfers disabled, please update the firmware");
	} else {
		LOG_INFO("KitProg3: Asynchronous USB transfers enabled");
	}
#endif

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Resets Acquisition parameters to their default values
 *************************************************************************************************/
static void kitprog3_reset_acquire_params(void)
{
	struct cmsis_dap *dap = cmsis_dap_handle;

	/* Reset all possibly overridden KitProg3 parameters :
	 * Parameter = 0: Set Acquire Timeout to zero
	 * set to zero - reset to default */
	dap->command[0] = KP3_REQUEST_ACQUIRE_PARAM;
	dap->command[1] = KP3_ACQUIRE_PARAM_TIMEOUT;
	dap->command[2] = 0;
	kitprog3_usb_request(KP3_USB_TIMEOUT, true);

#if defined(KP3_ACQUIRE_PARAM_SEQ_TYPE)
	/* Parameter = 1: Set type of DAP handshake
	 * set to zero - swd line reset for PSoC4, jtag-to-swd for others */
	dap->command[0] = KP3_REQUEST_ACQUIRE_PARAM;
	dap->command[1] = KP3_ACQUIRE_PARAM_SEQ_TYPE;
	dap->command[2] = 0;
	kitprog3_usb_request(KP3_USB_TIMEOUT, true);
#endif

	/* Parameter = 2:  Set DAP AP through which the test mode bit will be set
	 * set to zero - SysAP */
	dap->command[0] = KP3_REQUEST_ACQUIRE_PARAM;
	dap->command[1] = KP3_ACQUIRE_PARAM_AP_NUM;
	dap->command[2] = 0;
	kitprog3_usb_request(KP3_USB_TIMEOUT, true);
}

/** ***********************************************************************************************
 * @brief Initializes KitProg3 driver and underlying CMSIS-DAP subsystem. Function optionnally
 * powers up and acquires target device.
 * @return ERROR_OK in case of success, error code otherwise.
 *************************************************************************************************/
static int kitprog3_init(void)
{
	kitprog3_adapter_driver.speed = cmsis_dap_adapter_driver.speed;
	kitprog3_adapter_driver.speed_div = cmsis_dap_adapter_driver.speed_div;
	kitprog3_adapter_driver.khz = cmsis_dap_adapter_driver.khz;
	kitprog3_adapter_driver.reset = cmsis_dap_adapter_driver.reset;

	int hr = INVOKE_CMSIS_DAP(init);
	if (hr != ERROR_OK)
		return hr;

	const uint16_t vid = cmsis_dap_handle->vid;
	const uint16_t pid = cmsis_dap_handle->pid;

	if (vid == 0x0d28 && pid == 0x0204) {
		LOG_ERROR("It looks like your debug probe is in DAPLink mode. "
				  "Please switch to KitProg3 mode or use CMSIS-DAP driver instead.");

		INVOKE_CMSIS_DAP(quit);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	kitprog3_reset_acquire_params();

	hr = kitprog3_check_version();
	if (hr != ERROR_OK)
		return hr;

	if (power_config.configured) {
		hr = kitprog3_power_control();
		if (hr != ERROR_OK)
			return hr;
	}

	hr = kitprog3_get_power(&initial_voltage);
	if (hr == ERROR_OK)
		LOG_INFO("VTarget = %u.%03u V", initial_voltage / 1000, initial_voltage % 1000);

	if (acquire_config.configured && acquire_config.enabled)
		kitprog3_acquire_psoc();

	kitprog3_led_control(KP3_LED_STATE_READY);

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Deinitialezes KitProg3 driver and underlying CMSIS-DAP subsystem
 * @return ERROR_OK in case of success, error code otherwise.
 *************************************************************************************************/
static int kitprog3_quit(void)
{
	kitprog3_led_control(KP3_LED_STATE_READY);
	kitprog3_reset_acquire_params();

	if(g_kp3_ver_valid && kitprog3_compare_versions(&g_kp3_ver, &g_kp3_latest_ver) < 0)
		kitprog3_show_update_fw_message(false);

	free(g_kp3_latest_ver.directory); /* free(NULL) is OK */

	for (size_t i = 0; i < MAX_NUM_CUSTOM_COMMANDS; i++)
		free(acquire_config.custom_commands[i].cmd);

	return INVOKE_CMSIS_DAP(quit);
}

/**************************************************************************************************
 * Configuration command handlers
 *************************************************************************************************/
COMMAND_HANDLER(cmsis_dap_handle_acquire_config_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	bool enabled = false;
	uint8_t target_type = acquire_config.target_type;
	uint8_t acquire_mode = acquire_config.acquire_mode;
	uint8_t attempts = acquire_config.attempts;
	uint8_t acquire_timeout = acquire_config.acquire_timeout;
	uint8_t acquire_ap = acquire_config.acquire_ap;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], enabled);

	if (CMD_ARGC > 1) {
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], target_type);

		if (target_type == KP3_ACQUIRE_TARGET_CUSTOM) {
			for (size_t i = 0; i < MAX_NUM_CUSTOM_COMMANDS; i++) {
				free(acquire_config.custom_commands[i].cmd);
				acquire_config.custom_commands[i].cmd_len = 0;
			}

			for (size_t i = 0; i < CMD_ARGC - 2; i++) {
				int hr = byte_buffer_from_string(CMD_ARGV[i + 2],
								&acquire_config.custom_commands[i].cmd,
								&acquire_config.custom_commands[i].cmd_len);

				if (hr != ERROR_OK) {
					LOG_ERROR("Failed to parse hex string %s", CMD_ARGV[i + 1]);
					return ERROR_COMMAND_ARGUMENT_INVALID;
				}
			}
			acquire_config.configured = true;
			acquire_config.enabled = enabled;
			acquire_config.target_type = target_type;
			return ERROR_OK;
		}

		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[2], acquire_mode);
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[3], attempts);

		if (CMD_ARGC > 4) {
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[4], acquire_timeout);
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[5], acquire_ap);
		}
	}

	acquire_config.configured = true;
	acquire_config.enabled = enabled;
	acquire_config.target_type = target_type;
	acquire_config.acquire_mode = acquire_mode;
	acquire_config.attempts = attempts;
	acquire_config.acquire_timeout = acquire_timeout;
	acquire_config.acquire_ap = acquire_ap;

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_power_config_command)
{
	if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	bool enabled = false;
	uint16_t voltage = UINT16_MAX;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], enabled);

	if (CMD_ARGC == 2)
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], voltage);

	power_config.configured = true;
	power_config.voltage = enabled ? voltage : 0;

	switch (CMD_CTX->mode) {
	case COMMAND_EXEC:
		return kitprog3_power_control();
	default:
		return ERROR_OK;
	}
}

COMMAND_HANDLER(cmsis_dap_handle_power_control_command)
{
	LOG_USER("DEPRECATED! use 'kitprog3 power_config' not 'kitprog3 power_control'");
	return CALL_COMMAND_HANDLER(cmsis_dap_handle_power_config_command);
}

/**************************************************************************************************
 * Other command handlers
 *************************************************************************************************/
COMMAND_HANDLER(cmsis_dap_handle_acquire_psoc_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return kitprog3_acquire_psoc();
}

COMMAND_HANDLER(cmsis_dap_handle_get_power_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	uint16_t voltage = 0;
	int hr = kitprog3_get_power(&voltage);
	if (hr != ERROR_OK)
		return hr;

	command_print(CMD, "VTarget = %u.%03u V", voltage / 1000, voltage % 1000);
	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_get_version_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	struct cmsis_dap *dap = cmsis_dap_handle;
	dap->command[0] = KP3_REQUEST_VERSION;

	int hr = kitprog3_usb_request(KP3_USB_TIMEOUT, true);
	if (hr != ERROR_OK) {
		LOG_ERROR("kitprog3: get_version command failed");
		return hr;
	}

	struct kp3_version kp3_ver;
	kp3_ver.major = dap->response[2] + (dap->response[3] << 8);
	kp3_ver.minor = dap->response[4] + (dap->response[5] << 8);
	kp3_ver.build = dap->response[10] + (dap->response[11] << 8);

	uint16_t kit_hw_id = dap->response[6] + (dap->response[7] << 8);
	uint8_t kit_prot_major = dap->response[8];
	uint8_t kit_prot_minor = dap->response[9];

	command_print(CMD, "KitProg3 FW Ver = %zu.%zu.%zu, HW ID = %u, KHPI Ver = %u.%u",
				kp3_ver.major, kp3_ver.minor, kp3_ver.build,
				kit_hw_id, kit_prot_major, kit_prot_minor);

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_led_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	uint8_t status = KP3_LED_STATE_ERROR;
	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], status);

	return kitprog3_led_control(status);
}

COMMAND_HANDLER(cmsis_dap_handle_set_latest_version_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	struct kp3_version kp3_ver;

	if(sscanf(CMD_ARGV[1], "%zu.%zu.%zu", &kp3_ver.major, &kp3_ver.minor, &kp3_ver.build) != 3) {
		LOG_ERROR("KitProg3: Invalid version format, should match %%zu.%%zu.%%zu pattern");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	/* Remember only the latest version found */
	if(kitprog3_compare_versions(&kp3_ver, &g_kp3_latest_ver) > 0) {
		free(g_kp3_latest_ver.directory); /* free(NULL) is OK */
		g_kp3_latest_ver.directory = strdup(CMD_ARGV[0]);
		g_kp3_latest_ver.major = kp3_ver.major;
		g_kp3_latest_ver.minor = kp3_ver.minor;
		g_kp3_latest_ver.build = kp3_ver.build;
	}

	return ERROR_OK;
}

static const struct command_registration kitprog3_subcommand_handlers[] = {
	{
		.name = "acquire_config",
		.handler = &cmsis_dap_handle_acquire_config_command,
		.mode = COMMAND_ANY,
		.usage = "<enabled on|off> <target_type (0:PSoC4, 1:PSoC5-LP, 2:PSoC6, 3:TVII, 4:CYW20829)>"
			"<mode (0: Reset, 1: Power Cycle)> <attempts> [<timeout_sec> <ap_num>]",
		.help = "Controls PSoC acquisition during init phase",
	},
	{
		.name = "acquire_psoc",
		.handler = &cmsis_dap_handle_acquire_psoc_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Acquire the device",
	},
	{
		.name = "power_config",
		.handler = &cmsis_dap_handle_power_config_command,
		.mode = COMMAND_ANY,
		.usage = "<enabled on|off> [voltage (mV)]",
		.help = "Controls the target's power supply during init phase",
	},
	{
		.name = "power_control",
		.handler = &cmsis_dap_handle_power_control_command,
		.mode = COMMAND_ANY,
		.usage = "[on|off] <enabled on|off> [voltage (mV)]",
		.help = "Controls the target's power supply",
	},
	{
		.name = "led_control",
		.handler = &cmsis_dap_handle_led_command,
		.mode = COMMAND_EXEC,
		.usage = "<type (0:READY, 1:PROGRAMMING, 2:SUCCESS, 3:ERROR)>",
		.help = "Controls LED state of the kp3",
	},
	{
		.name = "get_power",
		.handler = &cmsis_dap_handle_get_power_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Reports the target's power supply voltage",
	},
	{
		.name = "get_version",
		.handler = &cmsis_dap_handle_get_version_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Reports the target's firmware and hardware versions",
	},
	{
		.name = "set_latest_version",
		.handler = &cmsis_dap_handle_set_latest_version_command,
		.mode = COMMAND_CONFIG,
		.usage = "",
		.help = "Sets expexted KitProg version. OpenOCD will warn the user if FW version is out of date.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration kitprog3_command_handlers[] = {
	{
		.name = "kitprog3",
		.mode = COMMAND_ANY,
		.help = "perform KitProg3 management",
		.usage = "<cmd>",
		.chain = kitprog3_subcommand_handlers,
	},
	{
		.chain = cmsis_dap_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

#define POWER_STABLE_TIMEOUT_MS 5000L
static int kitprog3_power_dropout(int *power_dropout)
{
	static uint16_t prev_mv;

	if (initial_voltage == UINT16_MAX || prev_mv == 0) {
		*power_dropout = 0;
		int hr = kitprog3_get_power(&initial_voltage);
		prev_mv = initial_voltage;
		return hr;
	}

	const int max_error  = KP3_TARGET_VOLTAGE_MAX_ERROR_PC * initial_voltage / 100;
	static int64_t last_mv_change_time;

	uint16_t now_mv;
	int hr = kitprog3_get_power(&now_mv);
	if (hr != ERROR_OK)
		return hr;

	bool dropout = false;

	const bool mv_wrong = abs(initial_voltage - now_mv) > max_error;
	const bool mv_now_changed = abs(prev_mv - now_mv) > max_error;
	prev_mv = now_mv;

	if(mv_now_changed)
		last_mv_change_time = timeval_ms();

	if (mv_wrong)
		dropout = true;

	if (dropout && timeval_ms() - last_mv_change_time > POWER_STABLE_TIMEOUT_MS) {
		LOG_INFO("Voltage is stable for more than %lu sec, assuming power is good",
				 POWER_STABLE_TIMEOUT_MS / 1000);
		initial_voltage = now_mv;
		dropout = false;
	}

	*power_dropout = dropout;
	return ERROR_OK;
}

static const char *const kitprog3_transport[] = { "swd", "jtag", NULL };

struct adapter_driver kitprog3_adapter_driver = {
	.name = "kitprog3",
	.commands = kitprog3_command_handlers,
	.transports = kitprog3_transport,
	.speed = NULL,
	.speed_div = NULL,
	.khz = NULL,
	.reset = NULL,
	.init = kitprog3_init,
	.quit = kitprog3_quit,
	.power_dropout = kitprog3_power_dropout,

	.jtag_ops = &cmsis_dap_interface,
	.swd_ops = &cmsis_dap_swd_driver,
};
