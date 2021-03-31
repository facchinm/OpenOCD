/*
 * SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2021 Arduino SA
 * Author: Martino Facchin <m.facchin@arduino.cc>
 *
 * RTX5 Task Awareness
 */

#include <rtos/rtos.h>
#include <target/target.h>
#include <target/target_type.h>

#include "rtos_standard_stackings.h"

#define RTX5_MAX_TASKS 32
#define RTX5_MAX_NAME 200
#define RTX5_IDLE_STRING "rtx_idle"

// Porting from https://github.com/pyocd/pyOCD/blob/master/pyocd/rtos/rtx5.py

struct rtx5_params {
	const char *target_name;
	size_t ptr_size;
	off_t task_offset_next;
	off_t task_offset_sp;
	off_t task_offset_delaynext;
	const struct rtos_register_stacking *stacking;
	const struct rtos_register_stacking *stacking_fpu;
};

static const int CURRENT_OFFSET = 20;
static const int THREADLIST_OFFSET = 36;
static const int DELAYLIST_OFFSET = 44;
static const int WAITLIST_OFFSET = 48;
static const int THREADNEXT_OFFSET = 8;
static const int DELAYNEXT_OFFSET = 16;
static const int STATE_OFFSET = 1;
static const int NAME_OFFSET = 4;
static const int PRIORITY_OFFSET = 33;
//static const int STACKFRAME_OFFSET = 34;
static const int SP_OFFSET = 56;

static const char* const rtx5_states[] = {
	"Inactive",					
	"Ready",
	"Running",
	"Blocked",
	"Terminated",
	"Waiting[Delay]",
	"Waiting[Join]",
	"Waiting[ThrFlg]",
	"Waiting[EvtFlg]",
	"Waiting[Mutex]",
	"Waiting[Sem]",
	"Waiting[MemPool]",
	"Waiting[MsgGet]",
	"Waiting[MsgPut]",
};

static const struct rtx5_params rtx5_params_list[] = {
	{
		.target_name = "hla_target",
		.ptr_size = 4,
		.task_offset_sp = SP_OFFSET,
		.stacking = &rtos_standard_Cortex_M3_stacking,
		.stacking_fpu = NULL, // TODO
	},
	{
		.target_name = "cortex_m",
		.ptr_size = 4,
		.task_offset_sp = SP_OFFSET,
		.stacking = &rtos_standard_Cortex_M3_stacking,
		.stacking_fpu = NULL, // TODO
	},
};

static const char * const rtx5_symbol_list[] = {
	"osRtxInfo",
	NULL,
};

enum rtx5_symbol_values {
	RTX5_INFO = 0,
	RTX5_VAL_COUNT,
};

static bool rtx5_detect_rtos(struct target *target)
{
	enum rtx5_symbol_values sym;

	if (!target || !target->rtos || !target->rtos->symbols)
		return false;

	for (sym = RTX5_INFO;
	     sym < RTX5_VAL_COUNT; sym++) {
		if (target->rtos->symbols[sym].address) {
			LOG_DEBUG("RTX5: Symbol \"%s\" found",
				 rtx5_symbol_list[sym]);
		} else {
			LOG_ERROR("RTX5: Symbol \"%s\" missing",
				 rtx5_symbol_list[sym]);
			return false;
		}
	}

	return target->rtos->symbols &&
	       target->rtos->symbols[RTX5_INFO].address;
}

static int rtx5_create(struct target *target)
{
	struct rtx5_params *params;
	size_t t;

	for (t = 0; t < ARRAY_SIZE(rtx5_params_list); t++)
		if (!strcmp(rtx5_params_list[t].target_name, target->type->name)) {
			params = malloc(sizeof(*params));
			if (!params) {
				LOG_ERROR("RTX5: out of memory");
				return ERROR_FAIL;
			}

			memcpy(params, &rtx5_params_list[t], sizeof(*params));
			target->rtos->rtos_specific_params = (void *)params;
			target->rtos->current_thread = 0;
			target->rtos->thread_details = NULL;
			target->rtos->thread_count = 0;

			LOG_INFO("RTX5: Using target: %s", target->type->name);
			return ERROR_OK;
		}

	LOG_ERROR("RTX5: target not supported: %s", target->type->name);
	return ERROR_FAIL;
}

static int rtx5_get_current_task_ptr(struct rtos *rtos, uint32_t *current_task)
{

	if (!rtos || !rtos->symbols)
		return ERROR_FAIL;

	return target_read_u32(rtos->target,
			       rtos->symbols[RTX5_INFO].address + CURRENT_OFFSET,
			       current_task);
}

static int getThreadList(struct rtos *rtos, uint32_t offset, uint32_t next_offset, void (*action)(struct rtos *rtos, uint32_t task, uint32_t index), uint32_t index) {
	uint32_t task;
	int ret, found = 0;

	ret = target_read_u32(rtos->target,
			      rtos->symbols[RTX5_INFO].address + offset,
			      &task);

	while (ret == ERROR_OK && task != 0) {

		if (action != NULL) {
			action(rtos, task, index + found);
		}

		ret = target_read_u32(rtos->target,
			      task + next_offset,
			      &task);

		found++;

		if (next_offset == 0) {
			break;
		}
	}
	return found;
}

static void populateThreadInfo(struct rtos *rtos, uint32_t thread_ptr, uint32_t tasks_found) {

	char thread_str_buf[RTX5_MAX_NAME];

	rtos->thread_details[tasks_found].threadid = thread_ptr;

	uint32_t name_ptr;
	target_read_u32(rtos->target, thread_ptr + NAME_OFFSET, &name_ptr);

	/* read name buffer */
	int ret = target_read_buffer(rtos->target, name_ptr, RTX5_MAX_NAME,
				(uint8_t *)thread_str_buf);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read task name");
		return;
	}

	rtos->thread_details[tasks_found].thread_name_str = strdup(thread_str_buf);

	uint8_t state;
	uint8_t priority;

	target_read_u8(rtos->target, thread_ptr + STATE_OFFSET, &state);
	target_read_u8(rtos->target, thread_ptr + PRIORITY_OFFSET, &priority);

	snprintf(thread_str_buf, sizeof(thread_str_buf),
		"State: %s, Priority: %u\n", state < 5 ? rtx5_states[state] : rtx5_states[state / 0x10 + 4], priority);

	rtos->thread_details[tasks_found].extra_info_str = strdup(thread_str_buf);
	rtos->thread_details[tasks_found].exists = true;
}

static int rtx5_get_num_tasks(struct rtos *rtos, int *num_tasks)
{
	int found = 0;

	found += getThreadList(rtos, THREADLIST_OFFSET, THREADNEXT_OFFSET, NULL, 0);
	found += getThreadList(rtos, DELAYLIST_OFFSET, DELAYNEXT_OFFSET, NULL, 0);
	found += getThreadList(rtos, WAITLIST_OFFSET, DELAYNEXT_OFFSET, NULL, 0);
	found += getThreadList(rtos, CURRENT_OFFSET, 0, NULL, 0);

	*num_tasks = found + 1;

	return ERROR_OK;
}

static int rtx5_update_threads(struct rtos *rtos)
{
	uint32_t current_task;
	int ret, num_tasks, tasks_found;
	struct rtx5_params *params;

	params = rtos->rtos_specific_params;
	if (!params)
		return ERROR_FAIL;

	if (!rtos->symbols)
		return ERROR_FAIL;

	num_tasks = 0;
	ret = rtx5_get_num_tasks(rtos, &num_tasks);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to get number of tasks");
		return ret;
	}

	current_task = 0;
	ret = rtx5_get_current_task_ptr(rtos, &current_task);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to get current task");
		return ret;
	}
	LOG_DEBUG("Current task: %lx tasks_found: %d",
		  (unsigned long)current_task,
		  num_tasks);

	/* set current task to what we read */
	rtos->current_thread = current_task;

	/* Nuke the old tasks */
	rtos_free_threadlist(rtos);

	if (!rtos->current_thread || !num_tasks) {
		num_tasks++;

		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * num_tasks);
		rtos->thread_details->threadid = 1;
		rtos->thread_details->exists = true;
		rtos->thread_details->extra_info_str = NULL;
		rtos->thread_details->thread_name_str = strdup("Current Execution");

		if (!num_tasks) {
			rtos->thread_count = 1;
			return ERROR_OK;
		}
	} else {
		/* create space for new thread details */
		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * num_tasks);
	}

	rtos->current_thread = current_task;

	tasks_found = 0;

	tasks_found += getThreadList(rtos, THREADLIST_OFFSET, THREADNEXT_OFFSET, populateThreadInfo, tasks_found);
	tasks_found += getThreadList(rtos, DELAYLIST_OFFSET, DELAYNEXT_OFFSET, populateThreadInfo, tasks_found);
	tasks_found += getThreadList(rtos, WAITLIST_OFFSET, DELAYNEXT_OFFSET, populateThreadInfo, tasks_found);
	tasks_found += getThreadList(rtos, CURRENT_OFFSET, 0, populateThreadInfo, tasks_found);

	rtos->thread_count = tasks_found;

	return ERROR_OK;
}

static int rtx5_get_thread_reg_list(struct rtos *rtos,
					   threadid_t threadid,
					   struct rtos_reg **reg_list,
					   int *num_regs)
{
	struct rtx5_params *params = rtos->rtos_specific_params;
	uint32_t stack_ptr = 0;
	int ret;

	ret = target_read_u32(rtos->target,
			   threadid + SP_OFFSET,
			   &stack_ptr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to load TCB");
		return ret;
	}

	return rtos_generic_stack_read(rtos->target, params->stacking,
				       stack_ptr, reg_list, num_regs);
}

static int rtx5_get_symbol_list_to_lookup(struct symbol_table_elem_struct *symbol_list[])
{
	size_t s;

	*symbol_list = calloc(ARRAY_SIZE(rtx5_symbol_list),
			      sizeof(struct symbol_table_elem_struct));
	if (!(*symbol_list)) {
		LOG_ERROR("RTX5: out of memory");
		return ERROR_FAIL;
	}

	for (s = 0; s < ARRAY_SIZE(rtx5_symbol_list); s++)
		(*symbol_list)[s].symbol_name = rtx5_symbol_list[s];

	return ERROR_OK;
}

const struct rtos_type rtx5_rtos = {
	.name = "RTX5",
	.detect_rtos = rtx5_detect_rtos,
	.create = rtx5_create,
	.update_threads = rtx5_update_threads,
	.get_thread_reg_list = rtx5_get_thread_reg_list,
	.get_symbol_list_to_lookup = rtx5_get_symbol_list_to_lookup,
};
