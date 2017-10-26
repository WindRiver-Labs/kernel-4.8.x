/*
* Support for Intel Camera Imaging ISP subsystem.
 * Copyright (c) 2010 - 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
*/


#include "ia_css_psys_device.h"
#include "ia_css_psys_device_trace.h"
#include "ia_css_psys_init.h"

#include <error_support.h>
#include <assert_support.h>
#include <print_support.h>
#include <misc_support.h>

#include "ia_css_cell.h"

#define IA_CSS_PSYS_CMD_QUEUE_SIZE		0x20
#define IA_CSS_PSYS_EVENT_QUEUE_SIZE		0x40

static struct ia_css_syscom_queue_config
	ia_css_psys_cmd_queue_cfg[IA_CSS_N_PSYS_CMD_QUEUE_ID] = {
	{IA_CSS_PSYS_CMD_QUEUE_SIZE, IA_CSS_PSYS_CMD_BITS/8},
	{IA_CSS_PSYS_CMD_QUEUE_SIZE, IA_CSS_PSYS_CMD_BITS/8}
};

static struct ia_css_syscom_queue_config
	ia_css_psys_event_queue_cfg[IA_CSS_N_PSYS_EVENT_QUEUE_ID] = {
	{IA_CSS_PSYS_EVENT_QUEUE_SIZE, IA_CSS_PSYS_EVENT_BITS/8},
};

static struct ia_css_syscom_config psys_syscom_config;
struct ia_css_syscom_context	*psys_syscom;
static bool external_alloc = true;

int ia_css_psys_config_print(
	const struct ia_css_syscom_config *config,
	void *fh)
{
	int retval = -1;

	NOT_USED(fh);

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, INFO, "ia_css_frame_print(): enter:\n");

	verifexit(config != NULL, EINVAL);

	retval = 0;
EXIT:
	if (retval != 0) {
		IA_CSS_TRACE_1(PSYSAPI_DEVICE, ERROR,
			"ia_css_frame_print failed (%i)\n", retval);
	}
	return retval;
}

int ia_css_psys_print(
	const struct ia_css_syscom_context	*context,
	void					*fh)
{
	int	retval = -1;

	NOT_USED(fh);

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, INFO, "ia_css_psys_print(): enter:\n");

	verifexit(context != NULL, EINVAL);

	retval = 0;
EXIT:
	if (retval != 0) {
		IA_CSS_TRACE_1(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_print failed (%i)\n", retval);
	}
	return retval;
}

struct ia_css_syscom_config *ia_css_psys_specify(void)
{
	struct ia_css_syscom_config	*config = &psys_syscom_config;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, INFO, "ia_css_psys_specify(): enter:\n");

	config->num_input_queues = IA_CSS_N_PSYS_CMD_QUEUE_ID;
	config->num_output_queues = IA_CSS_N_PSYS_EVENT_QUEUE_ID;
	config->input = ia_css_psys_cmd_queue_cfg;
	config->output = ia_css_psys_event_queue_cfg;

	return config;
}

size_t ia_css_sizeof_psys(
	struct ia_css_syscom_config				*config)
{
	size_t	size = 0;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_sizeof_psys(): enter:\n");

	NOT_USED(config);

	return size;
}

struct ia_css_syscom_context *ia_css_psys_open(
	const struct ia_css_psys_buffer_s *buffer,
	struct ia_css_syscom_config *config)
{
	struct ia_css_syscom_context *context;
	ia_css_psys_server_init_t *server_config;
	int i;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, INFO, "ia_css_psys_open(): enter:\n");

	if (config == NULL)
		goto EXIT;

	if (buffer == NULL) {
		/* Allocate locally */
		external_alloc = false;
	}

	/*
	 * Here we would like to pass separately the sub-system ID
	 * and optionally the user pointer to be mapped, depending on
	 * where this open is called, and which virtual memory handles
	 * we see here.
	 */
	/* context = ia_css_syscom_open(get_virtual_memory_handle(vied_psys_ID),
	 * buffer, config);
	 */
	context = ia_css_syscom_open(config, NULL);
	if (context == NULL)
		goto EXIT;


	/* Configure SPC icache prefetching and start SPC */
	server_config = (ia_css_psys_server_init_t *)config->specific_addr;
	IA_CSS_TRACE_1(PSYSAPI_DEVICE, INFO, "SPC prefetch: %d\n",
		       server_config->icache_prefetch_sp);
	ia_css_cell_start_prefetch(config->ssid, SPC0,
				   server_config->icache_prefetch_sp);

	for (i = 0; i < IA_CSS_N_PSYS_CMD_QUEUE_ID; i++) {
		int retval;

		do {
			retval = ia_css_syscom_send_port_open(context,
							      (unsigned int)i);
		} while (retval == ERROR_BUSY);
		verifexit(retval == 0, EINVAL);
	}

	for (i = 0; i < IA_CSS_N_PSYS_EVENT_QUEUE_ID; i++) {
		int retval;

		do {
			retval = ia_css_syscom_recv_port_open(context,
							      (unsigned int)i);
		} while (retval == ERROR_BUSY);
		verifexit(retval == 0, EINVAL);
	}

	return context;

EXIT:
	IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR, "ia_css_psys_open failed\n");
	return NULL;
}

bool ia_css_psys_open_is_ready(
	struct ia_css_syscom_context			*context)
{
	int retval = -1;
	bool ready = 0;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, INFO,
		"ia_css_psys_open_is_ready(): enter:\n");
	verifexit(context != NULL, EINVAL);

	retval = 0;
	ready = 1;
EXIT:
	if (retval != 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_open_is_ready failed\n");
	}
	return ready;
}


struct ia_css_syscom_context *ia_css_psys_close(
	struct ia_css_syscom_context *context)
{
	/* Success: return NULL, Error: return context pointer value
	 * Intention is to change return type to int (errno),
	 * see commented values.
	 */

	unsigned int i;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, INFO, "ia_css_psys_close(): enter:\n");

	/* NULL pointer check disabled, since there is no proper return value */

	for (i = 0; i < IA_CSS_N_PSYS_CMD_QUEUE_ID; i++) {
		if (ia_css_syscom_send_port_close(context, i) != 0)
			return context; /* EINVAL */
	}

	for (i = 0; i < IA_CSS_N_PSYS_EVENT_QUEUE_ID; i++) {
		if (ia_css_syscom_recv_port_close(context, i) != 0)
			return context; /* EINVAL */
	}

	/* request device close */
	if (ia_css_syscom_close(context) != 0)
		return context; /* EBUSY */

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, INFO,
		"ia_css_psys_close(): leave: OK\n");
	return NULL;
}


int ia_css_psys_release(
	struct ia_css_syscom_context *context,
	bool force)
{
	if (context == NULL)
		return -EFAULT;

	/* try to free resources */
	if (ia_css_syscom_release(context, force) != 0)
		return -EBUSY;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, INFO,
		"ia_css_psys_release(): leave: OK\n");
	return 0;
}

ia_css_psys_state_t ia_css_psys_check_state(
	struct ia_css_syscom_context			*context)
{
	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_psys_check_state(): enter:\n");

	NOT_USED(context);

	/* For the time being, return the READY state to be used by SPC test */
	return IA_CSS_PSYS_STATE_READY;
}

bool ia_css_is_psys_cmd_queue_full(
	struct ia_css_syscom_context		*context,
	ia_css_psys_cmd_queue_ID_t		id)
{
	bool			is_full = false;
	int	num_tokens;
	int				retval = -1;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_is_psys_cmd_queue_full(): enter:\n");
	verifexit(context != NULL, EINVAL);

	num_tokens = ia_css_syscom_send_port_available(context,
						       (unsigned int)id);
	verifexit(num_tokens >= 0, EINVAL);

	is_full = (num_tokens == 0);
	retval = 0;
EXIT:
	if (retval != 0) {
		is_full = true;
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_is_psys_cmd_queue_full failed\n");
	}
	return is_full;
}

bool ia_css_is_psys_cmd_queue_not_full(
	struct ia_css_syscom_context		*context,
	ia_css_psys_cmd_queue_ID_t		id)
{
	bool			is_not_full = false;
	int	num_tokens;
	int				retval = -1;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_is_psys_cmd_queue_not_full(): enter:\n");
	verifexit(context != NULL, EINVAL);

	num_tokens = ia_css_syscom_send_port_available(context,
						       (unsigned int)id);
	verifexit(num_tokens >= 0, EINVAL);

	is_not_full = (num_tokens != 0);
	retval = 0;
EXIT:
	if (retval != 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_is_psys_cmd_queue_not_full failed\n");
	}
	return is_not_full;
}

bool ia_css_has_psys_cmd_queue_N_space(
	struct ia_css_syscom_context		*context,
	ia_css_psys_cmd_queue_ID_t		id,
	const unsigned int						N)
{
	bool			has_N_space = false;
	int	num_tokens;
	int				retval = -1;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_has_psys_cmd_queue_N_space(): enter:\n");
	verifexit(context != NULL, EINVAL);

	num_tokens = ia_css_syscom_send_port_available(context,
						       (unsigned int)id);
	verifexit(num_tokens >= 0, EINVAL);

	has_N_space = ((unsigned int)num_tokens >= N);
EXIT:
	if (retval != 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_has_psys_cmd_queue_N_space failed\n");
	}
	return has_N_space;
}

int ia_css_psys_cmd_queue_get_available_space(
	struct ia_css_syscom_context		*context,
	ia_css_psys_cmd_queue_ID_t		id)
{
	int				N_space = -1;
	int	num_tokens;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_psys_cmd_queue_get_available_space(): enter:\n");
	verifexit(context != NULL, EINVAL);

	num_tokens = ia_css_syscom_send_port_available(context,
						       (unsigned int)id);
	verifexit(num_tokens >= 0, EINVAL);

	N_space = (int)(num_tokens);
EXIT:
	if (N_space < 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_cmd_queue_get_available_space failed\n");
	}
	return N_space;
}

bool ia_css_any_psys_event_queue_not_empty(
	struct ia_css_syscom_context		*context)
{
	ia_css_psys_event_queue_ID_t	i;
	bool	any_msg = false;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_any_psys_event_queue_not_empty(): enter:\n");
	verifexit(context != NULL, EINVAL);

	for (i = (ia_css_psys_event_queue_ID_t)0;
		i < IA_CSS_N_PSYS_EVENT_QUEUE_ID; i++) {
		any_msg =
		    any_msg || ia_css_is_psys_event_queue_not_empty(context, i);
	}

EXIT:
	return any_msg;
}

bool ia_css_is_psys_event_queue_empty(
	struct ia_css_syscom_context		*context,
	ia_css_psys_event_queue_ID_t		id)
{
	bool			is_empty = false;
	int	num_tokens;
	int				retval = -1;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_is_psys_event_queue_empty(): enter:\n");
	verifexit(context != NULL, EINVAL);

	num_tokens = ia_css_syscom_recv_port_available(context,
						       (unsigned int)id);
	verifexit(num_tokens >= 0, EINVAL);

	is_empty = (num_tokens == 0);
	retval = 0;
EXIT:
	if (retval != 0) {
		is_empty = true;
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			       "ia_css_is_psys_event_queue_empty failed\n");
	}
	return is_empty;
}

bool ia_css_is_psys_event_queue_not_empty(
	struct ia_css_syscom_context		*context,
	ia_css_psys_event_queue_ID_t		id)
{
	bool			is_not_empty = false;
	int	num_tokens;
	int				retval = -1;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_is_psys_event_queue_not_empty(): enter:\n");
	verifexit(context != NULL, EINVAL);

	num_tokens = ia_css_syscom_recv_port_available(context,
			(unsigned int)id);
	verifexit(num_tokens >= 0, EINVAL);

	is_not_empty = (num_tokens != 0);
	retval = 0;
EXIT:
	if (retval != 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_is_psys_event_queue_not_empty failed\n");
	}
	return is_not_empty;
}

bool ia_css_has_psys_event_queue_N_msgs(
	struct ia_css_syscom_context		*context,
	ia_css_psys_event_queue_ID_t		id,
	const unsigned int						N)
{
	bool			has_N_msgs = false;
	int	num_tokens;
	int				retval = -1;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_has_psys_event_queue_N_msgs(): enter:\n");
	verifexit(context != NULL, EINVAL);

	num_tokens = ia_css_syscom_recv_port_available(context,
						       (unsigned int)id);
	verifexit(num_tokens >= 0, EINVAL);

	has_N_msgs = ((unsigned int)num_tokens >= N);
	retval = 0;
EXIT:
	if (retval != 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_has_psys_event_queue_N_msgs failed\n");
	}
	return has_N_msgs;
}

int ia_css_psys_event_queue_get_available_msgs(
	struct ia_css_syscom_context		*context,
	ia_css_psys_event_queue_ID_t		id)
{
	int				N_msgs = -1;
	int	num_tokens;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_psys_event_queue_get_available_msgs(): enter:\n");
	verifexit(context != NULL, EINVAL);

	num_tokens = ia_css_syscom_recv_port_available(context,
						       (unsigned int)id);
	verifexit(num_tokens >= 0, EINVAL);

	N_msgs = (int)(num_tokens);
EXIT:
	if (N_msgs < 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_event_queue_get_available_msgs failed\n");
	}
	return N_msgs;
}

int ia_css_psys_cmd_queue_send(
	struct ia_css_syscom_context			*context,
	ia_css_psys_cmd_queue_ID_t		id,
	const void *cmd_msg_buffer)
{
	int	count = 0;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_psys_cmd_queue_send(): enter:\n");
	verifexit(context != NULL, EINVAL);

	verifexit(context != NULL, EINVAL);
	/* The ~full check fails on receive queues */
	verifexit(ia_css_is_psys_cmd_queue_not_full(context, id), EBUSY);
	verifexit(cmd_msg_buffer != NULL, EINVAL);

	verifexit(ia_css_syscom_send_port_transfer(context, (unsigned int)id,
			cmd_msg_buffer) >= 0, EBUSY);

	count = 1;
EXIT:
	if (count == 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_cmd_queue_send failed\n");
	}
	return count;
}

int ia_css_psys_cmd_queue_send_N(
	struct ia_css_syscom_context			*context,
	ia_css_psys_cmd_queue_ID_t		id,
	const void *cmd_msg_buffer,
	const unsigned int						N)
{
	struct ia_css_psys_cmd_s *cmd_msg_buffer_loc =
				     (struct ia_css_psys_cmd_s *)cmd_msg_buffer;
	int	count = 0;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_psys_cmd_queue_send_N(): enter:\n");
	verifexit(context != NULL, EINVAL);

	for (count = 0; count < (int)N; count++) {
		int count_loc = ia_css_psys_cmd_queue_send(context, id,
					(void *)(&cmd_msg_buffer_loc[count]));

		verifexit(count_loc == 1, EINVAL);
	}

EXIT:
	if ((unsigned int) count < N) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_cmd_queue_send_N failed\n");
	}
	return count;
}

int ia_css_psys_event_queue_receive(
	struct ia_css_syscom_context			*context,
	ia_css_psys_event_queue_ID_t			id,
	void *event_msg_buffer)
{
	int	count = 0;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_psys_event_queue_receive(): enter:\n");

	verifexit(context != NULL, EINVAL);
	/* The ~empty check fails on send queues */
	verifexit(ia_css_is_psys_event_queue_not_empty(context, id), EBUSY);
	verifexit(event_msg_buffer != NULL, EINVAL);

	verifexit(ia_css_syscom_recv_port_transfer(context, (unsigned int)id,
			event_msg_buffer) >= 0, EBUSY);

	count = 1;
EXIT:
	if (count == 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_event_queue_receive failed\n");
	}
	return count;
}

int ia_css_psys_event_queue_receive_N(
	struct ia_css_syscom_context			*context,
	ia_css_psys_event_queue_ID_t		id,
	void *event_msg_buffer,
	const unsigned int						N)
{
	struct ia_css_psys_event_s	*event_msg_buffer_loc;
	int	count;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		       "ia_css_psys_event_queue_receive_N(): enter:\n");

	event_msg_buffer_loc = (struct ia_css_psys_event_s *)event_msg_buffer;

	for (count = 0; count < (int)N; count++) {
		int	count_loc = ia_css_psys_event_queue_receive(context, id,
				    (void *)(&event_msg_buffer_loc[count]));

		verifexit(count_loc == 1, EINVAL);
	}

EXIT:
	if ((unsigned int) count < N) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_event_queue_receive_N failed\n");
	}
	return count;
}

size_t ia_css_psys_get_size(
	const struct ia_css_syscom_context		*context)
{
	size_t	size = 0;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		"ia_css_psys_get_size(): enter:\n");

	verifexit(context != NULL, EINVAL);
	/* How can I query the context ? */
EXIT:
	if (size == 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_get_size failed\n");
	}
	return size;
}

unsigned int ia_css_psys_get_cmd_queue_count(
	const struct ia_css_syscom_context		*context)
{
	unsigned int	count = 0;
	int			retval = -1;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		       "ia_css_psys_get_cmd_queue_count(): enter:\n");

	verifexit(context != NULL, EINVAL);
	/* How can I query the context ? */
	NOT_USED(context);
	count = (unsigned int)IA_CSS_N_PSYS_CMD_QUEUE_ID;
	retval = 0;
EXIT:
	if (retval != 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_get_cmd_queue_count failed\n");
	}
	return count;
}

unsigned int ia_css_psys_get_event_queue_count(
	const struct ia_css_syscom_context		*context)
{
	unsigned int	count = 0;
	int				retval = -1;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		       "ia_css_psys_get_event_queue_count(): enter:\n");

	verifexit(context != NULL, EINVAL);
	/* How can I query the context ? */
	NOT_USED(context);
	count = (unsigned int)IA_CSS_N_PSYS_EVENT_QUEUE_ID;
	retval = 0;
EXIT:
	if (retval != 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_get_event_queue_count failed\n");
	}
	return count;
}

size_t ia_css_psys_get_cmd_queue_size(
	const struct ia_css_syscom_context		*context,
	ia_css_psys_cmd_queue_ID_t		id)
{
	size_t	queue_size = 0;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		       "ia_css_psys_get_cmd_queue_size(): enter:\n");

	verifexit(context != NULL, EINVAL);
	/* How can I query the context ? */
	NOT_USED(context);
	queue_size = ia_css_psys_cmd_queue_cfg[id].queue_size;
EXIT:
	if (queue_size == 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_get_cmd_queue_size failed\n");
	}
	return queue_size;
}

size_t ia_css_psys_get_event_queue_size(
	const struct ia_css_syscom_context		*context,
	ia_css_psys_event_queue_ID_t		id)
{
	size_t	queue_size = 0;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		       "ia_css_psys_get_event_queue_size(): enter:\n");

	verifexit(context != NULL, EINVAL);
	/* How can I query the context ? */
	NOT_USED(context);
	queue_size = ia_css_psys_event_queue_cfg[id].queue_size;
EXIT:
	if (queue_size == 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_get_event_queue_size failed\n");
	}
	return queue_size;
}

size_t ia_css_psys_get_cmd_msg_size(
	const struct ia_css_syscom_context		*context,
	ia_css_psys_cmd_queue_ID_t		id)
{
	size_t	msg_size = 0;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		       "ia_css_psys_get_cmd_msg_size(): enter:\n");

	verifexit(context != NULL, EINVAL);
	/* How can I query the context ? */
	NOT_USED(context);
	msg_size = ia_css_psys_cmd_queue_cfg[id].token_size;
EXIT:
	if (msg_size == 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_get_cmd_msg_size failed\n");
	}
	return msg_size;
}

size_t ia_css_psys_get_event_msg_size(
	const struct ia_css_syscom_context		*context,
	ia_css_psys_event_queue_ID_t		id)
{
	size_t	msg_size = 0;

	IA_CSS_TRACE_0(PSYSAPI_DEVICE, VERBOSE,
		       "ia_css_psys_get_event_msg_size(): enter:\n");

	verifexit(context != NULL, EINVAL);
	/* How can I query the context ? */
	NOT_USED(context);
	msg_size = ia_css_psys_event_queue_cfg[id].token_size;
EXIT:
	if (msg_size == 0) {
		IA_CSS_TRACE_0(PSYSAPI_DEVICE, ERROR,
			"ia_css_psys_get_cmd_msg_size failed\n");
	}
	return msg_size;
}

