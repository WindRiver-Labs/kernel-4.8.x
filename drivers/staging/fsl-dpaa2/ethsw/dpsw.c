/* Copyright 2013-2015 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the above-listed copyright holders nor the
 * names of any contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "../../fsl-mc/include/mc-sys.h"
#include "../../fsl-mc/include/mc-cmd.h"
#include "dpsw.h"
#include "dpsw-cmd.h"

/* internal functions */
static void build_if_id_bitmap(const u16 *if_id,
			       const u16 num_ifs,
			       struct mc_command *cmd,
			       int start_param)
{
	int i;

	for (i = 0; (i < num_ifs) && (i < DPSW_MAX_IF); i++)
		cmd->params[start_param + (if_id[i] / 64)] |= mc_enc(
			(if_id[i] % 64), 1, 1);
}

static int read_if_id_bitmap(u16 *if_id,
			     u16 *num_ifs,
			     struct mc_command *cmd,
			     int start_param)
{
	int bitmap[DPSW_MAX_IF] = { 0 };
	int i, j = 0;
	int count = 0;

	for (i = 0; i < DPSW_MAX_IF; i++) {
		bitmap[i] = (int)mc_dec(cmd->params[start_param + i / 64],
					 i % 64, 1);
		count += bitmap[i];
	}

	*num_ifs = (u16)count;

	for (i = 0; (i < DPSW_MAX_IF) && (j < count); i++) {
		if (bitmap[i]) {
			if_id[j] = (u16)i;
			j++;
		}
	}

	return 0;
}

/**
 * dpsw_open() - Open a control session for the specified object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @dpsw_id:	DPSW unique ID
 * @token:	Returned token; use in subsequent API calls
 *
 * This function can be used to open a control session for an
 * already created object; an object may have been declared in
 * the DPL or by calling the dpsw_create() function.
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent commands for
 * this specific object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_open(struct fsl_mc_io *mc_io,
	      u32 cmd_flags,
	      int dpsw_id,
	      u16 *token)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_OPEN,
					  cmd_flags,
					  0);
	DPSW_CMD_OPEN(cmd, dpsw_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return 0;
}

/**
 * dpsw_close() - Close the control session of the object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 *
 * After this function is called, no further operations are
 * allowed on the object without opening a new control session.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_close(struct fsl_mc_io *mc_io,
	       u32 cmd_flags,
	u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CLOSE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_create() - Create the DPSW object.
 * @mc_io:	Pointer to MC portal's I/O object
 * @dprc_token:	Parent container token; '0' for default container
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @cfg:	Configuration structure
 * @obj_id: returned object id
 *
 * Create the DPSW object, allocate required resources and
 * perform required initialization.
 *
 * The object can be created either by declaring it in the
 * DPL file, or by calling this function.
 *
 * The function accepts an authentication token of a parent
 * container that this object should be assigned to. The token
 * can be '0' so the object will be assigned to the default container.
 * The newly created object can be opened with the returned
 * object id and using the container's associated tokens and MC portals.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_create(struct fsl_mc_io	*mc_io,
		u16	dprc_token,
		u32	cmd_flags,
		const struct dpsw_cfg	*cfg,
		u32	*obj_id)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CREATE,
					  cmd_flags,
					  dprc_token);
	DPSW_CMD_CREATE(cmd, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*obj_id = get_mc_cmd_create_object_id(&cmd);

	return 0;
}

/**
 * dpsw_destroy() - Destroy the DPSW object and release all its resources.
 * @mc_io:	Pointer to MC portal's I/O object
 * @dprc_token: Parent container token; '0' for default container
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @object_id:	The object id; it must be a valid id within the container that
 * created this object;
 *
 * The function accepts the authentication token of the parent container that
 * created the object (not the one that currently owns the object). The object
 * is searched within parent using the provided 'object_id'.
 * All tokens to the object must be closed before calling destroy.
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpsw_destroy(struct fsl_mc_io	*mc_io,
		 u16	dprc_token,
		u32	cmd_flags,
		u32	object_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_DESTROY,
					  cmd_flags,
					  dprc_token);
	/* set object id to destroy */
	cmd.params[0] = mc_enc(0, sizeof(object_id), object_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_enable() - Enable DPSW functionality
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_enable(struct fsl_mc_io *mc_io,
		u32 cmd_flags,
		u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ENABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_disable() - Disable DPSW functionality
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_disable(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_is_enabled() - Check if the DPSW is enabled
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @en:		Returns '1' if object is enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise
 */
int dpsw_is_enabled(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    int *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IS_ENABLED, cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_IS_ENABLED(cmd, *en);

	return 0;
}

/**
 * dpsw_reset() - Reset the DPSW, returns the object to initial state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_reset(struct fsl_mc_io *mc_io,
	       u32 cmd_flags,
	       u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_RESET,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_set_irq() - Set IRQ information for the DPSW to trigger an interrupt.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @irq_index:	Identifies the interrupt index to configure
 * @irq_cfg:	IRQ configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_set_irq(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u8 irq_index,
		 struct dpsw_irq_cfg *irq_cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IRQ,
					  cmd_flags,
					  token);
	DPSW_CMD_SET_IRQ(cmd, irq_index, irq_cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_get_irq() - Get IRQ information from the DPSW
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @irq_index:	The interrupt index to configure
 * @type:	Interrupt type: 0 represents message interrupt
 *		type (both irq_addr and irq_val are valid)
 * @irq_cfg:	IRQ attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_get_irq(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u8 irq_index,
		 int *type,
		 struct dpsw_irq_cfg *irq_cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_IRQ,
					  cmd_flags,
					  token);
	DPSW_CMD_GET_IRQ(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_GET_IRQ(cmd, *type, irq_cfg);

	return 0;
}

/**
 * dpsw_set_irq_enable() - Set overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPCI object
 * @irq_index:	The interrupt index to configure
 * @en:			Interrupt state - enable = 1, disable = 0
 *
 * Allows GPP software to control when interrupts are generated.
 * Each interrupt can have up to 32 causes.  The enable/disable control's the
 * overall interrupt state. if the interrupt is disabled no causes will cause
 * an interrupt
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_set_irq_enable(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u8 irq_index,
			u8 en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPSW_CMD_SET_IRQ_ENABLE(cmd, irq_index, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_get_irq_enable() - Get overall interrupt state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @irq_index:	The interrupt index to configure
 * @en:			Returned Interrupt state - enable = 1, disable = 0
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_get_irq_enable(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u8 irq_index,
			u8 *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPSW_CMD_GET_IRQ_ENABLE(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_GET_IRQ_ENABLE(cmd, *en);

	return 0;
}

/**
 * dpsw_set_irq_mask() - Set interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPCI object
 * @irq_index:	The interrupt index to configure
 * @mask:		event mask to trigger interrupt;
 *				each bit:
 *					0 = ignore event
 *					1 = consider event for asserting IRQ
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_set_irq_mask(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      u8 irq_index,
		      u32 mask)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPSW_CMD_SET_IRQ_MASK(cmd, irq_index, mask);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_get_irq_mask() - Get interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @irq_index:	The interrupt index to configure
 * @mask:		Returned event mask to trigger interrupt
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_get_irq_mask(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      u8 irq_index,
		      u32 *mask)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPSW_CMD_GET_IRQ_MASK(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_GET_IRQ_MASK(cmd, *mask);

	return 0;
}

/**
 * dpsw_get_irq_status() - Get the current status of any pending interrupts
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @irq_index:	The interrupt index to configure
 * @status:		Returned interrupts status - one bit per cause:
 *					0 = no interrupt pending
 *					1 = interrupt pending
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_get_irq_status(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u8 irq_index,
			u32 *status)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPSW_CMD_GET_IRQ_STATUS(cmd, irq_index, *status);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_GET_IRQ_STATUS(cmd, *status);

	return 0;
}

/**
 * dpsw_clear_irq_status() - Clear a pending interrupt's status
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPCI object
 * @irq_index:	The interrupt index to configure
 * @status:		bits to clear (W1C) - one bit per cause:
 *					0 = don't change
 *					1 = clear status bit
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_clear_irq_status(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u8 irq_index,
			  u32 status)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CLEAR_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPSW_CMD_CLEAR_IRQ_STATUS(cmd, irq_index, status);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_get_attributes() - Retrieve DPSW attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @attr:		Returned DPSW attributes
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_get_attributes(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			struct dpsw_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_ATTR,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_GET_ATTR(cmd, attr);

	return 0;
}

/**
 * dpsw_set_reflection_if() - Set target interface for reflected interfaces.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Id
 *
 *	Only one reflection receive interface is allowed per switch
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_set_reflection_if(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_REFLECTION_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_SET_REFLECTION_IF(cmd, if_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_link_cfg() - set the link configuration.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token: Token of DPSW object
 * @if_id: interface id
 * @cfg: Link configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_if_set_link_cfg(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 if_id,
			 struct dpsw_link_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_LINK_CFG,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_LINK_CFG(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_get_link_state - Return the link state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token: Token of DPSW object
 * @if_id: interface id
 * @state: link state	1 - linkup, 0 - link down or disconnected
 *
 * @returns	'0' on Success; Error code otherwise.
 */
int dpsw_if_get_link_state(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   struct dpsw_link_state *state)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_LINK_STATE,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_GET_LINK_STATE(cmd, if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_IF_GET_LINK_STATE(cmd, state);

	return 0;
}

/**
 * dpsw_if_set_flooding() - Enable Disable flooding for particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @en:			1 - enable, 0 - disable
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_flooding(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 if_id,
			 int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_FLOODING,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_FLOODING(cmd, if_id, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_broadcast() - Enable/disable broadcast for particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @en:			1 - enable, 0 - disable
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_broadcast(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u16 if_id,
			  int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_BROADCAST,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_FLOODING(cmd, if_id, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_multicast() - Enable/disable multicast for particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @en:			1 - enable, 0 - disable
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_multicast(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u16 if_id,
			  int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_MULTICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_FLOODING(cmd, if_id, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_tci() - Set default VLAN Tag Control Information (TCI)
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @cfg:		Tag Control Information Configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_tci(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id,
		    const struct dpsw_tci_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_TCI,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_TCI(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_get_tci() - Get default VLAN Tag Control Information (TCI)
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @cfg:		Tag Control Information Configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_get_tci(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id,
		    struct dpsw_tci_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err = 0;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_TCI,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_GET_TCI(cmd, if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_IF_GET_TCI(cmd, cfg);

	return 0;
}

/**
 * dpsw_if_set_stp() - Function sets Spanning Tree Protocol (STP) state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @cfg:		STP State configuration parameters
 *
 * The following STP states are supported -
 * blocking, listening, learning, forwarding and disabled.
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_stp(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id,
		    const struct dpsw_stp_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_STP,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_STP(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_accepted_frames()
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @cfg:		Frame types configuration
 *
 * When is admit_only_vlan_tagged- the device will discard untagged
 * frames or Priority-Tagged frames received on this interface.
 * When admit_only_untagged- untagged frames or Priority-Tagged
 * frames received on this interface will be accepted and assigned
 * to a VID based on the PVID and VID Set for this interface.
 * When admit_all - the device will accept VLAN tagged, untagged
 * and priority tagged frames.
 * The default is admit_all
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_accepted_frames(struct fsl_mc_io *mc_io,
				u32 cmd_flags,
				u16 token,
				u16 if_id,
				const struct dpsw_accepted_frames_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_ACCEPTED_FRAMES,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_ACCEPTED_FRAMES(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_accept_all_vlan()
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @accept_all:	Accept or drop frames having different VLAN
 *
 * When this is accept (FALSE), the device will discard incoming
 * frames for VLANs that do not include this interface in its
 * Member set. When accept (TRUE), the interface will accept all incoming frames
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_accept_all_vlan(struct fsl_mc_io *mc_io,
				u32 cmd_flags,
				u16 token,
				u16 if_id,
				int accept_all)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IF_ACCEPT_ALL_VLAN,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_ACCEPT_ALL_VLAN(cmd, if_id, accept_all);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_get_counter() - Get specific counter of particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @type:		Counter type
 * @counter:	return value
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_get_counter(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u16 if_id,
			enum dpsw_counter type,
			u64 *counter)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_COUNTER,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_GET_COUNTER(cmd, if_id, type);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_IF_GET_COUNTER(cmd, *counter);

	return 0;
}

/**
 * dpsw_if_set_counter() - Set specific counter of particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @type:		Counter type
 * @counter:	New counter value
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_counter(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u16 if_id,
			enum dpsw_counter type,
			u64 counter)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_COUNTER,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_COUNTER(cmd, if_id, type, counter);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_tx_selection() - Function is used for mapping variety
 *				of frame fields
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @cfg:		Traffic class mapping configuration
 *
 * Function is used for mapping variety of frame fields (DSCP, PCP)
 * to Traffic Class. Traffic class is a number
 * in the range from 0 to 7
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_tx_selection(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     u16 if_id,
			     const struct dpsw_tx_selection_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_TX_SELECTION,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_TX_SELECTION(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_add_reflection() - Identify interface to be reflected or mirrored
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @cfg:		Reflection configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_add_reflection(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   const struct dpsw_reflection_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_ADD_REFLECTION,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_ADD_REFLECTION(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_remove_reflection() - Remove interface to be reflected or mirrored
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @cfg:		Reflection configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_remove_reflection(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 if_id,
			      const struct dpsw_reflection_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_REMOVE_REFLECTION,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_REMOVE_REFLECTION(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_flooding_metering() - Set flooding metering
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @cfg:	Metering parameters
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_flooding_metering(struct fsl_mc_io *mc_io,
				  u32 cmd_flags,
				  u16 token,
				  u16 if_id,
				  const struct dpsw_metering_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_FLOODING_METERING,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_FLOODING_METERING(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_metering() - Set interface metering for flooding
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @tc_id:	Traffic class ID
 * @cfg:	Metering parameters
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_metering(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 if_id,
			 u8 tc_id,
			 const struct dpsw_metering_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_METERING,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_METERING(cmd, if_id, tc_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_prepare_early_drop() - Prepare an early drop for setting in to interface
 * @cfg:	Early-drop configuration
 * @early_drop_buf: Zeroed 256 bytes of memory before mapping it to DMA
 *
 * This function has to be called before dpsw_if_tc_set_early_drop
 *
 */
void dpsw_prepare_early_drop(const struct dpsw_early_drop_cfg *cfg,
			     u8 *early_drop_buf)
{
	u64 *ext_params = (u64 *)early_drop_buf;

	DPSW_PREP_EARLY_DROP(ext_params, cfg);
}

/**
 * dpsw_if_set_early_drop() - Set interface traffic class early-drop
 *				configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:		Interface Identifier
 * @tc_id:	Traffic class selection (0-7)
 * @early_drop_iova:  I/O virtual address of 64 bytes;
 * Must be cacheline-aligned and DMA-able memory
 *
 * warning: Before calling this function, call dpsw_prepare_if_tc_early_drop()
 *		to prepare the early_drop_iova parameter
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpsw_if_set_early_drop(struct fsl_mc_io	*mc_io,
			   u32		cmd_flags,
			   u16		token,
			   u16		if_id,
			   u8		tc_id,
			   u64		early_drop_iova)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_EARLY_DROP,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_EARLY_DROP(cmd, if_id, tc_id, early_drop_iova);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_add_custom_tpid() - API Configures a distinct Ethernet type value
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @cfg:		Tag Protocol identifier
 *
 * API Configures a distinct Ethernet type value (or TPID value)
 * to indicate a VLAN tag in addition to the common
 * TPID values 0x8100 and 0x88A8.
 * Two additional TPID's are supported
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_add_custom_tpid(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 const struct dpsw_custom_tpid_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ADD_CUSTOM_TPID,
					  cmd_flags,
					  token);
	DPSW_CMD_ADD_CUSTOM_TPID(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_remove_custom_tpid - API removes a distinct Ethernet type value
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @cfg:		Tag Protocol identifier
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_remove_custom_tpid(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    const struct dpsw_custom_tpid_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_REMOVE_CUSTOM_TPID,
					  cmd_flags,
					  token);
	DPSW_CMD_REMOVE_CUSTOM_TPID(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_enable() - Enable Interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_enable(struct fsl_mc_io *mc_io,
		   u32 cmd_flags,
		   u16 token,
		   u16 if_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_ENABLE,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_ENABLE(cmd, if_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_disable() - Disable Interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_disable(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_DISABLE,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_DISABLE(cmd, if_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_get_attributes() - Function obtains attributes of interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @attr:	Returned interface attributes
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_get_attributes(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   struct dpsw_if_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_ATTR,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_GET_ATTR(cmd, if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_IF_GET_ATTR(cmd, attr);

	return 0;
}

/**
 * dpsw_if_set_max_frame_length() - Set Maximum Receive frame length.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @frame_length: Maximum Frame Length
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_max_frame_length(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 if_id,
				 u16 frame_length)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_MAX_FRAME_LENGTH,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_MAX_FRAME_LENGTH(cmd, if_id, frame_length);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_get_max_frame_length() - Get Maximum Receive frame length.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @frame_length: Returned maximum Frame Length
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_get_max_frame_length(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 if_id,
				 u16 *frame_length)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_MAX_FRAME_LENGTH,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_GET_MAX_FRAME_LENGTH(cmd, if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	DPSW_RSP_IF_GET_MAX_FRAME_LENGTH(cmd, *frame_length);

	return 0;
}

/**
 * dpsw_vlan_add() - Adding new VLAN to DPSW.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	VLAN configuration
 *
 * Only VLAN ID and FDB ID are required parameters here.
 * 12 bit VLAN ID is defined in IEEE802.1Q.
 * Adding a duplicate VLAN ID is not allowed.
 * FDB ID can be shared across multiple VLANs. Shared learning
 * is obtained by calling dpsw_vlan_add for multiple VLAN IDs
 * with same fdb_id
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_add(struct fsl_mc_io *mc_io,
		  u32 cmd_flags,
		  u16 token,
		  u16 vlan_id,
		  const struct dpsw_vlan_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_ADD(cmd, vlan_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_add_if() - Adding a set of interfaces to an existing VLAN.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Set of interfaces to add
 *
 * It adds only interfaces not belonging to this VLAN yet,
 * otherwise an error is generated and an entire command is
 * ignored. This function can be called numerous times always
 * providing required interfaces delta.
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_add_if(struct fsl_mc_io *mc_io,
		     u32 cmd_flags,
		     u16 token,
		     u16 vlan_id,
		     const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_ADD_IF(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_add_if_untagged() - Defining a set of interfaces that should be
 *				transmitted as untagged.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	set of interfaces that should be transmitted as untagged
 *
 * These interfaces should already belong to this VLAN.
 * By default all interfaces are transmitted as tagged.
 * Providing un-existing interface or untagged interface that is
 * configured untagged already generates an error and the entire
 * command is ignored.
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_add_if_untagged(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD_IF_UNTAGGED,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_ADD_IF_UNTAGGED(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_add_if_flooding() - Define a set of interfaces that should be
 *			included in flooding when frame with unknown destination
 *			unicast MAC arrived.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Set of interfaces that should be used for flooding
 *
 * These interfaces should belong to this VLAN. By default all
 * interfaces are included into flooding list. Providing
 * un-existing interface or an interface that already in the
 * flooding list generates an error and the entire command is
 * ignored.
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_add_if_flooding(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD_IF_FLOODING,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_ADD_IF_FLOODING(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_remove_if() - Remove interfaces from an existing VLAN.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Set of interfaces that should be removed
 *
 * Interfaces must belong to this VLAN, otherwise an error
 * is returned and an the command is ignored
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_remove_if(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u16 vlan_id,
			const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_REMOVE_IF(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_remove_if_untagged() - Define a set of interfaces that should be
 *		converted from transmitted as untagged to transmit as tagged.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	set of interfaces that should be removed
 *
 * Interfaces provided by API have to belong to this VLAN and
 * configured untagged, otherwise an error is returned and the
 * command is ignored
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_remove_if_untagged(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 vlan_id,
				 const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE_IF_UNTAGGED,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_REMOVE_IF_UNTAGGED(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_remove_if_flooding() - Define a set of interfaces that should be
 *			removed from the flooding list.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	set of interfaces used for flooding
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_remove_if_flooding(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 vlan_id,
				 const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE_IF_FLOODING,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_REMOVE_IF_FLOODING(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_remove() - Remove an entire VLAN
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_remove(struct fsl_mc_io *mc_io,
		     u32 cmd_flags,
		     u16 token,
		     u16 vlan_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_REMOVE(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_get_attributes() - Get VLAN attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @attr:	Returned DPSW attributes
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_get_attributes(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     u16 vlan_id,
			     struct dpsw_vlan_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_ATTRIBUTES,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_GET_ATTR(cmd, vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_VLAN_GET_ATTR(cmd, attr);

	return 0;
}

/**
 * dpsw_vlan_get_if() - Get interfaces belong to this VLAN
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Returned set of interfaces belong to this VLAN
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_get_if(struct fsl_mc_io *mc_io,
		     u32 cmd_flags,
		     u16 token,
		     u16 vlan_id,
		     struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_GET_IF(cmd, vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_VLAN_GET_IF(cmd, cfg);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, &cmd, 1);

	return 0;
}

/**
 * dpsw_vlan_get_if_flooding() - Get interfaces used in flooding for this VLAN
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Returned set of flooding interfaces
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */

int dpsw_vlan_get_if_flooding(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_IF_FLOODING,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_GET_IF_FLOODING(cmd, vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_VLAN_GET_IF_FLOODING(cmd, cfg);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, &cmd, 1);

	return 0;
}

/**
 * dpsw_vlan_get_if_untagged() - Get interfaces that should be transmitted as
 *				untagged
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Returned set of untagged interfaces
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_get_if_untagged(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_IF_UNTAGGED,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_GET_IF_UNTAGGED(cmd, vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_VLAN_GET_IF(cmd, cfg);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, &cmd, 1);

	return 0;
}

/**
 * dpsw_fdb_add() - Add FDB to switch and Returns handle to FDB table for
 *		the reference
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Returned Forwarding Database Identifier
 * @cfg:	FDB Configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_add(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u16 *fdb_id,
		 const struct dpsw_fdb_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_ADD,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_ADD(cmd, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_FDB_ADD(cmd, *fdb_id);

	return 0;
}

/**
 * dpsw_fdb_remove() - Remove FDB from switch
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_remove(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 fdb_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_REMOVE,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_REMOVE(cmd, fdb_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_add_unicast() - Function adds an unicast entry into MAC lookup table
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Unicast entry configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_add_unicast(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 fdb_id,
			 const struct dpsw_fdb_unicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_ADD_UNICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_ADD_UNICAST(cmd, fdb_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_get_unicast() - Get unicast entry from MAC lookup table by
 *		unicast Ethernet address
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Returned unicast entry configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_get_unicast(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 fdb_id,
			 struct dpsw_fdb_unicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_GET_UNICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_GET_UNICAST(cmd, fdb_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_FDB_GET_UNICAST(cmd, cfg);

	return 0;
}

/**
 * dpsw_fdb_remove_unicast() - removes an entry from MAC lookup table
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Unicast entry configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_remove_unicast(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    u16 fdb_id,
			    const struct dpsw_fdb_unicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_REMOVE_UNICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_REMOVE_UNICAST(cmd, fdb_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_add_multicast() - Add a set of egress interfaces to multi-cast group
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Multicast entry configuration
 *
 * If group doesn't exist, it will be created.
 * It adds only interfaces not belonging to this multicast group
 * yet, otherwise error will be generated and the command is
 * ignored.
 * This function may be called numerous times always providing
 * required interfaces delta.
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_add_multicast(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 fdb_id,
			   const struct dpsw_fdb_multicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 2);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_ADD_MULTICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_ADD_MULTICAST(cmd, fdb_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_get_multicast() - Reading multi-cast group by multi-cast Ethernet
 *				address.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Returned multicast entry configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_get_multicast(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 fdb_id,
			   struct dpsw_fdb_multicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_GET_MULTICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_GET_MULTICAST(cmd, fdb_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_FDB_GET_MULTICAST(cmd, cfg);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, &cmd, 2);

	return 0;
}

/**
 * dpsw_fdb_remove_multicast() - Removing interfaces from an existing multicast
 *				group.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Multicast entry configuration
 *
 * Interfaces provided by this API have to exist in the group,
 * otherwise an error will be returned and an entire command
 * ignored. If there is no interface left in the group,
 * an entire group is deleted
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_remove_multicast(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 fdb_id,
			      const struct dpsw_fdb_multicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 2);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_REMOVE_MULTICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_REMOVE_MULTICAST(cmd, fdb_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_set_learning_mode() - Define FDB learning mode
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @mode:	learning mode
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_set_learning_mode(struct fsl_mc_io *mc_io,
			       u32 cmd_flags,
			       u16 token,
			       u16 fdb_id,
			       enum dpsw_fdb_learning_mode mode)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_SET_LEARNING_MODE,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_SET_LEARNING_MODE(cmd, fdb_id, mode);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_get_attributes() - Get FDB attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @attr:	Returned FDB attributes
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_get_attributes(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    u16 fdb_id,
			    struct dpsw_fdb_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_GET_ATTR,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_GET_ATTR(cmd, fdb_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_FDB_GET_ATTR(cmd, attr);

	return 0;
}

/**
 * dpsw_acl_add() - Adds ACL to L2 switch.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	Returned ACL ID, for the future reference
 * @cfg:	ACL configuration
 *
 * Create Access Control List. Multiple ACLs can be created and
 * co-exist in L2 switch
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_add(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u16 *acl_id,
		 const struct dpsw_acl_cfg  *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_ADD,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_ADD(cmd, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_ACL_ADD(cmd, *acl_id);

	return 0;
}

/**
 * dpsw_acl_remove() - Removes ACL from L2 switch.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL ID
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_remove(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 acl_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_REMOVE,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_REMOVE(cmd, acl_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_acl_prepare_entry_cfg() - Set an entry to ACL.
 * @key:	key
 * @entry_cfg_buf: Zeroed 256 bytes of memory before mapping it to DMA
 *
 * This function has to be called before adding or removing acl_entry
 *
 */
void dpsw_acl_prepare_entry_cfg(const struct dpsw_acl_key *key,
				u8 *entry_cfg_buf)
{
	u64 *ext_params = (u64 *)entry_cfg_buf;

	DPSW_PREP_ACL_ENTRY(ext_params, key);
}

/**
 * dpsw_acl_add_entry() - Adds an entry to ACL.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL ID
 * @cfg:	entry configuration
 *
 * warning: This function has to be called after dpsw_acl_set_entry_cfg()
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_add_entry(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       u16 acl_id,
		       const struct dpsw_acl_entry_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_ADD_ENTRY,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_ADD_ENTRY(cmd, acl_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_acl_remove_entry() - Removes an entry from ACL.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL ID
 * @cfg:	entry configuration
 *
 * warning: This function has to be called after dpsw_acl_set_entry_cfg()
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_remove_entry(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u16 acl_id,
			  const struct dpsw_acl_entry_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_REMOVE_ENTRY,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_REMOVE_ENTRY(cmd, acl_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_acl_add_if() - Associate interface/interfaces with ACL.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL ID
 * @cfg:	interfaces list
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_add_if(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 acl_id,
		    const struct dpsw_acl_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_ADD_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_ADD_IF(cmd, acl_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_acl_remove_if() - De-associate interface/interfaces from ACL.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL ID
 * @cfg:	interfaces list
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_remove_if(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       u16 acl_id,
		       const struct dpsw_acl_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_REMOVE_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_REMOVE_IF(cmd, acl_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_acl_get_attributes() - Get specific counter of particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:      ACL Identifier
 * @attr:        Returned ACL attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_get_attributes(struct fsl_mc_io		*mc_io,
			    u32			cmd_flags,
			    u16			token,
			    u16			acl_id,
			    struct dpsw_acl_attr	*attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_GET_ATTR,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_GET_ATTR(cmd, acl_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_ACL_GET_ATTR(cmd, attr);

	return 0;
}

/**
 * dpsw_ctrl_if_get_attributes() - Obtain control interface attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @attr:	Returned control interface attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_ctrl_if_get_attributes(struct fsl_mc_io		*mc_io,
				u32			cmd_flags,
				u16			token,
				struct dpsw_ctrl_if_attr	*attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CTRL_IF_GET_ATTR,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_CTRL_IF_GET_ATTR(cmd, attr);

	return 0;
}

/**
 * dpsw_ctrl_if_set_pools() - Set control interface buffer pools
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @cfg:		buffer pools configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_ctrl_if_set_pools(struct fsl_mc_io			*mc_io,
			   u32				cmd_flags,
			   u16				token,
			   const struct dpsw_ctrl_if_pools_cfg *pools)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CTRL_IF_SET_POOLS,
					  cmd_flags,
					  token);
	DPSW_CMD_CTRL_IF_SET_POOLS(cmd, pools);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_ctrl_if_enable() - Enable control interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_ctrl_if_enable(struct fsl_mc_io	*mc_io,
			u32		cmd_flags,
			u16		token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CTRL_IF_ENABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_ctrl_if_disable() - Function disables control interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_ctrl_if_disable(struct fsl_mc_io	*mc_io,
			 u32		cmd_flags,
			 u16		token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CTRL_IF_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}
/**
 * dpsw_get_api_version() - Get Data Path Switch API version
 * @mc_io:  Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @major_ver:	Major version of data path switch API
 * @minor_ver:	Minor version of data path switch API
 *
 * Return:  '0' on Success; Error code otherwise.
 */
int dpsw_get_api_version(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 *major_ver,
			 u16 *minor_ver)
{
	struct mc_command cmd = { 0 };
	int err;

	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_API_VERSION,
					cmd_flags,
					0);

	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	DPSW_RSP_GET_API_VERSION(cmd, *major_ver, *minor_ver);

	return 0;
}
