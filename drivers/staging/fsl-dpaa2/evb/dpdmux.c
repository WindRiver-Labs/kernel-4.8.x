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
#include "dpdmux.h"
#include "dpdmux-cmd.h"

/**
 * dpdmux_open() - Open a control session for the specified object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @dpdmux_id:		DPDMUX unique ID
 * @token:		Returned token; use in subsequent API calls
 *
 * This function can be used to open a control session for an
 * already created object; an object may have been declared in
 * the DPL or by calling the dpdmux_create() function.
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent commands for
 * this specific object.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_open(struct fsl_mc_io *mc_io,
		u32 cmd_flags,
		int dpdmux_id,
		u16 *token)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_OPEN,
					  cmd_flags,
					  0);
	DPDMUX_CMD_OPEN(cmd, dpdmux_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return 0;
}

/**
 * dpdmux_close() - Close the control session of the object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPDMUX object
 *
 * After this function is called, no further operations are
 * allowed on the object without opening a new control session.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_close(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_CLOSE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_create() - Create the DPDMUX object
 * @mc_io:	Pointer to MC portal's I/O object
 * @dprc_token:	Parent container token; '0' for default container
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @cfg:	Configuration structure
 * @obj_id: returned object id
 *
 * Create the DPDMUX object, allocate required resources and
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
int dpdmux_create(struct fsl_mc_io	*mc_io,
		  u16	dprc_token,
		u32	cmd_flags,
		const struct dpdmux_cfg	*cfg,
		u32 *obj_id)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_CREATE,
					  cmd_flags,
					  dprc_token);
	DPDMUX_CMD_CREATE(cmd, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*obj_id = get_mc_cmd_create_object_id(&cmd);

	return 0;
}

/**
 * dpdmux_destroy() - Destroy the DPDMUX object and release all its resources.
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
int dpdmux_destroy(struct fsl_mc_io *mc_io,
		   u16 dprc_token,
		u32 cmd_flags,
		u32 object_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_DESTROY,
					  cmd_flags,
					  dprc_token);
	/* set object id to destroy */
	cmd.params[0] = mc_enc(0, sizeof(object_id), object_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_enable() - Enable DPDMUX functionality
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_enable(struct fsl_mc_io *mc_io,
		  u32 cmd_flags,
		  u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_ENABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_disable() - Disable DPDMUX functionality
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_disable(struct fsl_mc_io *mc_io,
		   u32 cmd_flags,
		   u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_is_enabled() - Check if the DPDMUX is enabled.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @en:		Returns '1' if object is enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_is_enabled(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      int *en)
{
	struct mc_command cmd = { 0 };
	int err;
	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_IS_ENABLED,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMUX_RSP_IS_ENABLED(cmd, *en);

	return 0;
}

/**
 * dpdmux_reset() - Reset the DPDMUX, returns the object to initial state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_reset(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_RESET,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_set_irq() - Set IRQ information for the DPDMUX to trigger an interrupt
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	Identifies the interrupt index to configure
 * @irq_cfg:	IRQ configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_set_irq(struct fsl_mc_io	*mc_io,
		   u32		cmd_flags,
		   u16		token,
		   u8		irq_index,
		   struct dpdmux_irq_cfg *irq_cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_SET_IRQ,
					  cmd_flags,
					  token);
	DPDMUX_CMD_SET_IRQ(cmd, irq_index, irq_cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_get_irq() - Get IRQ information from the DPDMUX.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @type:	Interrupt type: 0 represents message interrupt
 *		type (both irq_addr and irq_val are valid)
 * @irq_cfg:	IRQ attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_get_irq(struct fsl_mc_io *mc_io,
		   u32 cmd_flags,
		   u16		token,
		   u8		irq_index,
		   int			*type,
		   struct dpdmux_irq_cfg *irq_cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_GET_IRQ,
					  cmd_flags,
					  token);
	DPDMUX_CMD_GET_IRQ(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMUX_RSP_GET_IRQ(cmd, *type, irq_cfg);

	return 0;
}

/**
 * dpdmux_set_irq_enable() - Set overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @en:		Interrupt state - enable = 1, disable = 0
 *
 * Allows GPP software to control when interrupts are generated.
 * Each interrupt can have up to 32 causes.  The enable/disable control's the
 * overall interrupt state. if the interrupt is disabled no causes will cause
 * an interrupt.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_set_irq_enable(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u8 irq_index,
			  u8 en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_SET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPDMUX_CMD_SET_IRQ_ENABLE(cmd, irq_index, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_get_irq_enable() - Get overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @en:		Returned interrupt state - enable = 1, disable = 0
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_get_irq_enable(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u8 irq_index,
			  u8 *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_GET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPDMUX_CMD_GET_IRQ_ENABLE(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMUX_RSP_GET_IRQ_ENABLE(cmd, *en);

	return 0;
}

/**
 * dpdmux_set_irq_mask() - Set interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @mask:	event mask to trigger interrupt;
 *		each bit:
 *			0 = ignore event
 *			1 = consider event for asserting IRQ
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_set_irq_mask(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u8 irq_index,
			u32 mask)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_SET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPDMUX_CMD_SET_IRQ_MASK(cmd, irq_index, mask);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_get_irq_mask() - Get interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @mask:	Returned event mask to trigger interrupt
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_get_irq_mask(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u8 irq_index,
			u32 *mask)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_GET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPDMUX_CMD_GET_IRQ_MASK(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMUX_RSP_GET_IRQ_MASK(cmd, *mask);

	return 0;
}

/**
 * dpdmux_get_irq_status() - Get the current status of any pending interrupts.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @status:	Returned interrupts status - one bit per cause:
 *			0 = no interrupt pending
 *			1 = interrupt pending
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_get_irq_status(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u8 irq_index,
			  u32 *status)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_GET_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPDMUX_CMD_GET_IRQ_STATUS(cmd, irq_index, *status);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMUX_RSP_GET_IRQ_STATUS(cmd, *status);

	return 0;
}

/**
 * dpdmux_clear_irq_status() - Clear a pending interrupt's status
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @status:	bits to clear (W1C) - one bit per cause:
 *			0 = don't change
 *			1 = clear status bit
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_clear_irq_status(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    u8 irq_index,
			    u32 status)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_CLEAR_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPDMUX_CMD_CLEAR_IRQ_STATUS(cmd, irq_index, status);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_get_attributes() - Retrieve DPDMUX attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @attr:	Returned object's attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_get_attributes(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  struct dpdmux_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_GET_ATTR,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMUX_RSP_GET_ATTR(cmd, attr);

	return 0;
}

/**
 * dpdmux_set_max_frame_length() - Set the maximum frame length in DPDMUX
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPDMUX object
 * @max_frame_length:	The required maximum frame length
 *
 * Update the maximum frame length on all DMUX interfaces.
 * In case of VEPA, the maximum frame length on all dmux interfaces
 * will be updated with the minimum value of the mfls of the connected
 * dpnis and the actual value of dmux mfl.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_set_max_frame_length(struct fsl_mc_io *mc_io,
				u32 cmd_flags,
				   u16 token,
				   u16 max_frame_length)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_SET_MAX_FRAME_LENGTH,
					  cmd_flags,
					  token);
	DPDMUX_CMD_SET_MAX_FRAME_LENGTH(cmd, max_frame_length);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
* dpdmux_ul_reset_counters() - Function resets the uplink counter
* @mc_io:	Pointer to MC portal's I/O object
* @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
* @token:	Token of DPDMUX object
*
* Return:	'0' on Success; Error code otherwise.
*/
int dpdmux_ul_reset_counters(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_UL_RESET_COUNTERS,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_if_set_accepted_frames() - Set the accepted frame types
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @if_id:	Interface ID (0 for uplink, or 1-num_ifs);
 * @cfg:	Frame types configuration
 *
 * if 'DPDMUX_ADMIT_ONLY_VLAN_TAGGED' is set - untagged frames or
 * priority-tagged frames are discarded.
 * if 'DPDMUX_ADMIT_ONLY_UNTAGGED' is set - untagged frames or
 * priority-tagged frames are accepted.
 * if 'DPDMUX_ADMIT_ALL' is set (default mode) - all VLAN tagged,
 * untagged and priority-tagged frame are accepted;
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_if_set_accepted_frames(struct fsl_mc_io *mc_io,
				  u32 cmd_flags,
				  u16 token,
				  u16 if_id,
				  const struct dpdmux_accepted_frames *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_IF_SET_ACCEPTED_FRAMES,
					  cmd_flags,
					  token);
	DPDMUX_CMD_IF_SET_ACCEPTED_FRAMES(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_if_get_attributes() - Obtain DPDMUX interface attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @if_id:	Interface ID (0 for uplink, or 1-num_ifs);
 * @attr:	Interface attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_if_get_attributes(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     u16 if_id,
			     struct dpdmux_if_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_IF_GET_ATTR,
					  cmd_flags,
					  token);
	DPDMUX_CMD_IF_GET_ATTR(cmd, if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMUX_RSP_IF_GET_ATTR(cmd, attr);

	return 0;
}

/**
 * dpdmux_if_remove_l2_rule() - Remove L2 rule from DPDMUX table
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @if_id:	Destination interface ID
 * @rule:	L2 rule
 *
 * Function removes a L2 rule from DPDMUX table
 * or adds an interface to an existing multicast address
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_if_remove_l2_rule(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     u16 if_id,
			     const struct dpdmux_l2_rule *rule)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_IF_REMOVE_L2_RULE,
					  cmd_flags,
					  token);
	DPDMUX_CMD_IF_REMOVE_L2_RULE(cmd, if_id, rule);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_if_add_l2_rule() - Add L2 rule into DPDMUX table
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @if_id:	Destination interface ID
 * @rule:	L2 rule
 *
 * Function adds a L2 rule into DPDMUX table
 * or adds an interface to an existing multicast address
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_if_add_l2_rule(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u16 if_id,
			  const struct dpdmux_l2_rule *rule)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_IF_ADD_L2_RULE,
					  cmd_flags,
					  token);
	DPDMUX_CMD_IF_ADD_L2_RULE(cmd, if_id, rule);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
* dpdmux_if_get_counter() - Functions obtains specific counter of an interface
* @mc_io: Pointer to MC portal's I/O object
* @cmd_flags: Command flags; one or more of 'MC_CMD_FLAG_'
* @token: Token of DPDMUX object
* @if_id:  Interface Id
* @counter_type: counter type
* @counter: Returned specific counter information
*
* Return:	'0' on Success; Error code otherwise.
*/
int dpdmux_if_get_counter(struct fsl_mc_io		*mc_io,
			  u32			cmd_flags,
			  u16			token,
			  u16			if_id,
			  enum dpdmux_counter_type	counter_type,
			  u64			*counter)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_IF_GET_COUNTER,
					  cmd_flags,
					  token);
	DPDMUX_CMD_IF_GET_COUNTER(cmd, if_id, counter_type);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMUX_RSP_IF_GET_COUNTER(cmd, *counter);

	return 0;
}

/**
 * dpdmux_if_set_link_cfg() - set the link configuration.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token: Token of DPSW object
 * @if_id: interface id
 * @cfg: Link configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_if_set_link_cfg(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   struct dpdmux_link_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_IF_SET_LINK_CFG,
					  cmd_flags,
					  token);
	DPDMUX_CMD_IF_SET_LINK_CFG(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpdmux_if_get_link_state - Return the link state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token: Token of DPSW object
 * @if_id: interface id
 * @state: link state
 *
 * @returns	'0' on Success; Error code otherwise.
 */
int dpdmux_if_get_link_state(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     u16 if_id,
			     struct dpdmux_link_state *state)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_IF_GET_LINK_STATE,
					  cmd_flags,
					  token);
	DPDMUX_CMD_IF_GET_LINK_STATE(cmd, if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMUX_RSP_IF_GET_LINK_STATE(cmd, state);

	return 0;
}

/**
 * dpdmux_get_api_version() - Get Data Path Demux API version
 * @mc_io:  Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @major_ver:	Major version of data path demux API
 * @minor_ver:	Minor version of data path demux API
 *
 * Return:  '0' on Success; Error code otherwise.
 */
int dpdmux_get_api_version(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 *major_ver,
			   u16 *minor_ver)
{
	struct mc_command cmd = { 0 };
	int err;

	cmd.header = mc_encode_cmd_header(DPDMUX_CMDID_GET_API_VERSION,
					cmd_flags,
					0);

	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	DPDMUX_RSP_GET_API_VERSION(cmd, *major_ver, *minor_ver);

	return 0;
}
