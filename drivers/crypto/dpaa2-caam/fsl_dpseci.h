/* Copyright 2013-2016 Freescale Semiconductor Inc.
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
#ifndef __FSL_DPSECI_H
#define __FSL_DPSECI_H

/* Data Path SEC Interface API
 * Contains initialization APIs and runtime control APIs for DPSECI
 */

struct fsl_mc_io;
struct opr_cfg;
struct opr_qry;

/**
 * General DPSECI macros
 */

/**
 * Maximum number of Tx/Rx priorities per DPSECI object
 */
#define DPSECI_PRIO_NUM		8

/**
 * All queues considered; see dpseci_set_rx_queue()
 */
#define DPSECI_ALL_QUEUES	(uint8_t)(-1)

/**
 * dpseci_open() - Open a control session for the specified object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @dpseci_id:	DPSECI unique ID
 * @token:	Returned token; use in subsequent API calls
 *
 * This function can be used to open a control session for an
 * already created object; an object may have been declared in
 * the DPL or by calling the dpseci_create() function.
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent commands for
 * this specific object.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_open(struct fsl_mc_io	*mc_io,
		uint32_t		cmd_flags,
		int			dpseci_id,
		uint16_t		*token);

/**
 * dpseci_close() - Close the control session of the object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 *
 * After this function is called, no further operations are
 * allowed on the object without opening a new control session.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_close(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token);

/**
 * Enable the Order Restoration support
 */
#define DPSECI_OPT_HAS_OPR					0x000040

/**
 * Order Point Records are shared for the entire DPSECI
 */
#define DPSECI_OPT_OPR_SHARED				0x000080

/**
 * struct dpseci_cfg - Structure representing DPSECI configuration
 * @options: Any combination of the following options:
 *		DPSECI_OPT_HAS_OPR
 *		DPSECI_OPT_OPR_SHARED
 * @num_tx_queues: num of queues towards the SEC
 * @num_rx_queues: num of queues back from the SEC
 * @priorities: Priorities for the SEC hardware processing;
 *		each place in the array is the priority of the tx queue
 *		towards the SEC,
 *		valid priorities are configured with values 1-8;
 */
struct dpseci_cfg {
	uint32_t options;
	uint8_t num_tx_queues;
	uint8_t num_rx_queues;
	uint8_t priorities[DPSECI_PRIO_NUM];
};

/**
 * dpseci_create() - Create the DPSECI object
 * @mc_io:	Pointer to MC portal's I/O object
 * @dprc_token:	Parent container token; '0' for default container
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @cfg:	Configuration structure
 * @obj_id: returned object id
 *
 * Create the DPSECI object, allocate required resources and
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
int dpseci_create(struct fsl_mc_io		*mc_io,
		  uint16_t			dprc_token,
		  uint32_t			cmd_flags,
		  const struct dpseci_cfg	*cfg,
		  uint32_t			*obj_id);

/**
 * dpseci_destroy() - Destroy the DPSECI object and release all its resources.
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
int dpseci_destroy(struct fsl_mc_io	*mc_io,
		   uint16_t		dprc_token,
		   uint32_t		cmd_flags,
		   uint32_t		object_id);

/**
 * dpseci_enable() - Enable the DPSECI, allow sending and receiving frames.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_enable(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		  uint16_t		token);

/**
 * dpseci_disable() - Disable the DPSECI, stop sending and receiving frames.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_disable(struct fsl_mc_io	*mc_io,
		   uint32_t		cmd_flags,
		   uint16_t		token);

/**
 * dpseci_is_enabled() - Check if the DPSECI is enabled.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @en:		Returns '1' if object is enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_is_enabled(struct fsl_mc_io	*mc_io,
		      uint32_t		cmd_flags,
		      uint16_t		token,
		      int		*en);

/**
 * dpseci_reset() - Reset the DPSECI, returns the object to initial state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_reset(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token);

/**
 * dpseci_set_irq_enable() - Set overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSECI object
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
int dpseci_set_irq_enable(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint8_t		en);

/**
 * dpseci_get_irq_enable() - Get overall interrupt state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSECI object
 * @irq_index:	The interrupt index to configure
 * @en:			Returned Interrupt state - enable = 1, disable = 0
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_get_irq_enable(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint8_t		*en);

/**
 * dpseci_set_irq_mask() - Set interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSECI object
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
int dpseci_set_irq_mask(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t			irq_index,
			uint32_t		mask);

/**
 * dpseci_get_irq_mask() - Get interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSECI object
 * @irq_index:	The interrupt index to configure
 * @mask:		Returned event mask to trigger interrupt
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_get_irq_mask(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t			irq_index,
			uint32_t		*mask);

/**
 * dpseci_get_irq_status() - Get the current status of any pending interrupts
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSECI object
 * @irq_index:	The interrupt index to configure
 * @status:		Returned interrupts status - one bit per cause:
 *					0 = no interrupt pending
 *					1 = interrupt pending
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_get_irq_status(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint32_t		*status);

/**
 * dpseci_clear_irq_status() - Clear a pending interrupt's status
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSECI object
 * @irq_index:	The interrupt index to configure
 * @status:		bits to clear (W1C) - one bit per cause:
 *					0 = don't change
 *					1 = clear status bit
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_clear_irq_status(struct fsl_mc_io	*mc_io,
			    uint32_t		cmd_flags,
			    uint16_t		token,
			    uint8_t		irq_index,
			    uint32_t		status);

/**
 * struct dpseci_attr - Structure representing DPSECI attributes
 * @id: DPSECI object ID
 * @num_tx_queues: number of queues towards the SEC
 * @num_rx_queues: number of queues back from the SEC
 */
struct dpseci_attr {
	int	id;
	uint8_t	num_tx_queues;
	uint8_t	num_rx_queues;
};

/**
 * dpseci_get_attributes() - Retrieve DPSECI attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @attr:	Returned object's attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_get_attributes(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  struct dpseci_attr	*attr);

/**
 * enum dpseci_dest - DPSECI destination types
 * @DPSECI_DEST_NONE: Unassigned destination; The queue is set in parked mode
 *		and does not generate FQDAN notifications; user is expected to
 *		dequeue from the queue based on polling or other user-defined
 *		method
 * @DPSECI_DEST_DPIO: The queue is set in schedule mode and generates FQDAN
 *		notifications to the specified DPIO; user is expected to dequeue
 *		from the queue only after notification is received
 * @DPSECI_DEST_DPCON: The queue is set in schedule mode and does not generate
 *		FQDAN notifications, but is connected to the specified DPCON
 *		object; user is expected to dequeue from the DPCON channel
 */
enum dpseci_dest {
	DPSECI_DEST_NONE = 0,
	DPSECI_DEST_DPIO = 1,
	DPSECI_DEST_DPCON = 2
};

/**
 * struct dpseci_dest_cfg - Structure representing DPSECI destination parameters
 * @dest_type: Destination type
 * @dest_id: Either DPIO ID or DPCON ID, depending on the destination type
 * @priority: Priority selection within the DPIO or DPCON channel; valid values
 *	are 0-1 or 0-7, depending on the number of priorities in that
 *	channel; not relevant for 'DPSECI_DEST_NONE' option
 */
struct dpseci_dest_cfg {
	enum dpseci_dest	dest_type;
	int			dest_id;
	uint8_t			priority;
};

/**
 * DPSECI queue modification options
 */

/**
 * Select to modify the user's context associated with the queue
 */
#define DPSECI_QUEUE_OPT_USER_CTX		0x00000001

/**
 * Select to modify the queue's destination
 */
#define DPSECI_QUEUE_OPT_DEST			0x00000002

/**
 * Select to modify the queue's order preservation
 */
#define DPSECI_QUEUE_OPT_ORDER_PRESERVATION	0x00000004

/**
 * struct dpseci_rx_queue_cfg - DPSECI RX queue configuration
 * @options: Flags representing the suggested modifications to the queue;
 *	Use any combination of 'DPSECI_QUEUE_OPT_<X>' flags
 * @order_preservation_en: order preservation configuration for the rx queue
 * valid only if 'DPSECI_QUEUE_OPT_ORDER_PRESERVATION' is contained in 'options'
 * @user_ctx: User context value provided in the frame descriptor of each
 *	dequeued frame;
 *	valid only if 'DPSECI_QUEUE_OPT_USER_CTX' is contained in 'options'
 * @dest_cfg: Queue destination parameters;
 *	valid only if 'DPSECI_QUEUE_OPT_DEST' is contained in 'options'
 */
struct dpseci_rx_queue_cfg {
	uint32_t options;
	int order_preservation_en;
	uint64_t user_ctx;
	struct dpseci_dest_cfg dest_cfg;
};

/**
 * dpseci_set_rx_queue() - Set Rx queue configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @queue:	Select the queue relative to number of
 *		priorities configured at DPSECI creation; use
 *		DPSECI_ALL_QUEUES to configure all Rx queues identically.
 * @cfg:	Rx queue configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_set_rx_queue(struct fsl_mc_io			*mc_io,
			uint32_t				cmd_flags,
			uint16_t				token,
			uint8_t					queue,
			const struct dpseci_rx_queue_cfg	*cfg);

/**
 * struct dpseci_rx_queue_attr - Structure representing attributes of Rx queues
 * @user_ctx: User context value provided in the frame descriptor of each
 *	dequeued frame
 * @order_preservation_en: Status of the order preservation configuration
 *				on the queue
 * @dest_cfg: Queue destination configuration
 * @fqid: Virtual FQID value to be used for dequeue operations
 */
struct dpseci_rx_queue_attr {
	uint64_t		user_ctx;
	int			order_preservation_en;
	struct dpseci_dest_cfg	dest_cfg;
	uint32_t		fqid;
};

/**
 * dpseci_get_rx_queue() - Retrieve Rx queue attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @queue:	Select the queue relative to number of
 *				priorities configured at DPSECI creation
 * @attr:	Returned Rx queue attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_get_rx_queue(struct fsl_mc_io		*mc_io,
			uint32_t			cmd_flags,
			uint16_t			token,
			uint8_t				queue,
			struct dpseci_rx_queue_attr	*attr);

/**
 * struct dpseci_tx_queue_attr - Structure representing attributes of Tx queues
 * @fqid: Virtual FQID to be used for sending frames to SEC hardware
 * @priority: SEC hardware processing priority for the queue
 */
struct dpseci_tx_queue_attr {
	uint32_t fqid;
	uint8_t priority;
};

/**
 * dpseci_get_tx_queue() - Retrieve Tx queue attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @queue:	Select the queue relative to number of
 *				priorities configured at DPSECI creation
 * @attr:	Returned Tx queue attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_get_tx_queue(struct fsl_mc_io		*mc_io,
			uint32_t			cmd_flags,
			uint16_t			token,
			uint8_t				queue,
			struct dpseci_tx_queue_attr	*attr);

/**
 * struct dpseci_sec_attr - Structure representing attributes of the SEC
 *			hardware accelerator
 * @ip_id:	ID for SEC.
 * @major_rev: Major revision number for SEC.
 * @minor_rev: Minor revision number for SEC.
 * @era: SEC Era.
 * @deco_num: The number of copies of the DECO that are implemented in
 * this version of SEC.
 * @zuc_auth_acc_num: The number of copies of ZUCA that are implemented
 * in this version of SEC.
 * @zuc_enc_acc_num: The number of copies of ZUCE that are implemented
 * in this version of SEC.
 * @snow_f8_acc_num: The number of copies of the SNOW-f8 module that are
 * implemented in this version of SEC.
 * @snow_f9_acc_num: The number of copies of the SNOW-f9 module that are
 * implemented in this version of SEC.
 * @crc_acc_num: The number of copies of the CRC module that are implemented
 * in this version of SEC.
 * @pk_acc_num:  The number of copies of the Public Key module that are
 * implemented in this version of SEC.
 * @kasumi_acc_num: The number of copies of the Kasumi module that are
 * implemented in this version of SEC.
 * @rng_acc_num: The number of copies of the Random Number Generator that are
 * implemented in this version of SEC.
 * @md_acc_num: The number of copies of the MDHA (Hashing module) that are
 * implemented in this version of SEC.
 * @arc4_acc_num: The number of copies of the ARC4 module that are implemented
 * in this version of SEC.
 * @des_acc_num: The number of copies of the DES module that are implemented
 * in this version of SEC.
 * @aes_acc_num: The number of copies of the AES module that are implemented
 * in this version of SEC.
 **/

struct dpseci_sec_attr {
	uint16_t	ip_id;
	uint8_t	major_rev;
	uint8_t	minor_rev;
	uint8_t	era;
	uint8_t	deco_num;
	uint8_t	zuc_auth_acc_num;
	uint8_t	zuc_enc_acc_num;
	uint8_t	snow_f8_acc_num;
	uint8_t	snow_f9_acc_num;
	uint8_t	crc_acc_num;
	uint8_t	pk_acc_num;
	uint8_t	kasumi_acc_num;
	uint8_t	rng_acc_num;
	uint8_t	md_acc_num;
	uint8_t	arc4_acc_num;
	uint8_t	des_acc_num;
	uint8_t	aes_acc_num;
};

/**
 * dpseci_get_sec_attr() - Retrieve SEC accelerator attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @attr:	Returned SEC attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_get_sec_attr(struct fsl_mc_io		*mc_io,
			uint32_t			cmd_flags,
			uint16_t			token,
			struct dpseci_sec_attr		*attr);

/**
 * struct dpseci_sec_counters - Structure representing global SEC counters and
 *				not per dpseci counters
 * @dequeued_requests:	Number of Requests Dequeued
 * @ob_enc_requests:	Number of Outbound Encrypt Requests
 * @ib_dec_requests:	Number of Inbound Decrypt Requests
 * @ob_enc_bytes:		Number of Outbound Bytes Encrypted
 * @ob_prot_bytes:		Number of Outbound Bytes Protected
 * @ib_dec_bytes:		Number of Inbound Bytes Decrypted
 * @ib_valid_bytes:		Number of Inbound Bytes Validated
 */
struct dpseci_sec_counters {
	uint64_t	dequeued_requests;
	uint64_t	ob_enc_requests;
	uint64_t	ib_dec_requests;
	uint64_t	ob_enc_bytes;
	uint64_t	ob_prot_bytes;
	uint64_t	ib_dec_bytes;
	uint64_t	ib_valid_bytes;
};

/**
 * dpseci_get_sec_counters() - Retrieve SEC accelerator counters.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @counters:	Returned SEC counters
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_get_sec_counters(struct fsl_mc_io		*mc_io,
			    uint32_t			cmd_flags,
			    uint16_t			token,
			    struct dpseci_sec_counters	*counters);

/**
 * dpseci_get_api_version() - Get Data Path SEC Interface API version
 * @mc_io:  Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @major_ver:	Major version of data path sec API
 * @minor_ver:	Minor version of data path sec API
 *
 * Return:  '0' on Success; Error code otherwise.
 */
int dpseci_get_api_version(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			   uint16_t *major_ver,
			   uint16_t *minor_ver);

/**
 * dpseci_set_opr() - Set Order Restoration configuration.
 * @mc_io:		Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSECI object
 * @index:	The queue index
 * @options:	Configuration mode options
 *				can be OPR_OPT_CREATE or OPR_OPT_RETIRE
 * @cfg:		Configuration options for the OPR
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_set_opr(struct fsl_mc_io *mc_io,
	      uint32_t cmd_flags,
	      uint16_t token,
		  uint8_t index,
		  uint8_t options,
		  struct opr_cfg *cfg);

/**
 * dpseci_get_opr() - Retrieve Order Restoration config and query.
 * @mc_io:		Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSECI object
 * @index:	The queue index
 * @cfg:		Returned OPR configuration
 * @qry:		Returned OPR query
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpseci_get_opr(struct fsl_mc_io *mc_io,
		      uint32_t cmd_flags,
		     uint16_t token,
			 uint8_t index,
			 struct opr_cfg *cfg,
			 struct opr_qry *qry);

/**
 * enum dpseci_congestion_unit - DPSECI congestion units
 * @DPSECI_CONGESTION_UNIT_BYTES: bytes units
 * @DPSECI_CONGESTION_UNIT_FRAMES: frames units
 */
enum dpseci_congestion_unit {
	DPSECI_CONGESTION_UNIT_BYTES = 0,
	DPSECI_CONGESTION_UNIT_FRAMES
};

#define DPSECI_CGN_MODE_WRITE_MEM_ON_ENTER		0x00000001
#define DPSECI_CGN_MODE_WRITE_MEM_ON_EXIT		0x00000002
#define DPSECI_CGN_MODE_COHERENT_WRITE			0x00000004
#define DPSECI_CGN_MODE_NOTIFY_DEST_ON_ENTER		0x00000008
#define DPSECI_CGN_MODE_NOTIFY_DEST_ON_EXIT		0x00000010
#define DPSECI_CGN_MODE_INTR_COALESCING_DISABLED	0x00000020

/**
 * struct dpseci_congestion_notification_cfg - congestion notification
 *	configuration
 * @units: units type
 * @threshold_entry: above this threshold we enter a congestion state.
 *	set it to '0' to disable it
 * @threshold_exit: below this threshold we exit the congestion state.
 * @message_ctx: The context that will be part of the CSCN message
 * @message_iova: I/O virtual address (must be in DMA-able memory),
 *	must be 16B aligned;
 * @dest_cfg: CSCN can be send to either DPIO or DPCON WQ channel
 * @notification_mode: Mask of available options; use 'DPSECI_CGN_MODE_<X>'
 *  values
 */
struct dpseci_congestion_notification_cfg {
	enum dpseci_congestion_unit units;
	uint32_t threshold_entry;
	uint32_t threshold_exit;
	uint64_t message_ctx;
	uint64_t message_iova;
	struct dpseci_dest_cfg dest_cfg;
	uint16_t notification_mode;
};

/**
 * dpseci_set_congestion_notification() - Set congestion group
 *	notification configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @cfg:	congestion notification configuration
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpseci_set_congestion_notification(struct fsl_mc_io	*mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			const struct dpseci_congestion_notification_cfg *cfg);

/**
 * dpseci_get_congestion_notification() - Get congestion group
 *	notification configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSECI object
 * @cfg:	congestion notification configuration
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpseci_get_congestion_notification(struct fsl_mc_io	*mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			struct dpseci_congestion_notification_cfg *cfg);



#endif /* __FSL_DPSECI_H */
