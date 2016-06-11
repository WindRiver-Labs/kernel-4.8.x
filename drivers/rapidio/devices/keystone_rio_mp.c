/*
 * Copyright (C) 2014 Texas Instruments Incorporated
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/kfifo.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/soc/ti/knav_qmss.h>
#include <linux/soc/ti/knav_dma.h>
#include <linux/soc/ti/knav_helpers.h>

#include "keystone_rio_serdes.h"
#include "keystone_rio.h"

static void keystone_rio_tx_complete(void *data);
static void keystone_rio_tx_error(void *data);

/*
 * DMA descriptor helper functions
 */
static inline void get_pkt_info(u32 *buff, u32 *buff_len, u32 *ndesc,
				struct knav_dma_desc *desc)
{
	*buff_len = desc->buff_len;
	*buff = desc->buff;
	*ndesc = desc->next_desc;
}

static inline void get_pad_info(u32 *pad0, u32 *pad1,
				struct knav_dma_desc *desc)
{
	*pad0 = desc->pad[0];
	*pad1 = desc->pad[1];
}

static inline void get_org_pkt_info(u32 *buff, u32 *buff_len,
				    struct knav_dma_desc *desc)
{
	*buff = desc->orig_buff;
	*buff_len = desc->orig_len;
}

static inline void get_words(u32 *words, int num_words, u32 *desc)
{
	int i;

	for (i = 0; i < num_words; i++)
		words[i] = desc[i];
}

static inline void set_pkt_info(u32 buff, u32 buff_len, u32 ndesc,
				struct knav_dma_desc *desc)
{
	desc->buff_len = buff_len;
	desc->buff = buff;
	desc->next_desc = ndesc;
}

static inline void set_desc_info(u32 desc_info, u32 pkt_info,
				 struct knav_dma_desc *desc)
{
	desc->desc_info = desc_info;
	desc->packet_info = pkt_info;
}

static inline void set_pad_info(u32 pad0, u32 pad1,
				struct knav_dma_desc *desc)
{
	desc->pad[0] = pad0;
	desc->pad[1] = pad1;
}

static inline void set_org_pkt_info(u32 buff, u32 buff_len,
				    struct knav_dma_desc *desc)
{
	desc->orig_buff = buff;
	desc->orig_len = buff_len;
}

static inline void set_words(u32 *words, int num_words, u32 *desc)
{
	int i;

	for (i = 0; i < num_words; i++)
		desc[i] = words[i];
}

/*
 * Retrieve MP receive code completion
 */
static inline u32 keystone_rio_mp_get_cc(u32 psdata1, u32 packet_type)
{
	return (packet_type == RIO_PACKET_TYPE_MESSAGE ?
		psdata1 >> 15 : psdata1 >> 8) & 0x3;
}

/*
 * This function retrieves the packet type for a given mbox
 */
static inline u32 keystone_rio_mp_get_type(int mbox,
					   struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan =
					&krio_priv->rx_channels[mbox];

	return (u32)((krx_chan->packet_type != RIO_PACKET_TYPE_STREAM) &&
		     (krx_chan->packet_type != RIO_PACKET_TYPE_MESSAGE)) ?
		RIO_PACKET_TYPE_MESSAGE : krx_chan->packet_type;
}

/*
 * This function retrieves the mapping from Linux RIO mailbox to stream id for
 * type 9 packets
 */
static inline u32
keystone_rio_mbox_to_strmid(int mbox, struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan =
				&krio_priv->rx_channels[mbox];

	return (u32)krx_chan->stream_id;
}

void keystone_rio_chan_work_handler(unsigned long data)
{
	struct keystone_rio_rx_chan_info *krx_chan =
		(struct keystone_rio_rx_chan_info *)data;
	struct keystone_rio_data *krio_priv = krx_chan->priv;
	struct keystone_rio_mbox_info *p_mbox;
	int mbox = krx_chan->mbox_id;

	p_mbox = &krio_priv->rx_mbox[mbox];
	if (likely(p_mbox->running)) {
		/* Client callback (slot is not used) */
		p_mbox->port->inb_msg[mbox].mcback(
			p_mbox->port,
			p_mbox->dev_id,
			mbox,
			0);
	}

	knav_queue_enable_notify(krx_chan->queue);
}

static void keystone_rio_rx_notify(void *arg)
{
	struct keystone_rio_rx_chan_info *krx_chan = arg;

	knav_queue_disable_notify(krx_chan->queue);

	tasklet_schedule(&krx_chan->tasklet);
}

/* Release descriptors and attached buffers from Rx FDQ */
static void keystone_rio_free_inb_buf(
	struct keystone_rio_rx_chan_info *krx_chan,
	int fdq)
{
	struct keystone_rio_data *krio_priv = krx_chan->priv;
	struct knav_dma_desc *desc;
	unsigned int buf_len, dma_sz;
	dma_addr_t dma;
	void *buffer;
	u32 tmp;

	/* Release inbound descriptors */
	while ((dma = knav_queue_pop(krx_chan->fdq[fdq], &dma_sz))) {
		desc = knav_pool_desc_unmap(krx_chan->pool, dma, dma_sz);
		if (unlikely(!desc)) {
			dev_err(krio_priv->dev, "failed to unmap Rx desc\n");
			continue;
		}

		get_org_pkt_info(&dma, &buf_len, desc);
		get_pad_info((u32 *)&buffer, &tmp, desc);

		if (unlikely(!dma)) {
			dev_err(krio_priv->dev, "NULL orig_buff in desc\n");
			knav_pool_desc_put(krx_chan->pool, desc);
			continue;
		}

		if (unlikely(!buffer)) {
			dev_err(krio_priv->dev, "NULL bufptr in desc\n");
			knav_pool_desc_put(krx_chan->pool, desc);
			continue;
		}

		dma_unmap_single(krio_priv->dev, dma, buf_len,
				 DMA_FROM_DEVICE);

		/*
		 * We do not free the buffer since the upper layer (mp client)
		 * must be responsible for that task
		 */
		knav_pool_desc_put(krx_chan->pool, desc);
	}
}

static void keystone_rio_inb_pool_free(
	struct keystone_rio_rx_chan_info *krx_chan)
{
	struct keystone_rio_data *krio_priv = krx_chan->priv;
	int i;

	for (i = 0; i < KNAV_DMA_FDQ_PER_CHAN &&
	     !IS_ERR_OR_NULL(krx_chan->fdq[i]); i++) {
		dev_dbg(krio_priv->dev, "freeing fdq %d\n", i);
		keystone_rio_free_inb_buf(krx_chan, i);
	}

	if (knav_pool_count(krx_chan->pool) != krx_chan->pool_size)
		dev_err(krio_priv->dev,
			"lost Rx (%d) descriptors\n",
			krx_chan->pool_size - knav_pool_count(krx_chan->pool));

	knav_pool_destroy(krx_chan->pool);
	krx_chan->pool = NULL;
}

static void keystone_rio_inb_knav_free(
	struct keystone_rio_rx_chan_info *krx_chan)
{
	int i;

	/* Close channel */
	if (!IS_ERR_OR_NULL(krx_chan->channel)) {
		knav_dma_close_channel(krx_chan->channel);
		krx_chan->channel = NULL;
	}

	/* Free pools and associated descriptors */
	if (!IS_ERR_OR_NULL(krx_chan->pool))
		keystone_rio_inb_pool_free(krx_chan);

	/* Close completion queue */
	if (!IS_ERR_OR_NULL(krx_chan->queue)) {
		knav_queue_close(krx_chan->queue);
		krx_chan->queue = NULL;
	}

	/* Close free queues */
	for (i = 0; i < KNAV_DMA_FDQ_PER_CHAN &&
	     !IS_ERR_OR_NULL(krx_chan->fdq[i]); ++i) {
		knav_queue_close(krx_chan->fdq[i]);
		krx_chan->fdq[i] = NULL;
	}
}

static void keystone_rio_inb_exit(int mbox,
				  struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan =
					&krio_priv->rx_channels[mbox];

	if (!(krx_chan->channel))
		return;

	knav_queue_disable_notify(krx_chan->queue);

	/* Free all navigator Rx resources */
	keystone_rio_inb_knav_free(krx_chan);
}

static int keystone_rio_mp_inb_init(int mbox,
				    struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan =
					&krio_priv->rx_channels[mbox];
	struct knav_queue_notify_config notify_cfg;
	struct knav_dma_cfg config;
	u8 name[24];
	u32 last_fdq = 0;
	int err = -ENODEV;
	int i;

	/* If already initialized, return without error */
	if (krx_chan->channel)
		return 0;

	if (!krx_chan->name) {
		dev_err(krio_priv->dev,
			"Rx channel name for mbox %d is not defined!\n",
			mbox);
		return err;
	}

	/* Create Rx descriptor pool */
	snprintf(name, sizeof(name), "rx-pool-rio-mbox-%d", mbox);
	krx_chan->pool = knav_pool_create(name,
					  krx_chan->pool_size,
					  krx_chan->pool_region_id);
	if (IS_ERR_OR_NULL(krx_chan->pool)) {
		dev_err(krio_priv->dev,
			"couldn't create Rx pool\n");
		err = PTR_ERR(krx_chan->pool);
		goto fail;
	}

	/* Open Rx completion (receive) queue */
	snprintf(name, sizeof(name), "rx-compl-rio-mbox-%d", mbox);
	krx_chan->queue = knav_queue_open(name, krx_chan->queue_id, 0);
	if (IS_ERR_OR_NULL(krx_chan->queue)) {
		err = PTR_ERR(krx_chan->queue);
		goto fail;
	}

	krx_chan->queue_id = knav_queue_get_id(krx_chan->queue);

	/* Set notification for Rx completion */
	notify_cfg.fn = keystone_rio_rx_notify;
	notify_cfg.fn_arg = krx_chan;
	err = knav_queue_device_control(krx_chan->queue,
					KNAV_QUEUE_SET_NOTIFIER,
					(unsigned long)&notify_cfg);
	if (err)
		goto fail;

	knav_queue_disable_notify(krx_chan->queue);

	/* Open Rx FDQs */
	for (i = 0; i < KNAV_DMA_FDQ_PER_CHAN &&
	     krx_chan->queue_depths[i] &&
	     krx_chan->buffer_sizes[i]; ++i) {
		snprintf(name, sizeof(name), "rx-fdq-rio-mbox-%d-%d", mbox, i);
		krx_chan->fdq[i] = knav_queue_open(name, KNAV_QUEUE_GP, 0);
		if (IS_ERR_OR_NULL(krx_chan->fdq[i])) {
			err = PTR_ERR(krx_chan->fdq[i]);
			goto fail;
		}
	}

	memset(&config, 0, sizeof(config));
	config.direction		= DMA_DEV_TO_MEM;
	config.u.rx.einfo_present	= true;
	config.u.rx.psinfo_present	= true;
	config.u.rx.err_mode		= DMA_DROP;
	config.u.rx.desc_type		= DMA_DESC_HOST;
	config.u.rx.psinfo_at_sop	= false;
	config.u.rx.sop_offset		= 0;
	config.u.rx.dst_q		= krx_chan->queue_id;
	config.u.rx.thresh		= DMA_THRESH_NONE;

	for (i = 0; i < KNAV_DMA_FDQ_PER_CHAN; ++i) {
		if (krx_chan->fdq[i])
			last_fdq = knav_queue_get_id(krx_chan->fdq[i]);
		config.u.rx.fdq[i] = last_fdq;
	}

	krx_chan->channel = knav_dma_open_channel(krio_priv->dev,
						  krx_chan->name,
						  &config);
	if (IS_ERR_OR_NULL(krx_chan->channel)) {
		dev_err(krio_priv->dev,
			"failed opening Rx chan(%s\n",
			krx_chan->name);
		err = PTR_ERR(krx_chan->channel);
		goto fail;
	}

	krx_chan->priv    = krio_priv;
	krx_chan->flow_id = knav_dma_get_flow(krx_chan->channel);
	krx_chan->mbox_id = mbox;

	/* Initialize the associated Rx tasklet */
	tasklet_init(&krx_chan->tasklet,
		     keystone_rio_chan_work_handler,
		     (unsigned long)krx_chan);

	knav_queue_enable_notify(krx_chan->queue);

	dev_dbg(krio_priv->dev,
		"opened Rx channel: mbox = %d, complete_q = %d (0x%x), channel/flow = %d (0x%x), pkt_type = %d)\n",
		mbox, krx_chan->queue_id, (u32)krx_chan->queue,
		krx_chan->flow_id, (u32)krx_chan->channel,
		krx_chan->packet_type);

	return 0;

fail:
	keystone_rio_inb_knav_free(krx_chan);

	return err;
}

static int keystone_rio_get_rxu_map(struct keystone_rio_data *krio_priv)
{
	int id;
	unsigned long bit_sz = sizeof(krio_priv->rxu_map_bitmap) * 8;

	id = find_first_zero_bit(&krio_priv->rxu_map_bitmap[0], bit_sz);

	while (id < krio_priv->rxu_map_start)
		id = find_next_zero_bit(&krio_priv->rxu_map_bitmap[0],
					bit_sz, ++id);

	if (id > krio_priv->rxu_map_end)
		return -1;

	__set_bit(id, &krio_priv->rxu_map_bitmap[0]);

	return id;
}

static void keystone_rio_free_rxu_map(int id,
				      struct keystone_rio_data *krio_priv)
{
	clear_bit(id, &krio_priv->rxu_map_bitmap[0]);
}

/**
 * keystone_rio_map_mbox - Map a mailbox to a given queue for both type 11
 * and type 9 packets.
 * @mbox: mailbox to map
 * @queue: associated queue number
 * @flow_id: flow Id
 * @size: device Id size
 *
 * Returns %0 on success or %-ENOMEM on failure.
 */
static int keystone_rio_map_mbox(int mbox,
				 int queue,
				 int flow_id,
				 int size,
				 struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];
	u32 mapping_entry_low = 0;
	u32 mapping_entry_high = 0;
	u32 mapping_entry_qid;
	u32 mapping_t9_reg[3];
	u32 pkt_type;
	int i;

	/* Retrieve the packet type */
	pkt_type = keystone_rio_mp_get_type(mbox, krio_priv);

	if (pkt_type == RIO_PACKET_TYPE_MESSAGE) {
		/*
		 * Map the multi-segment mailbox to the corresponding Rx queue
		 * for type 11.
		 * Given mailbox, all letters, srcid = 0
		 */
		mapping_entry_low = ((mbox & 0x1f) << 16) | (0x3f000000);

		/*
		 * Multi-segment messaging and promiscuous (don't care about
		 * src/dst id)
		 */
		mapping_entry_high = KEYSTONE_RIO_MAP_FLAG_SEGMENT
			| KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC
			| KEYSTONE_RIO_MAP_FLAG_DST_PROMISC;
	} else {
		/*
		 * Map the multi-segment mailbox for type 9
		 * accept all COS and srcid = 0, use promiscuous (don't care
		 * about src/dst id)
		 */
		mapping_t9_reg[0] = 0;
		mapping_t9_reg[1] = KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC
			| KEYSTONE_RIO_MAP_FLAG_DST_PROMISC;
		mapping_t9_reg[2] = (0xffff << 16)
			| (keystone_rio_mbox_to_strmid(mbox, krio_priv));
	}

	/* Set TT flag */
	if (size) {
		mapping_entry_high |= KEYSTONE_RIO_MAP_FLAG_TT_16;
		mapping_t9_reg[1]  |= KEYSTONE_RIO_MAP_FLAG_TT_16;
	}

	/* QMSS/PktDMA mapping (generic for both type 9 and 11) */
	mapping_entry_qid = (queue & 0x3fff) | (flow_id << 16);

	i = keystone_rio_get_rxu_map(krio_priv);
	if (i < 0)
		return -ENOMEM;

	rx_mbox->rxu_map_id[0] = i;

	dev_dbg(krio_priv->dev,
		"using RXU map %d @ 0x%p: mbox=%d, flow_id=%d, queue=%d pkt_type=%d\n",
		i, &krio_priv->regs->rxu_map[i], mbox,
		flow_id, queue, pkt_type);

	if (pkt_type == RIO_PACKET_TYPE_MESSAGE) {
		/* Set packet type 11 rx mapping */
		__raw_writel(mapping_entry_low,
			     &krio_priv->regs->rxu_map[i].ltr_mbox_src);
		__raw_writel(mapping_entry_high,
			     &krio_priv->regs->rxu_map[i].dest_prom_seg);
	} else {
		/* Set packet type 9 rx mapping */
		__raw_writel(mapping_t9_reg[0],
			     &krio_priv->regs->rxu_type9_map[i].cos_src);
		__raw_writel(mapping_t9_reg[1],
			     &krio_priv->regs->rxu_type9_map[i].dest_prom);
		__raw_writel(mapping_t9_reg[2],
			     &krio_priv->regs->rxu_type9_map[i].stream);
	}

	__raw_writel(mapping_entry_qid,
		     &krio_priv->regs->rxu_map[i].flow_qid);

	if (pkt_type == RIO_PACKET_TYPE_MESSAGE) {
		/*
		 * The RapidIO peripheral looks at the incoming RapidIO msgs
		 * and if there is only one segment (the whole msg fits into
		 * one RapidIO msg), the peripheral uses the single segment
		 * mapping table. Therefore we need to map the single-segment
		 * mailbox too.
		 * The same Rx CPPI Queue is used (as for the multi-segment
		 * mailbox).
		 */
		mapping_entry_high &= ~KEYSTONE_RIO_MAP_FLAG_SEGMENT;

		i = keystone_rio_get_rxu_map(krio_priv);
		if (i < 0)
			return -ENOMEM;

		rx_mbox->rxu_map_id[1] = i;

		dev_dbg(krio_priv->dev,
			"using RXU map %d @ 0x%p: mbox=%d, flow_id=%d, queue=%d pkt_type=%d\n",
			i, &krio_priv->regs->rxu_map[i], mbox,
			flow_id, queue, pkt_type);

		__raw_writel(mapping_entry_low,
			     &krio_priv->regs->rxu_map[i].ltr_mbox_src);
		__raw_writel(mapping_entry_high,
			     &krio_priv->regs->rxu_map[i].dest_prom_seg);
		__raw_writel(mapping_entry_qid,
			     &krio_priv->regs->rxu_map[i].flow_qid);
	}

	return 0;
}

/**
 * keystone_rio_open_inb_mbox - Initialize KeyStone inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring, request the inbound message interrupt,
 * and enables the inbound message unit. Returns %0 on success
 * and %-EINVAL, %-EBUSY or %-ENOMEM on failure.
 */
int keystone_rio_open_inb_mbox(struct rio_mport *mport,
			       void *dev_id,
			       int mbox,
			       int entries)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];
	struct keystone_rio_rx_chan_info *krx_chan =
		&krio_priv->rx_channels[mbox];
	int res;

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return -EINVAL;

	/*
	 * Check that number of entries is a power of two to ease ring
	 * management
	 */
	if ((entries & (entries - 1)) != 0)
		return -EINVAL;

	/* Check if already initialized */
	if (rx_mbox->port)
		return -EBUSY;

	dev_dbg(krio_priv->dev,
		"open_inb_mbox: mport=0x%p, dev_id=0x%p, mbox=%d, entries=%d\n",
		mport, dev_id, mbox, entries);

	/* Initialization of RapidIO inbound MP */
	res = keystone_rio_mp_inb_init(mbox, krio_priv);
	if (res)
		return res;

	rx_mbox->dev_id    = dev_id;
	rx_mbox->entries   = entries;
	rx_mbox->port      = mport;
	rx_mbox->running   = 1;

	/* Map the mailbox to queue/flow */
	res = keystone_rio_map_mbox(mbox,
				    krx_chan->queue_id,
				    krx_chan->flow_id,
				    mport->sys_size,
				    krio_priv);

	return res;
}

/**
 * keystone_rio_close_inb_mbox - Shutdown KeyStone inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
void keystone_rio_close_inb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *rx_mbox = &krio_priv->rx_mbox[mbox];

	dev_dbg(krio_priv->dev, "close inb mbox: mport = 0x%p, mbox = %d\n",
		mport, mbox);

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return;

	rx_mbox->running = 0;

	if (!rx_mbox->port)
		return;

	rx_mbox->port = NULL;

	keystone_rio_inb_exit(mbox, krio_priv);

	/* Release associated resources */
	keystone_rio_free_rxu_map(rx_mbox->rxu_map_id[0], krio_priv);
	keystone_rio_free_rxu_map(rx_mbox->rxu_map_id[1], krio_priv);
}

/**
 * keystone_rio_hw_add_inb_buffer - Add buffer to the KeyStone inbound message
 * queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Adds the @buf buffer to the KeyStone inbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
int keystone_rio_hw_add_inb_buffer(struct rio_mport *mport,
				   int mbox,
				   void *buffer)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_rx_chan_info *krx_chan =
		&krio_priv->rx_channels[mbox];
	struct knav_dma_desc *desc;
	u32 desc_info, pkt_info;
	unsigned int buf_len, dma_sz;
	dma_addr_t dma, dma_addr;
	u32 pad[2];
	int fdq = 0;
	int ret;

	/* Allocate descriptor */
	desc = knav_pool_desc_get(krx_chan->pool);
	if (IS_ERR_OR_NULL(desc)) {
		dev_dbg(krio_priv->dev, "out of rx pool desc\n");
		return -ENOMEM;
	}

	/* Fill the proper free queue */
	while (krx_chan->queue_depths[fdq] &&
	       krx_chan->buffer_sizes[fdq] &&
	       (knav_queue_get_count(krx_chan->fdq[fdq])
		>= krx_chan->queue_depths[fdq])) {
		int nfdq = fdq + 1;

		if (unlikely(IS_ERR_OR_NULL(krx_chan->fdq[nfdq]) ||
			     (nfdq >= KNAV_DMA_FDQ_PER_CHAN))) {
			dev_err(krio_priv->dev,
				"free queue %d is full and cannot use free queue %d!!!\n",
				fdq, nfdq);
			return -ENOMEM;
		}

		fdq = nfdq;
	}

	buf_len = krx_chan->buffer_sizes[fdq];
	dma_addr = dma_map_single(krio_priv->dev,
				  buffer,
				  buf_len,
				  DMA_TO_DEVICE);

	if (IS_ERR_OR_NULL((void *)dma_addr)) {
		dev_err(krio_priv->dev, "dma map failed\n");
		knav_pool_desc_put(krx_chan->pool, desc);
		return -EINVAL;
	}

	pad[0] = (u32)buffer;
	pad[1] = 0; /* Slot is not used in receive */

	desc_info  =  KNAV_DMA_DESC_PS_INFO_IN_DESC;
	desc_info |= buf_len & KNAV_DMA_DESC_PKT_LEN_MASK;
	pkt_info   =  KNAV_DMA_DESC_HAS_EPIB;
	pkt_info  |= KNAV_DMA_NUM_PS_WORDS << KNAV_DMA_DESC_PSLEN_SHIFT;
	pkt_info  |= (krx_chan->queue_id & KNAV_DMA_DESC_RETQ_MASK) <<
		KNAV_DMA_DESC_RETQ_SHIFT;
	set_pkt_info(dma_addr, buf_len, 0, desc);
	set_org_pkt_info(dma_addr, buf_len, desc);
	set_pad_info(pad[0], pad[1], desc);
	set_desc_info(desc_info, pkt_info, desc);

	/* Push to FDQs */
	ret = knav_pool_desc_map(krx_chan->pool, desc, sizeof(*desc), &dma,
				 &dma_sz);
	if (unlikely(ret)) {
		dev_err(krio_priv->dev, "%s() failed to map desc\n", __func__);
		dma_unmap_single(krio_priv->dev, dma_addr, buf_len,
				 DMA_TO_DEVICE);
		knav_pool_desc_put(krx_chan->pool, desc);
		return -ENOMEM;
	}

	knav_queue_push(krx_chan->fdq[fdq], dma, sizeof(*desc), 0);

	return 0;
}

/**
 * keystone_rio_hw_get_inb_message - Fetch inbound message from
 * the KeyStone message unit
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 *
 * Gets the next available inbound message from the inbound message queue.
 * A pointer to the message is returned on success or NULL on failure.
 */
void *keystone_rio_hw_get_inb_message(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_rx_chan_info *krx_chan =
					&krio_priv->rx_channels[mbox];
	unsigned int dma_sz, buf_len;
	struct knav_dma_desc *desc = NULL;
	dma_addr_t dma_desc, dma_buff;
	void *buffer = NULL;
	u32 cc, pad1;

	/* Get incoming descriptor */
	dma_desc = knav_queue_pop(krx_chan->queue, &dma_sz);
	if (!dma_desc)
		goto end;

	desc = knav_pool_desc_unmap(krx_chan->pool, dma_desc, dma_sz);
	if (unlikely(!desc)) {
		dev_err(krio_priv->dev, "failed to unmap Rx desc\n");
		goto end;
	}

	get_pkt_info(&dma_buff, &buf_len, &dma_desc, desc);
	get_pad_info((u32 *)&buffer, &pad1, desc);

	if (unlikely(!buffer)) {
		dev_err(krio_priv->dev, "NULL bufptr in desc\n");
		goto end;
	}

	dma_unmap_single(krio_priv->dev, dma_buff, buf_len, DMA_FROM_DEVICE);

	/* Check CC from PS descriptor word 1 */
	cc = keystone_rio_mp_get_cc(desc->psdata[1], krx_chan->packet_type);
	if (cc) {
		dev_warn(krio_priv->dev,
			 "MP receive completion code is non zero (0x%x)\n",
			 cc);
	}

end:
	/* Free the descriptor */
	if (desc)
		knav_pool_desc_put(krx_chan->pool, desc);

	return buffer;
}

static void keystone_rio_outb_knav_free(
	struct keystone_rio_tx_chan_info *ktx_chan)
{
	/* Close channel */
	if (!IS_ERR_OR_NULL(ktx_chan->channel)) {
		knav_dma_close_channel(ktx_chan->channel);
		ktx_chan->channel = NULL;
	}

	/* Close transmit queue */
	if (!IS_ERR_OR_NULL(ktx_chan->queue)) {
		knav_queue_close(ktx_chan->queue);
		ktx_chan->queue = NULL;
	}

	/* Close completion queue */
	if (!IS_ERR_OR_NULL(ktx_chan->complet_queue)) {
		knav_queue_close(ktx_chan->complet_queue);
		ktx_chan->complet_queue = NULL;
	}

	/* Close garbage queue */
	if (!IS_ERR_OR_NULL(ktx_chan->garbage_queue)) {
		knav_queue_close(ktx_chan->garbage_queue);
		ktx_chan->garbage_queue = NULL;
	}

	/* Free pools and associated descriptors */
	if (!IS_ERR_OR_NULL(ktx_chan->pool)) {
		knav_pool_destroy(ktx_chan->pool);
		ktx_chan->pool = NULL;
	}
}

static void keystone_rio_mp_outb_exit(struct keystone_rio_data *krio_priv,
				      int mbox)
{
	struct keystone_rio_tx_chan_info *ktx_chan =
					&krio_priv->tx_channels[mbox];

	if (!(ktx_chan->channel))
		return;

	knav_queue_disable_notify(ktx_chan->complet_queue);
	knav_queue_disable_notify(ktx_chan->garbage_queue);

	/* Free all navigator Tx resources */
	keystone_rio_outb_knav_free(ktx_chan);
}

static int keystone_rio_mp_outb_init(u8 port_id,
				     int mbox,
				     struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_tx_chan_info *ktx_chan =
					&krio_priv->tx_channels[mbox];
	struct knav_queue_notify_config notify_cfg;
	struct knav_dma_cfg config;
	u8 name[24];
	int err = -ENODEV;

	/* If already initialized, return without error */
	if (ktx_chan->channel)
		return 0;

	if (!ktx_chan->name) {
		dev_err(krio_priv->dev,
			"Tx channel name for mbox %d is not defined!\n",
			mbox);
		return err;
	}

	memset(&config, 0, sizeof(config));
	config.direction         = DMA_MEM_TO_DEV;
	config.u.tx.filt_einfo   = false;
	config.u.tx.filt_pswords = false;
	config.u.tx.priority     = DMA_PRIO_MED_L;

	ktx_chan->channel = knav_dma_open_channel(krio_priv->dev,
						  ktx_chan->name,
						  &config);
	if (IS_ERR_OR_NULL(ktx_chan->channel)) {
		dev_err(krio_priv->dev,
			"failed opening Tx chan(%s\n",
			ktx_chan->name);
		err = PTR_ERR(ktx_chan->channel);
		goto fail;
	}

	/* Create Tx descriptor pool */
	snprintf(name, sizeof(name), "tx-pool-rio-mbox-%d", mbox);
	ktx_chan->pool = knav_pool_create(name,
					  ktx_chan->pool_size,
					  ktx_chan->pool_region_id);
	if (IS_ERR_OR_NULL(ktx_chan->pool)) {
		dev_err(krio_priv->dev,
			"couldn't create Tx pool\n");
		err = PTR_ERR(ktx_chan->pool);
		goto fail;
	}

	ktx_chan->priv = krio_priv;
	ktx_chan->mbox_id = mbox;

	/* Open Tx completion queue */
	snprintf(name, sizeof(name), "tx-compl-rio-mbox-%d", mbox);
	ktx_chan->complet_queue = knav_queue_open(name,
						  ktx_chan->complet_queue_id,
						  0);
	if (IS_ERR_OR_NULL(ktx_chan->complet_queue)) {
		err = PTR_ERR(ktx_chan->complet_queue);
		goto fail;
	}

	ktx_chan->complet_queue_id = knav_queue_get_id(ktx_chan->complet_queue);

	/* Set notification for Tx completion */
	notify_cfg.fn = keystone_rio_tx_complete;
	notify_cfg.fn_arg = ktx_chan;
	err = knav_queue_device_control(ktx_chan->complet_queue,
					KNAV_QUEUE_SET_NOTIFIER,
					(unsigned long)&notify_cfg);
	if (err)
		goto fail;

	knav_queue_disable_notify(ktx_chan->complet_queue);

	/* Open Tx submit queue */
	snprintf(name, sizeof(name), "tx-submit-rio-mbox-%d", mbox);
	ktx_chan->queue = knav_queue_open(name,
					  ktx_chan->queue_id,
					  0);
	if (IS_ERR_OR_NULL(ktx_chan->queue)) {
		err = PTR_ERR(ktx_chan->queue);
		goto fail;
	}

	ktx_chan->queue_id = knav_queue_get_id(ktx_chan->queue);

	/* Set the output port Id to the corresponding Tx queue */
	__raw_writel(port_id << 4, &krio_priv->regs->tx_queue_sch_info[mbox]);

	knav_queue_enable_notify(ktx_chan->complet_queue);

	/* Open Tx garbage queue */
	snprintf(name, sizeof(name), "tx-error-rio-mbox-%d", mbox);
	ktx_chan->garbage_queue = knav_queue_open(name,
						  ktx_chan->garbage_queue_id,
						  0);
	if (IS_ERR_OR_NULL(ktx_chan->garbage_queue)) {
		err = PTR_ERR(ktx_chan->garbage_queue);
		goto fail;
	}

	ktx_chan->garbage_queue_id = knav_queue_get_id(ktx_chan->garbage_queue);

	/* Set notification for Tx garbage queue */
	notify_cfg.fn = keystone_rio_tx_error;
	notify_cfg.fn_arg = ktx_chan;
	err = knav_queue_device_control(ktx_chan->garbage_queue,
					KNAV_QUEUE_SET_NOTIFIER,
					(unsigned long)&notify_cfg);
	if (err)
		goto fail;

	/* Set the garbage queues */
	__raw_writel((ktx_chan->garbage_queue_id << 16) |
		     ktx_chan->garbage_queue_id,
		     &krio_priv->regs->garbage_coll_qid[0]);
	__raw_writel((ktx_chan->garbage_queue_id << 16) |
		     ktx_chan->garbage_queue_id,
		     &krio_priv->regs->garbage_coll_qid[1]);
	__raw_writel(ktx_chan->garbage_queue_id,
		     &krio_priv->regs->garbage_coll_qid[2]);

	knav_queue_enable_notify(ktx_chan->garbage_queue);

	dev_dbg(krio_priv->dev,
		"opened Tx channel: mbox = %d, complete_q = %d (0x%x), tx_q = %d (0x%x), garbage_q = %d (0x%x), channel = 0x%x, port id = %d\n",
		mbox, ktx_chan->complet_queue_id, (u32)ktx_chan->complet_queue,
		ktx_chan->queue_id, (u32)ktx_chan->queue,
		ktx_chan->garbage_queue_id, (u32)ktx_chan->garbage_queue,
		(u32)ktx_chan->channel, port_id);

	return 0;

fail:
	keystone_rio_outb_knav_free(ktx_chan->channel);

	return err;
}

/**
 * keystone_rio_open_outb_mbox - Initialize KeyStone outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the outbound mailbox ring
 *
 * Initializes buffer ring, request the outbound message interrupt,
 * and enables the outbound message unit. Returns %0 on success and
 * %-EINVAL, %-EBUSY or %-ENOMEM on failure.
 */
int keystone_rio_open_outb_mbox(struct rio_mport *mport,
				void *dev_id,
				int mbox,
				int entries)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *tx_mbox = &krio_priv->tx_mbox[mbox];
	int res;

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return -EINVAL;

	/*
	 * Check that number of entries is a power of two to ease ring
	 * management
	 */
	if ((entries & (entries - 1)) != 0)
		return -EINVAL;

	/* Check if already initialized */
	if (tx_mbox->port)
		return -EBUSY;

	dev_dbg(krio_priv->dev,
		"open_outb_mbox: mport = 0x%x, dev_id = 0x%x, mbox = %d, entries = %d\n",
		(u32)mport, (u32)dev_id, mbox, entries);

	/* Initialization of RapidIO outbound MP */
	res = keystone_rio_mp_outb_init(mport->index, mbox, krio_priv);
	if (res)
		return res;

	tx_mbox->dev_id  = dev_id;
	tx_mbox->entries = entries;
	tx_mbox->port    = mport;
	tx_mbox->running = 1;
	atomic_set(&tx_mbox->slot, 0);

	return 0;
}

/**
 * keystone_rio_close_outb_mbox - Shutdown KeyStone outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
void keystone_rio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_mbox_info *tx_mbox = &krio_priv->tx_mbox[mbox];

	dev_dbg(krio_priv->dev, "close outb mbox: mport = 0x%x, mbox = %d\n",
		(u32)mport, mbox);

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return;

	tx_mbox->running = 0;

	if (!tx_mbox->port)
		return;

	tx_mbox->port = NULL;

	keystone_rio_mp_outb_exit(krio_priv, mbox);
}

/* Gerneric handling of both Tx completion and error */
static void keystone_rio_tx_handler(
	struct keystone_rio_tx_chan_info *ktx_chan,
	void *queue,
	int error)
{
	struct keystone_rio_data *krio_priv = ktx_chan->priv;
	int mbox_id  = ktx_chan->mbox_id;
	struct keystone_rio_mbox_info *mbox = &krio_priv->tx_mbox[mbox_id];
	struct rio_mport *port = mbox->port;
	void *dev_id  = mbox->dev_id;
	void *buffer = NULL;
	unsigned int dma_sz;
	dma_addr_t dma;
	int slot;

	knav_queue_disable_notify(queue);

	while ((dma = knav_queue_pop(queue, &dma_sz))) {
		struct knav_dma_desc *desc;

		/* Unmap descriptor */
		desc = knav_pool_desc_unmap(ktx_chan->pool, dma, dma_sz);
		if (unlikely(!desc)) {
			dev_err(krio_priv->dev, "failed to unmap Tx desc\n");
			continue;
		}

		/* Retrieve virtual temporary buffer if any and slot id */
		get_pad_info((u32 *)&buffer, &slot, desc);

		/* Release descriptor */
		knav_pool_desc_put(ktx_chan->pool, desc);

		/* kfree is working even if pointer is null */
		kfree(buffer);

		/*
		 * If it is an error, we do not notify the upper layer which
		 * should be able to manage non received acknowledgment with
		 * its slot.
		 */
		if (likely((mbox->running) && (!error))) {
			port->outb_msg[mbox_id].mcback(port,
						       dev_id,
						       mbox_id,
						       slot % mbox->entries);
		}

		if (unlikely(error))
			dev_dbg(krio_priv->dev, "error with Tx slot %d\n",
				slot);
	}

	knav_queue_enable_notify(queue);
}

static void keystone_rio_tx_error(void *data)
{
	struct keystone_rio_tx_chan_info *ktx_chan = data;

	keystone_rio_tx_handler(ktx_chan, ktx_chan->garbage_queue, 1);
}

static void keystone_rio_tx_complete(void *data)
{
	struct keystone_rio_tx_chan_info *ktx_chan = data;

	keystone_rio_tx_handler(ktx_chan, ktx_chan->complet_queue, 0);
}

/**
 * keystone_rio_hw_add_outb_message - Add a message to the KeyStone
 * outbound message queue
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox: Outbound mailbox
 * @buffer: Message to add to outbound queue
 * @len: Length of message
 *
 * Adds the @buffer message to the KeyStone outbound message queue. Returns
 * %0 on success or %-EBUSY on failure.
 */
int keystone_rio_hw_add_outb_message(struct rio_mport *mport,
				     struct rio_dev *rdev,
				     int mbox,
				     void *buffer,
				     const size_t len)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_tx_chan_info *ktx_chan =
					&krio_priv->tx_channels[mbox];
	struct keystone_rio_mbox_info *ktx_mbox =
					&krio_priv->tx_mbox[mbox];
	struct knav_dma_desc *desc;
	dma_addr_t dma_addr, dma;
	unsigned int dma_sz;
	u32 plen;
	u32 packet_type;
	int ret = 0;
	void *send_buffer = NULL, *p_buffer;
	u32 desc_info, pkt_info;
	u32 pad[2];

	if (unlikely((ktx_mbox->port != mport) || (!rdev)))
		return -EINVAL;

	/*
	 * Ensure that the number of bytes being transmitted is a multiple
	 * of double-word. This is as per the specification.
	 */
	plen = ((len + 7) & ~0x7);

#if defined(CONFIG_RAPIDIO_CHMAN) || defined(CONFIG_RAPIDIO_CHMAN_MODULE)
	/*
	 * Copy the outbound message in a temporary buffer. This is needed for
	 * RIO_CM.
	 */
	send_buffer = kmalloc(plen, GFP_ATOMIC | GFP_DMA);
	if (unlikely(!send_buffer)) {
		dev_err(krio_priv->dev, "failed to alloc send buffer\n");
		return -ENOMEM;
	}

	memcpy(send_buffer, buffer, plen);
	p_buffer = send_buffer;
#else
	p_buffer = buffer;
#endif

	/* Map the linear buffer */
	dma_addr = dma_map_single(krio_priv->dev, p_buffer, plen,
				  DMA_TO_DEVICE);
	if (unlikely(!dma_addr)) {
		dev_err(krio_priv->dev, "failed to map skb buffer\n");
		ret = -ENOMEM;
		goto fail;
	}

	desc = knav_pool_desc_get(ktx_chan->pool);
	if (unlikely(IS_ERR_OR_NULL(desc))) {
		dev_err(krio_priv->dev, "out of TX desc\n");
		dma_unmap_single(krio_priv->dev, dma_addr, plen, DMA_TO_DEVICE);
		ret = -ENOMEM;
		goto fail;
	}

	/* Save virtual buffer address for copy-case and slot */
	pad[0] = (u32)send_buffer; /* NULL if not using copy */
	pad[1] = atomic_read(&ktx_mbox->slot);

	/*
	 * Move slot index to the next message to be sent.
	 * Client is in charge of freeing the associated buffers
	 * because we do not have explicit hardware ring but queues, we
	 * do not know where we are in the sw ring. Let's try to keep
	 * slot in sync with client.
	 */
	atomic_inc(&ktx_mbox->slot);

	/* Word 1: source id and dest id (common to packet 11 and packet 9) */
	desc->psdata[0] = (rdev->destid & 0xffff)
		| (mport->host_deviceid << 16);

	/*
	 * Warning - Undocumented HW requirement:
	 *      For type 9, packet type MUST be set to 30 in
	 *	keystone_hw_desc.desc_info[29:25] bits.
	 *
	 *	For type 11, setting packet type to 31 in
	 *	those bits is optional.
	 */
	if (keystone_rio_mp_get_type(mbox, krio_priv)
	    == RIO_PACKET_TYPE_MESSAGE) {
		/* Packet 11 case (Message) */
		packet_type = 31;

		/* Word 2: ssize = 32 dword, 4 retries, letter = 0, mbox */
		desc->psdata[1] = (KEYSTONE_RIO_MSG_SSIZE << 17) | (4 << 21)
			| (mbox & 0x3f);
	} else {
		/* Packet 9 case (Data Streaming) */
		packet_type = 30;

		/* Word 2: COS = 0, stream id */
		desc->psdata[1] =
			keystone_rio_mbox_to_strmid(mbox, krio_priv) << 16;
	}

	if ((rdev->net) && (rdev->net->hport->sys_size))
		desc->psdata[1] |= KEYSTONE_RIO_DESC_FLAG_TT_16; /* tt */

	desc_info  = KNAV_DMA_DESC_PS_INFO_IN_DESC;
	desc_info |= plen & KNAV_DMA_DESC_PKT_LEN_MASK;
	desc_info |= packet_type << 25;
	pkt_info   = KNAV_DMA_DESC_HAS_EPIB;
	pkt_info  |= KNAV_DMA_NUM_PS_WORDS << KNAV_DMA_DESC_PSLEN_SHIFT;
	pkt_info  |= (ktx_chan->complet_queue_id & KNAV_DMA_DESC_RETQ_MASK) <<
		KNAV_DMA_DESC_RETQ_SHIFT;
	set_pkt_info(dma_addr, len, 0, desc);
	set_org_pkt_info(dma_addr, len, desc);
	set_pad_info(pad[0], pad[1], desc);
	set_desc_info(desc_info, pkt_info, desc);

	/* Map and push the descriptor */
	ret = knav_pool_desc_map(ktx_chan->pool, desc, sizeof(*desc), &dma,
				 &dma_sz);
	if (unlikely(ret)) {
		dev_err(krio_priv->dev, "%s() failed to map desc\n", __func__);
		dma_unmap_single(krio_priv->dev, dma_addr, plen, DMA_TO_DEVICE);
		knav_pool_desc_put(ktx_chan->pool, desc);
		ret = -ENOMEM;
		goto fail;
	}

	knav_queue_push(ktx_chan->queue, dma, dma_sz, 0);

	return ret;

fail:
	kfree(send_buffer);

	return ret;
}
