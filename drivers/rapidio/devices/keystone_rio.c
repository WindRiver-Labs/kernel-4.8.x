/*
 * Copyright (C) 2010, 2011, 2012, 2013, 2014 Texas Instruments Incorporated
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
 * - Main driver implementation.
 * - Updated for support on TI KeyStone 2 platform.
 *
 * Copyright (C) 2012, 2013 Texas Instruments Incorporated
 * WingMan Kwok <w-kwok2@ti.com>
 * - Updated for support on TI KeyStone 1 platform.
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
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/kfifo.h>
#include <linux/platform_device.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>

#include "keystone_rio_serdes.h"
#include "keystone_rio.h"

#define DRIVER_VER "v1.4"

static bool serdes_calibration;
module_param(serdes_calibration, bool, 0);
MODULE_PARM_DESC(
	serdes_calibration,
	"Perform Serdes calibration before starting RapidIO (default = 0)");

static bool enable_ports = 1;
module_param(enable_ports, bool, 0);
MODULE_PARM_DESC(
	enable_ports,
	"Enable RapidIO ports at boottime (default = 1)");

/*
 * When using some RIO switches like CPS gen2, error recovery must be
 * disabled. Local controller reset and reset symbol will be used instead.
 */
static bool error_recovery = 1;
module_param(error_recovery, bool, 0);
MODULE_PARM_DESC(
	error_recovery,
	"Use RapidIO error recovery mechanism (default = 1)");

static void dbell_handler(
	struct keystone_rio_data *krio_priv);
static void keystone_rio_port_write_handler(
	struct keystone_rio_data *krio_priv);
static void keystone_rio_handle_logical_error(
	struct keystone_rio_data *krio_priv);
static int  keystone_rio_setup_controller(
	struct keystone_rio_data *krio_priv);
static void keystone_rio_shutdown_controller(
	struct keystone_rio_data *krio_priv);

/*
 * Table with the various lanes per port configuration modes:
 * path mode 0: 4 ports in 1x
 * path mode 1: 3 ports in 2x/1x
 * path mode 2: 3 ports in 1x/2x
 * path mode 3: 2 ports in 2x
 * path mode 4: 1 ports in 4x
 */
static struct keystone_lane_config keystone_lane_configs[5][4] = {
	{ {0, 1}, {1, 2},   {2, 3},   {3, 4}   },
	{ {0, 2}, {-1, -1}, {2, 3},   {3, 4}   },
	{ {0, 1}, {1, 2},   {2, 4},   {-1, -1} },
	{ {0, 2}, {-1, -1}, {2, 4},   {-1, -1} },
	{ {0, 4}, {-1, -1}, {-1, -1}, {-1, -1} },
};

#define krio_write_reg(r, v)		writel(reg_val, reg)
#define krio_read_reg(r)		readl(reg)

#define krio_read(k, r)			readl(&k->regs->r)
#define krio_write(k, r, v)		writel((v), &k->regs->r)

#define krio_car_csr_read(k, r)		readl(&k->car_csr_regs->r)
#define krio_car_csr_write(k, r, v)	writel((v), &k->car_csr_regs->r)

#define krio_car_csr_read_ofs(k, ofs)		\
		readl((void __iomem *)k->car_csr_regs + ofs)
#define krio_car_csr_write_ofs(k, ofs, v)	\
		writel((v), (void __iomem *)k->car_csr_regs + ofs)

#define krio_sp_read(k, r)		readl(&k->serial_port_regs->r)
#define krio_sp_write(k, r, v)		writel((v), &k->serial_port_regs->r)

#define krio_err_read(k, r)		readl(&k->err_mgmt_regs->r)
#define krio_err_write(k, r, v)		writel((v), &k->err_mgmt_regs->r)

#define krio_phy_read(k, r)		readl(&k->phy_regs->r)
#define krio_phy_write(k, r, v)		writel((v), &k->phy_regs->r)

#define krio_tp_read(k, r)		readl(&k->transport_regs->r)
#define krio_tp_write(k, r, v)		writel((v), &k->transport_regs->r)

#define krio_pb_read(k, r)		readl(&k->pkt_buf_regs->r)
#define krio_pb_write(k, r, v)		writel((v), &k->pkt_buf_regs->r)

#define krio_ev_read(k, r)		readl(&k->evt_mgmt_regs->r)
#define krio_ev_write(k, r, v)		writel((v), &k->evt_mgmt_regs->r)

#define krio_pw_read(k, r)		readl(&k->port_write_regs->r)
#define krio_pw_write(k, r, v)		writel((v), &k->port_write_regs->r)

#define krio_lnk_read(k, r)		readl(&k->link_regs->r)
#define krio_lnk_write(k, r, v)		writel((v), &k->link_regs->r)

#define krio_fab_read(k, r)		readl(&k->fabric_regs->r)
#define krio_fab_write(k, r, v)		writel((v), &k->fabric_regs->r)

/*----------------------- Interrupt management -------------------------*/

static irqreturn_t lsu_interrupt_handler(int irq, void *data)
{
#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	struct keystone_rio_data *krio_priv = data;
	u32 pending_err_int = krio_read(krio_priv, lsu_int[0].status);

	u32 pending_lsu_int;

	/* Call DMA interrupt handling */
	while ((pending_lsu_int = krio_read(krio_priv, lsu_int[1].status) &
				  KEYSTONE_RIO_ICSR_LSU1_COMPLETE_MASK)) {
		u32 lsu = __ffs(pending_lsu_int);

		keystone_rio_dma_interrupt_handler(krio_priv, lsu, 0);
		krio_write(krio_priv, lsu_int[1].clear, BIT(lsu));
	}

	/* In case of LSU completion with error */
	if (pending_err_int & KEYSTONE_RIO_ICSR_LSU0_ERROR_MASK) {
		keystone_rio_dma_interrupt_handler(krio_priv, 0, 1);

		krio_write(krio_priv, lsu_int[0].clear,
			   pending_err_int & KEYSTONE_RIO_ICSR_LSU0_ERROR_MASK);
	}
#endif

	return IRQ_HANDLED;
}

static void keystone_rio_reset_symbol_handler(
	struct keystone_rio_data *krio_priv)
{
	/* Disable SerDes lanes asap to generate a loss of link */
	krio_priv->serdes.ops->disable_lanes(krio_priv->board_rio_cfg.lanes,
					     &krio_priv->serdes);

	/* Schedule SRIO peripheral reinitialization */
	schedule_work(&krio_priv->reset_work);
}

static inline void keystone_rio_send_reset(
	struct keystone_rio_data *krio_priv,
	u32 port)
{
	/* Send a reset control symbol on appropriate port */
	krio_sp_write(krio_priv, sp[port].link_maint_req, 0x3);
}

static void keystone_rio_port_error(struct keystone_rio_data *krio_priv,
				    u32 port)
{
	if (error_recovery) {
		dev_dbg(krio_priv->dev, "do recovery of port %d\n", port);

		/* Add port to failed ports and schedule immediate recovery */
		krio_priv->pe_cnt =
			krio_priv->board_rio_cfg.port_register_timeout /
			(KEYSTONE_RIO_REGISTER_DELAY / HZ);
		krio_priv->pe_ports |= BIT(port);

		schedule_delayed_work(&krio_priv->pe_work, 0);
	} else {
		dev_dbg(krio_priv->dev, "send reset symbol to port %d\n", port);

		/* Send reset control symbol to peer */
		if (krio_priv->board_rio_cfg.ports & BIT(port))
			keystone_rio_send_reset(krio_priv, port);

		/* Reset local ports (all) */
		if (!krio_priv->ports_registering)
			keystone_rio_reset_symbol_handler(krio_priv);
	}
}

static void special_interrupt_handler(int ics,
				      struct keystone_rio_data *krio_priv)
{
	/* Acknowledge the interrupt */
	krio_write(krio_priv, err_rst_evnt_int_clear, BIT(ics));

	dev_dbg(krio_priv->dev, "ics = %d\n", ics);

	switch (ics) {
	case KEYSTONE_RIO_MCAST_EVT_INT:
		/* Multi-cast event control symbol interrupt received */
		break;

	case KEYSTONE_RIO_PORT_WRITEIN_INT:
		/* Port-write-in request received on any port */
		keystone_rio_port_write_handler(krio_priv);
		break;

	case KEYSTONE_RIO_EVT_CAP_ERROR_INT:
		/* Logical layer error management event capture */
		keystone_rio_handle_logical_error(krio_priv);
		break;

	case KEYSTONE_RIO_PORT0_ERROR_INT:
	case KEYSTONE_RIO_PORT1_ERROR_INT:
	case KEYSTONE_RIO_PORT2_ERROR_INT:
	case KEYSTONE_RIO_PORT3_ERROR_INT:
		/* Port error */
		keystone_rio_port_error(krio_priv,
					ics - KEYSTONE_RIO_PORT0_ERROR_INT);
		break;

	case KEYSTONE_RIO_RESET_INT:
		/* Device reset interrupt on any port */
		keystone_rio_reset_symbol_handler(krio_priv);
		break;
	}
}

static irqreturn_t rio_interrupt_handler(int irq, void *data)
{
	struct keystone_rio_data *krio_priv = data;
	u32 pending_err_rst_evnt_int =
			krio_read(krio_priv, err_rst_evnt_int_stat) &
			KEYSTONE_RIO_ERR_RST_EVNT_MASK;

	/* Handle special interrupts (error, reset, special event) */
	while (pending_err_rst_evnt_int) {
		u32 ics = __ffs(pending_err_rst_evnt_int);

		pending_err_rst_evnt_int &= ~BIT(ics);
		special_interrupt_handler(ics, krio_priv);
	}

	/* Call doorbell handler(s) */
	dbell_handler(krio_priv);

	return IRQ_HANDLED;
}

/*
 * Map a SRIO event to a SRIO interrupt
 */
static void keystone_rio_interrupt_map(u32 __iomem *reg, u32 mask, u32 rio_int)
{
	int i;
	u32 reg_val;

	reg_val = krio_read_reg(reg);

	for (i = 0; i <= 32; i += 4) {
		if ((mask >> i) & 0xf) {
			reg_val &= ~(0xf << i);
			reg_val |= (rio_int << i);
		}
	}
	krio_write_reg(reg, reg_val);
}

/*
 * Setup RIO interrupts
 */
static int keystone_rio_interrupt_setup(struct keystone_rio_data *krio_priv)
{
	int res;
	u8 lsu;

	/* Clear all pending interrupts */
	krio_write(krio_priv, doorbell_int[0].clear, 0x0000ffff);
	krio_write(krio_priv, doorbell_int[1].clear, 0x0000ffff);
	krio_write(krio_priv, doorbell_int[2].clear, 0x0000ffff);
	krio_write(krio_priv, doorbell_int[3].clear, 0x0000ffff);
	krio_write(krio_priv, err_rst_evnt_int_clear,
		   KEYSTONE_RIO_ERR_RST_EVNT_MASK);

	for (lsu = krio_priv->lsu_start; lsu <= krio_priv->lsu_end; lsu++)
		krio_write(krio_priv, lsu_int[lsu].clear, 0xffffffff);

	/* LSU interrupts are routed to RIO interrupt dest 1 (LSU) */
	keystone_rio_interrupt_map(&krio_priv->regs->lsu0_int_route[0],
				   0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&krio_priv->regs->lsu0_int_route[1],
				   0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&krio_priv->regs->lsu0_int_route[2],
				   0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&krio_priv->regs->lsu0_int_route[3],
				   0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&krio_priv->regs->lsu1_int_route1,
				   0x11111111, KEYSTONE_LSU_RIO_INT);

	/*
	 * Error, reset and special event interrupts are routed to RIO
	 * interrupt dest 0 (Rx/Tx)
	 */
	keystone_rio_interrupt_map(&krio_priv->regs->err_rst_evnt_int_route[0],
				   0x00000111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(&krio_priv->regs->err_rst_evnt_int_route[1],
				   0x00001111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(&krio_priv->regs->err_rst_evnt_int_route[2],
				   0x00000001, KEYSTONE_GEN_RIO_INT);

	/*
	 * The doorbell interrupts routing table is for the 16 general purpose
	 * interrupts
	 */
	krio_write(krio_priv, interrupt_ctl, 0x1);

	/* Do not use pacing */
	krio_write(krio_priv, intdst_rate_disable, 0x0000ffff);

	/* Attach interrupt handlers */
	res = request_irq(krio_priv->board_rio_cfg.rio_irq,
			  rio_interrupt_handler,
			  0,
			  "SRIO",
			  krio_priv);
	if (res) {
		dev_err(krio_priv->dev,
			"Failed to request RIO irq (%d)\n",
			krio_priv->board_rio_cfg.rio_irq);
		return res;
	}

	res = request_irq(krio_priv->board_rio_cfg.lsu_irq,
			  lsu_interrupt_handler,
			  0,
			  "SRIO LSU",
			  krio_priv);
	if (res) {
		dev_err(krio_priv->dev,
			"Failed to request LSU irq (%d)\n",
			krio_priv->board_rio_cfg.lsu_irq);
		free_irq(krio_priv->board_rio_cfg.rio_irq, krio_priv);
		return res;
	}

	return 0;
}

static void keystone_rio_interrupt_release(struct keystone_rio_data *krio_priv)
{
	free_irq(krio_priv->board_rio_cfg.rio_irq, krio_priv);
	free_irq(krio_priv->board_rio_cfg.lsu_irq, krio_priv);
}

/*---------------------------- LSU management -------------------------------*/

u8 keystone_rio_lsu_alloc(struct keystone_rio_data *krio_priv)
{
	u8 lsu_id = krio_priv->lsu_free++;

	if (krio_priv->lsu_free > krio_priv->lsu_end)
		krio_priv->lsu_free = krio_priv->lsu_start;

	return lsu_id;
}

static u32 keystone_rio_lsu_get_cc(u32 lsu_id, u8 ltid, u8 *lcb,
				   struct keystone_rio_data *krio_priv)
{
	u32 idx;
	u32 shift;
	u32 value;
	u32 cc;
	/*  LSU shadow register status mapping */
	u32 lsu_index[8] = { 0, 9, 15, 20, 24, 33, 39, 44 };

	/* Compute LSU stat index from LSU id and LTID */
	idx   = (lsu_index[lsu_id] + ltid) >> 3;
	shift = ((lsu_index[lsu_id] + ltid) & 0x7) << 2;

	/* Get completion code and context */
	value  = krio_read(krio_priv, lsu_stat_reg[idx]);
	cc     = (value >> (shift + 1)) & 0x7;
	*lcb   = (value >> shift) & 0x1;

	return cc;
}

/*
 * Initiate a LSU transfer
 */
int keystone_rio_lsu_start_transfer(int lsu,
				    int port_id,
				    u16 dest_id,
				    dma_addr_t src_addr,
				    u64 tgt_addr,
				    u32 size_bytes,
				    int size,
				    u32 packet_type,
				    u32 *lsu_context,
				    int interrupt_req,
				    struct keystone_rio_data *krio_priv)
{
	u32 count;
	u32 status = 0;
	u32 res = 0;
	u8  lcb;
	u8  ltid;

	if (size_bytes > KEYSTONE_RIO_MAX_DIO_PKT_SIZE)
		return -EINVAL;

	size_bytes &= (KEYSTONE_RIO_MAX_DIO_PKT_SIZE - 1);

	/* If interrupt mode, do not spin */
	if (interrupt_req)
		count = KEYSTONE_RIO_TIMEOUT_CNT;
	else
		count = 0;

	/* Check if there is space in the LSU shadow reg and that it is free */
	while (1) {
		status = krio_read(krio_priv, lsu_reg[lsu].busy_full);
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0) &&
		    ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;

		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_err(krio_priv->dev,
				"no LSU%d shadow register available, status = 0x%x\n",
				lsu, status);
			res = -EBUSY;
			goto out;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}

	/* Get LCB and LTID, LSU reg 6 is already read */
	lcb  = (status >> 4) & 0x1;
	ltid = status & 0xf;
	*lsu_context = status;

	/* LSU Reg 0 - MSB of destination */
	krio_write(krio_priv, lsu_reg[lsu].addr_msb, (u32)(tgt_addr >> 32));

	/* LSU Reg 1 - LSB of destination */
	krio_write(krio_priv, lsu_reg[lsu].addr_lsb_cfg_ofs, (u32)tgt_addr);

	/* LSU Reg 2 - source address */
	krio_write(krio_priv, lsu_reg[lsu].phys_addr, src_addr);

	/* LSU Reg 3 - byte count */
	krio_write(krio_priv, lsu_reg[lsu].dbell_val_byte_cnt, size_bytes);

	/*
	 * LSU Reg 4 -
	 * out port ID = rio.port
	 * priority = LSU_PRIO
	 * Xambs = 0
	 * ID size = 8 or 16 bit
	 * Dest ID specified as arg
	 * interrupt request
	 */
	krio_write(krio_priv, lsu_reg[lsu].destid,
		   ((port_id << 8)
		    | (KEYSTONE_RIO_LSU_PRIO << 4)
		    | (size ? BIT(10) : 0)
		    | ((u32)dest_id << 16)
		    | (interrupt_req & 0x1)));

	/*
	 * LSU Reg 5 -
	 * doorbell info = packet_type[16-31],
	 * hop count = packet_type [8-15]
	 * FType = packet_type[4-7], TType = packet_type[0-3]
	 * Writing this register will initiate the transfer
	 */
	krio_write(krio_priv, lsu_reg[lsu].dbell_info_fttype, packet_type);

out:
	return res;
}

/*
 * Cancel a LSU transfer
 */
static inline void keystone_rio_lsu_cancel_transfer(
	int lsu,
	struct keystone_rio_data *krio_priv)
{
	u32 status;
	u32 count = 0;

	while (1) {
		/* Read register 6 to get the lock */
		status = krio_read(krio_priv, lsu_reg[lsu].busy_full);

		/* If not busy or if full, we can flush */
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0) ||
		    (status & KEYSTONE_RIO_LSU_BUSY_MASK))
			break;

		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_err(krio_priv->dev,
				"no LSU%d shadow register available for flushing\n",
				lsu);
			return;
		}

		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}

	if (status & KEYSTONE_RIO_LSU_FULL_MASK) {
		/* Flush the transaction with our privID */
		krio_write(krio_priv, lsu_reg[lsu].busy_full,
			   BIT(0) | (8 << 28));
	} else {
		/* Flush the transaction with our privID and CBusy bit */
		krio_write(krio_priv, lsu_reg[lsu].busy_full,
			   BIT(0) | (8 << 28) | BIT(27));
	}
}

/*
 * Complete a LSU transfer
 */
int keystone_rio_lsu_complete_transfer(int lsu,
				       u32 lsu_context,
				       struct keystone_rio_data *krio_priv)
{
	u32 status = 0;
	u32 res = 0;
	u8  lcb = (lsu_context >> 4) & 0x1;
	u8  ltid = (lsu_context) & 0xf;
	u8  n_lcb;

	/* Retrieve our completion code */
	status = keystone_rio_lsu_get_cc(lsu, ltid, &n_lcb, krio_priv);
	if (n_lcb != lcb)
		return -EAGAIN;

	if (unlikely(status))
		dev_dbg(krio_priv->dev, "LSU%d status 0x%x\n", lsu, status);

	switch (status & KEYSTONE_RIO_LSU_CC_MASK) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
		res = -ETIMEDOUT;
		keystone_rio_lsu_cancel_transfer(lsu, krio_priv);
		break;
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		res = -EIO;
		keystone_rio_lsu_cancel_transfer(lsu, krio_priv);
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
		res = -EBUSY;
		break;
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		res = -EIO;
		break;
	default:
		break;
	}

	return res;
}

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
static int keystone_rio_lsu_dma_allocate_channel(struct rio_mport *mport)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct dma_chan *dchan;

	dchan = rio_request_mport_dma(mport);
	if (!dchan) {
		dev_err(krio_priv->dev,
			"cannot find DMA channel for port %d\n",
			mport->index);
		return -ENODEV;
	}

	dev_dbg(krio_priv->dev, "get channel 0x%p for port %d\n",
		dchan, mport->index);

	krio_priv->dma_chan[mport->index] = dchan;

	return 0;
}

static void keystone_rio_lsu_dma_free_channel(struct rio_mport *mport)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct dma_chan *dchan = krio_priv->dma_chan[mport->index];

	if (dchan) {
		struct keystone_rio_dma_chan *chan = from_dma_chan(dchan);

		/* Remove from global list */
		list_del_init(&chan->node);

		rio_release_dma(dchan);
		krio_priv->dma_chan[mport->index] = NULL;
	}
}
#endif /* CONFIG_RAPIDIO_DMA_ENGINE */

/*----------------------------- Doorbell management --------------------------*/

static inline int dbell_get(u32 *pending)
{
	if (*pending) {
		int n = __ffs(*pending);
		*pending &= ~BIT(n);
		return n;
	}

	return -1;
}

static inline void dbell_call_handler(u32 dbell_num,
				      struct keystone_rio_data *krio_priv)
{
	struct rio_dbell *dbell;
	int i;
	int found = 0;

	for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++) {
		if (krio_priv->mport[i]) {
			struct rio_mport *mport = krio_priv->mport[i];

			list_for_each_entry(dbell, &mport->dbells, node) {
				if ((dbell->res->start <= dbell_num) &&
				    (dbell->res->end   >= dbell_num)) {
					found = 1;
					break;
				}
			}

			if (found && dbell->dinb) {
				/* Call the registered handler if any */
				dbell->dinb(mport,
					    dbell->dev_id,
					    -1, /* unknown source Id */
					    mport->host_deviceid,
					    dbell_num);
				break;
			}
		}
	}

	if (!found)
		dev_dbg(krio_priv->dev,
			"DBELL: received spurious doorbell %d\n",
			dbell_num);
}

static void dbell_handler(struct keystone_rio_data *krio_priv)
{
	u32 pending_dbell;
	unsigned int i;

	for (i = 0; i < KEYSTONE_RIO_DBELL_NUMBER; i++) {
		pending_dbell = krio_read(krio_priv, doorbell_int[i].status);
		if (pending_dbell)
			/* Acknowledge the interrupts for these doorbells */
			krio_write(krio_priv, doorbell_int[i].clear,
				   pending_dbell);

		while (pending_dbell) {
			u32 dbell_num = dbell_get(&pending_dbell) + (i << 4);

			/* Call the registered dbell handler(s) */
			dbell_call_handler(dbell_num, krio_priv);
		}
	}
}

static int keystone_rio_dbell_send(struct rio_mport *mport,
				   int index,
				   u16 dest_id,
				   u16 num)
{
#ifdef CONFIG_RAPIDIO_DMA_ENGINE

	struct keystone_rio_data *krio_priv = mport->priv;
	u32 lsu_context;
	u32 count = 0;
	int res;

	/* Transform doorbell number into info field */
	u16 info   = (num & 0xf) | (((num >> 4) & 0x3) << 5);
	u32 packet_type = ((info & 0xffff) << 16)
				| KEYSTONE_RIO_PACKET_TYPE_DBELL;

	mutex_lock(&krio_priv->lsu_lock_maint);
	res = keystone_rio_lsu_start_transfer(krio_priv->lsu_maint,
					      mport->index,
					      dest_id,
					      0,
					      0,
					      0,
					      mport->sys_size,
					      packet_type,
					      &lsu_context,
					      1,
					      krio_priv);
	if (res) {
		mutex_unlock(&krio_priv->lsu_lock_maint);
		dev_err(krio_priv->dev, "DIO: send doorbell error %d\n", res);
		return res;
	}

	while (1) {
		res = keystone_rio_lsu_complete_transfer(krio_priv->lsu_maint,
							 lsu_context,
							 krio_priv);
		if (res != -EAGAIN)
			break;

		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			break;
		}

		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}

	mutex_unlock(&krio_priv->lsu_lock_maint);

	dev_info(krio_priv->dev, "DIO: send doorbell complete %u\n", count);
	return res;
#else /* CONFIG_RAPIDIO_DMA_ENGINE */

	return -ENOTSUPP;

#endif /* CONFIG_RAPIDIO_DMA_ENGINE */
}

/*---------------------- Maintenance Request Management  ---------------------*/

/**
 * keystone_rio_maint_request - Perform a maintenance request
 * @port_id: output port ID
 * @dest_id: destination ID of target device
 * @hopcount: hopcount for this request
 * @offset: offset in the RapidIO configuration space
 * @buff: dma address of the data on the host
 * @buff_len: length of the data
 * @size: 1 for 16bit, 0 for 8bit ID size
 * @type: packet type
 *
 * Returns %0 on success or %-EINVAL, %-EIO, %-EAGAIN or %-EBUSY on failure.
 */
static inline int keystone_rio_maint_request(
	int port_id,
	u32 dest_id,
	u8  hopcount,
	u32 offset,
	dma_addr_t buff,
	int buff_len,
	u16 size,
	u16 packet_type,
	struct keystone_rio_data *krio_priv)
{
	int res;
	u32 lsu_context;
	u32 count = 0;
	u32 type = ((hopcount & 0xff) << 8) | (packet_type & 0xff);

	mutex_lock(&krio_priv->lsu_lock_maint);

	res = keystone_rio_lsu_start_transfer(krio_priv->lsu_maint,
					      port_id,
					      dest_id,
					      buff,
					      offset,
					      buff_len,
					      size,
					      type,
					      &lsu_context,
					      0,
					      krio_priv);
	if (res) {
		mutex_unlock(&krio_priv->lsu_lock_maint);
		dev_err(krio_priv->dev, "maintenance packet transfer error\n");
		return res;
	}

	while (1) {
		res = keystone_rio_lsu_complete_transfer(krio_priv->lsu_maint,
							 lsu_context,
							 krio_priv);
		if (res != -EAGAIN)
			break;

		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			break;
		}

		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}

	mutex_unlock(&krio_priv->lsu_lock_maint);

	return res;
}

static int keystone_rio_maint_read(struct keystone_rio_data *krio_priv,
				   int port_id,
				   u16 destid,
				   u16 size,
				   u8  hopcount,
				   u32 offset,
				   int len,
				   u32 *val)
{
	u32 *tbuf;
	int res;
	dma_addr_t dma;
	struct device *dev = krio_priv->dev;
	size_t align_len = L1_CACHE_ALIGN(len);

	tbuf = kzalloc(align_len, GFP_KERNEL | GFP_DMA);
	if (!tbuf)
		return -ENOMEM;

	dma = dma_map_single(dev, tbuf, len, DMA_FROM_DEVICE);

	res = keystone_rio_maint_request(port_id,
					 destid,
					 hopcount,
					 offset,
					 dma,
					 len,
					 size,
					 KEYSTONE_RIO_PACKET_TYPE_MAINT_R,
					 krio_priv);

	dma_unmap_single(dev, dma, len, DMA_FROM_DEVICE);

	/* Taking care of byteswap */
	switch (len) {
	case 1:
		*val = *((u8 *)tbuf);
		break;
	case 2:
		*val = ntohs(*((u16 *)tbuf));
		break;
	default:
		*val = ntohl(*((u32 *)tbuf));
		break;
	}

	dev_dbg(dev,
		"maint_r: index %d destid %d hopcount %d offset 0x%x len %d val 0x%x res %d\n",
		port_id, destid, hopcount, offset, len, *val, res);

	kfree(tbuf);

	return res;
}

static int keystone_rio_maint_write(struct keystone_rio_data *krio_priv,
				    int port_id,
				    u16 destid,
				    u16 size,
				    u8  hopcount,
				    u32 offset,
				    int len,
				    u32 val)
{
	u32 *tbuf;
	int res;
	dma_addr_t dma;
	struct device *dev = krio_priv->dev;
	size_t align_len = L1_CACHE_ALIGN(len);

	tbuf = kzalloc(align_len, GFP_KERNEL | GFP_DMA);
	if (!tbuf)
		return -ENOMEM;

	/* Taking care of byteswap */
	switch (len) {
	case 1:
		*tbuf = ((u8)val);
		break;
	case 2:
		*tbuf = htons((u16)val);
		break;
	default:
		*tbuf = htonl((u32)val);
		break;
	}

	dma = dma_map_single(dev, tbuf, len, DMA_TO_DEVICE);

	res = keystone_rio_maint_request(port_id,
					 destid,
					 hopcount,
					 offset,
					 dma,
					 len,
					 size,
					 KEYSTONE_RIO_PACKET_TYPE_MAINT_W,
					 krio_priv);

	dma_unmap_single(dev, dma, len, DMA_TO_DEVICE);

	dev_dbg(dev,
		"maint_w: index %d destid %d hopcount %d offset 0x%x len %d val 0x%x res %d\n",
		port_id, destid, hopcount, offset, len, val, res);

	kfree(tbuf);

	return res;
}

/*------------------------- RapidIO hw controller setup ---------------------*/

/* Retrieve the corresponding lanes bitmask from ports bitmask and path_mode */
static int keystone_rio_get_lane_config(u32 ports, u32 path_mode)
{
	u32 lanes = 0;

	while (ports) {
		u32 lane;
		u32 port = __ffs(ports);

		ports &= ~BIT(port);

		if (keystone_lane_configs[path_mode][port].start == -1)
			return -1;

		for (lane = keystone_lane_configs[path_mode][port].start;
		     lane < keystone_lane_configs[path_mode][port].end;
		     lane++) {
			lanes |= KEYSTONE_SERDES_LANE(lane);
		}
	}

	return (int)lanes;
}

/**
 * keystone_rio_lanes_init_and_wait - Initialize and wait lanes for a given
 * RIO port
 *
 * @port: RIO port
 * @start: if non null, lanes will be started
 * @init_rx: if non null, lanes Rx coefficients will be applied
 *
 * Returns %0 on success or %1 if lane is not OK during the expected timeout
 */
static int keystone_rio_lanes_init_and_wait(u32 port, int start,
					    struct keystone_rio_data *krio_priv)
{
	u32 path_mode = krio_priv->board_rio_cfg.path_mode;
	int lanes     = keystone_rio_get_lane_config(BIT(port), path_mode);
	int res;

	dev_dbg(krio_priv->dev,
		"initializing lane mask 0x%x for port %d",
		lanes, port);

	/* Eventually start the lane */
	if (start) {
		dev_dbg(krio_priv->dev,
			"starting lane mask 0x%x for port %d",
			lanes, port);

		krio_priv->serdes.ops->start_tx_lanes((u32)lanes,
						      &krio_priv->serdes);
	}

	/* Wait lanes to be OK */
	res = krio_priv->serdes.ops->wait_lanes_ok(lanes,
						   &krio_priv->serdes);
	if (res < 0) {
		dev_dbg(krio_priv->dev,
			"port %d lane mask 0x%x is not OK\n",
			port, lanes);

		return 1;
	}

	return 0;
}

/*
 * SerDes main configuration
 */
static int keystone_rio_serdes_init(u32 baud, int calibrate,
				    struct keystone_rio_data *krio_priv)
{
	u32 path_mode = krio_priv->board_rio_cfg.path_mode;
	u32 ports = krio_priv->board_rio_cfg.ports;
	u32 lanes;
	int res;

	/* Retrieve lane termination */
	res = keystone_rio_get_lane_config(ports, path_mode);
	if (res <= 0)
		return res;

	/* Initialize SerDes */
	lanes = (u32)res;
	krio_priv->board_rio_cfg.lanes = lanes;

	res = krio_priv->serdes.ops->config_lanes(lanes,
						  baud,
						  &krio_priv->serdes);
	if (res < 0)
		return res;

	/* Check if we need to calibrate SerDes */
	if (calibrate && !krio_priv->calibrating) {
		krio_priv->calibrating = 1;

		/* Set calibration timeout */
		krio_priv->board_rio_cfg.serdes_config.cal_timeout =
			krio_priv->board_rio_cfg.port_register_timeout;

		/* Do the calibration */
		krio_priv->serdes.ops->calibrate_lanes(lanes,
						       &krio_priv->serdes);

		krio_priv->calibrating = 0;
	}

	return 0;
}

/**
 * keystone_rio_hw_init - Configure a RapidIO controller
 * @baud: serdes baudrate
 *
 * Returns %0 on success or %-EINVAL or %-EIO on failure.
 */
static int keystone_rio_hw_init(u32 baud, struct keystone_rio_data *krio_priv)
{
	struct keystone_serdes_config *serdes_config =
			&krio_priv->board_rio_cfg.serdes_config;
	u32 lsu_mask = 0, val, block, port;
	int res = 0, i;

	/* Reset blocks */
	krio_write(krio_priv, gbl_en, 0);
	for (block = 0; block < KEYSTONE_RIO_BLK_NUM; block++) {
		krio_write(krio_priv, blk[block].enable, 0);
		while (krio_read(krio_priv, blk[block].status) & 0x1)
			usleep_range(10, 50);
	}

	ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);

	/* Set SRIO out of reset */
	krio_write(krio_priv, pcr,
		   KEYSTONE_RIO_PER_RESTORE | KEYSTONE_RIO_PER_FREE);

	/* Clear BOOT_COMPLETE bit (allowing write) */
	krio_write(krio_priv, per_set_cntl, 0);

	/* Set LSU timeout count to zero */
	krio_write(krio_priv, lsu_setup_reg[1], 0x000000ff);

	/* Enable blocks */
	krio_write(krio_priv, gbl_en, 1);
	for (block = 0; block < KEYSTONE_RIO_BLK_NUM; block++)
		krio_write(krio_priv, blk[block].enable, 1);

	/* Set control register 1 configuration */
	krio_write(krio_priv, per_set_cntl1, 0);

	/* Set control register */
	krio_write(krio_priv, per_set_cntl,
		   0x0009c000 | (krio_priv->board_rio_cfg.pkt_forwarding ?
				 BIT(21) | BIT(8) : 0));

	/* Initialize SerDes and eventually perform their calibration */
	res = keystone_rio_serdes_init(
		baud,
		krio_priv->board_rio_cfg.serdes_calibration,
		krio_priv);

	if (res < 0) {
		dev_err(krio_priv->dev, "initialization of SerDes failed\n");
		return res;
	}

	/* Set prescalar for ip_clk */
	krio_lnk_write(krio_priv, prescalar_srv_clk,
		       serdes_config->prescalar_srv_clk);

	/* Peripheral-specific configuration and capabilities */
	krio_car_csr_write(krio_priv, dev_id, KEYSTONE_RIO_DEV_ID_VAL);
	krio_car_csr_write(krio_priv, dev_info, KEYSTONE_RIO_DEV_INFO_VAL);
	krio_car_csr_write(krio_priv, assembly_id, KEYSTONE_RIO_ID_TI);
	krio_car_csr_write(krio_priv, assembly_info, KEYSTONE_RIO_EXT_FEAT_PTR);
	krio_car_csr_write(krio_priv, base_dev_id, krio_priv->base_dev_id);

	krio_priv->rio_pe_feat = RIO_PEF_PROCESSOR
				 | RIO_PEF_CTLS
				 | KEYSTONE_RIO_PEF_FLOW_CONTROL
				 | RIO_PEF_EXT_FEATURES
				 | RIO_PEF_ADDR_34
				 | RIO_PEF_STD_RT
				 | RIO_PEF_INB_DOORBELL
				 | RIO_PEF_INB_MBOX;

	krio_car_csr_write(krio_priv, pe_feature, krio_priv->rio_pe_feat);
	krio_car_csr_write(krio_priv, sw_port, KEYSTONE_RIO_MAX_PORT << 8);

	krio_car_csr_write(krio_priv, src_op,
			   RIO_SRC_OPS_READ
			   | RIO_SRC_OPS_WRITE
			   | RIO_SRC_OPS_STREAM_WRITE
			   | RIO_SRC_OPS_WRITE_RESPONSE
			   | RIO_SRC_OPS_DATA_MSG
			   | RIO_SRC_OPS_DOORBELL
			   | RIO_SRC_OPS_ATOMIC_TST_SWP
			   | RIO_SRC_OPS_ATOMIC_INC
			   | RIO_SRC_OPS_ATOMIC_DEC
			   | RIO_SRC_OPS_ATOMIC_SET
			   | RIO_SRC_OPS_ATOMIC_CLR
			   | RIO_SRC_OPS_PORT_WRITE);

	krio_car_csr_write(krio_priv, dest_op,
			   RIO_DST_OPS_READ
			   | RIO_DST_OPS_WRITE
			   | RIO_DST_OPS_STREAM_WRITE
			   | RIO_DST_OPS_WRITE_RESPONSE
			   | RIO_DST_OPS_DATA_MSG
			   | RIO_DST_OPS_DOORBELL
			   | RIO_DST_OPS_PORT_WRITE);

	krio_car_csr_write(krio_priv, pe_logical_ctl, RIO_PELL_ADDR_34);

	val = (((KEYSTONE_RIO_SP_HDR_NEXT_BLK_PTR & 0xffff) << 16) |
	       KEYSTONE_RIO_SP_HDR_EP_REC_ID);
	krio_sp_write(krio_priv, sp_maint_blk_hdr, val);

	/* Clear high bits of local config space base addr */
	krio_car_csr_write(krio_priv, local_cfg_hbar, 0);

	/* Set local config space base addr */
	krio_car_csr_write(krio_priv, local_cfg_bar, 0x00520000);

	/* Enable HOST BIT(31) & MASTER_ENABLE BIT(30) bits */
	krio_sp_write(krio_priv, sp_gen_ctl, 0xc0000000);

	/* Set link timeout value */
	krio_sp_write(krio_priv, sp_link_timeout_ctl, 0x000FFF00);

	/* Set response timeout value */
	krio_sp_write(krio_priv, sp_rsp_timeout_ctl, 0x01000000);

	/* Allows SELF_RESET and PWDN_PORT resets to clear stcky reg bits */
	krio_lnk_write(krio_priv, reg_rst_ctl, 0x00000001);

	/* Clear all errors */
	krio_err_write(krio_priv, err_det, 0);
	krio_lnk_write(krio_priv, local_err_det, 0);

	/* Set error detection */
	if (krio_priv->board_rio_cfg.pkt_forwarding) {
		/* Disable all error detection if using packet forwarding */
		krio_lnk_write(krio_priv, local_err_en, 0);
		krio_err_write(krio_priv, err_en, 0);
	} else {
		/* Enable logical layer error detection */
		krio_err_write(krio_priv, err_en, BIT(24) | BIT(25) | BIT(31));
		krio_lnk_write(krio_priv, local_err_en, BIT(22) | BIT(26));
	}

	/* Set err det block header */
	val = (((KEYSTONE_RIO_ERR_HDR_NEXT_BLK_PTR & 0xffff) << 16) |
	       KEYSTONE_RIO_ERR_EXT_FEAT_ID);
	krio_err_write(krio_priv, err_report_blk_hdr, val);

	/* Clear msb of err captured addr reg */
	krio_err_write(krio_priv, h_addr_capt, 0);

	/* Clear lsb of err captured addr reg */
	krio_err_write(krio_priv, addr_capt, 0);

	/* Clear err captured source and dest DevID reg */
	krio_err_write(krio_priv, id_capt, 0);

	/* Clear err captured packet info */
	krio_err_write(krio_priv, ctrl_capt, 0);

	/* Set per port information */
	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		krio_phy_write(krio_priv, phy_sp[port].__rsvd[3], 0x41004141);

		/* Set the baud rate to the port information */
		val = krio_sp_read(krio_priv, sp[port].ctl2);
		val |= BIT(24 - (baud << 1));
		krio_sp_write(krio_priv, sp[port].ctl2, val);
	}

	/* Disable LSU to perform LSU configuration */
	krio_write(krio_priv, blk[KEYSTONE_RIO_BLK_LSU_ID].enable, 0);
	while (krio_read(krio_priv, blk[KEYSTONE_RIO_BLK_LSU_ID].status) & 0x1)
		usleep_range(10, 50);

	/* Set the SRIO shadow registers configuration to 4/4/4/4 */
	krio_write(krio_priv, lsu_setup_reg[0], 0);

	/* Use LSU completion interrupt per LSU (not per SRCID) */
	for (i = krio_priv->lsu_start; i <= krio_priv->lsu_end; i++)
		lsu_mask |= BIT(i);

	krio_write(krio_priv, lsu_setup_reg[1], lsu_mask);

	/* Enable LSU */
	krio_write(krio_priv, blk[KEYSTONE_RIO_BLK_LSU_ID].enable, 1);

	/* Global port-write generation */
	if (krio_priv->board_rio_cfg.pkt_forwarding) {
		/*
		 * Disable generation of port-write requests if using
		 * packet forwarding
		 */
		val = krio_ev_read(krio_priv, evt_mgmt_port_wr_enable);
		krio_ev_write(krio_priv, evt_mgmt_port_wr_enable,
			      val & ~(BIT(8) | BIT(28))); /* LOG | LOCALOG */

		val = krio_ev_read(krio_priv, evt_mgmt_dev_port_wr_en);
		krio_ev_write(krio_priv, evt_mgmt_dev_port_wr_en,
			      val & ~BIT(0)); /* PW_EN */
	} else {
		/*
		 * Enable generation of port-write requests
		 */
		val = krio_ev_read(krio_priv, evt_mgmt_port_wr_enable);
		krio_ev_write(krio_priv, evt_mgmt_port_wr_enable,
			      val | BIT(8) | BIT(28)); /* LOG | LOCALOG */

		val = krio_ev_read(krio_priv, evt_mgmt_dev_port_wr_en);
		krio_ev_write(krio_priv, evt_mgmt_dev_port_wr_en,
			      val  | BIT(0)); /* PW_EN */
	}

	/* Set packet forwarding */
	for (i = 0; i < KEYSTONE_RIO_MAX_PKT_FW_ENTRIES; i++) {
		if ((krio_priv->board_rio_cfg.pkt_forwarding) && (i < 8)) {
			struct keystone_routing_config *routing =
				krio_priv->board_rio_cfg.routing_config;

			/*
			 * Enable packet forwarding DevId and port as defined
			 * in DTS
			 */
			krio_write(krio_priv, pkt_fwd_cntl[i].pf_16b,
				   routing[i].dev_id_low
				   | routing[i].dev_id_high << 16);

			krio_write(krio_priv, pkt_fwd_cntl[i].pf_8b,
				   (routing[i].dev_id_low & 0xff)
				   | (routing[i].dev_id_high & 0xff) << 8
				   | routing[i].port << 16);

			dev_info(krio_priv->dev,
				 "enabling packet forwarding to port %d for DestID 0x%04x - 0x%04x\n",
				 routing[i].port, routing[i].dev_id_low,
				 routing[i].dev_id_high);
		} else {
			/* Disable packet forwarding */
			krio_write(krio_priv, pkt_fwd_cntl[i].pf_16b,
				   0xffffffff);
			krio_write(krio_priv, pkt_fwd_cntl[i].pf_8b,
				   0xffffffff);
		}
	}

	if (!krio_priv->board_rio_cfg.pkt_forwarding)
		dev_info(krio_priv->dev, "packet forwarding disabled\n");

	/* Force all writes to finish */
	val = krio_err_read(krio_priv, ctrl_capt);

	return res;
}

/**
 * keystone_rio_start - Start RapidIO controller
 */
static void keystone_rio_start(struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Set PEREN bit to enable logical layer data flow */
	krio_write(krio_priv, pcr, KEYSTONE_RIO_PER_EN | KEYSTONE_RIO_PER_FREE);

	/* Set BOOT_COMPLETE bit */
	val = krio_read(krio_priv, per_set_cntl);
	krio_write(krio_priv, per_set_cntl, val | KEYSTONE_RIO_BOOT_COMPLETE);
}

static void keystone_rio_reset_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv =
		container_of(work, struct keystone_rio_data, reset_work);
	u32 ports_rst;
	u32 ports;
	u32 port;

	if (krio_priv->started == 0)
		return;

	ports_rst = krio_ev_read(krio_priv, evt_mgmt_rst_port_stat);

	dev_dbg(krio_priv->dev,
		"reset device request received on ports: 0x%x\n", ports_rst);

	/* Acknowledge reset */
	ports = ports_rst;
	while (ports) {
		port = __ffs(ports);
		ports &= ~BIT(port);
		krio_phy_write(krio_priv, phy_sp[port].status,
			       KEYSTONE_RIO_PORT_PLM_STATUS_RST_REQ);
	}

	krio_ev_write(krio_priv, evt_mgmt_rst_port_stat, ports_rst);

	/* Reinitialize SRIO peripheral */
	keystone_rio_shutdown_controller(krio_priv);
	keystone_rio_setup_controller(krio_priv);
}

/**
 * keystone_rio_stop - Stop RapidIO controller
 */
static void keystone_rio_stop(struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Disable PEREN bit to stop all new logical layer transactions */
	val = krio_read(krio_priv, pcr);
	krio_write(krio_priv, pcr, val & ~KEYSTONE_RIO_PER_EN);

	/* Clear BOOT_COMPLETE bit */
	val = krio_read(krio_priv, per_set_cntl);
	krio_write(krio_priv, per_set_cntl, val & ~KEYSTONE_RIO_BOOT_COMPLETE);
}

static void keystone_rio_handle_logical_error(
	struct keystone_rio_data *krio_priv)
{
	u32 err_det = krio_err_read(krio_priv, err_det);

	while (err_det) {
		u32 err = __ffs(err_det);
		u32 val;

		err_det &= ~BIT(err);

		/* Acknowledge logical layer error */
		val = krio_err_read(krio_priv, err_det);
		krio_err_write(krio_priv, err_det, val & ~BIT(err));

		dev_dbg(krio_priv->dev,
			"logical layer error %d detected\n", err);

		/* Acknowledge local logical layer error as well if needed */
		if ((err == 22) || (err == 26)) {
			val = krio_lnk_read(krio_priv, local_err_det);
			krio_lnk_write(krio_priv, local_err_det,
				       val & ~BIT(err));
		}
	}
}

static int keystone_rio_get_remote_port(
	u8 port,
	struct keystone_rio_data *krio_priv)
{
	int res;
	u32 value;

	res = keystone_rio_maint_read(krio_priv, port, 0xffff,
				      krio_priv->board_rio_cfg.size,
				      0, RIO_SWP_INFO_CAR, 4, &value);
	if (res < 0)
		return res;

	return RIO_GET_PORT_NUM(value);
}

static int keystone_rio_port_sync_ackid(u32 port,
					struct keystone_rio_data *krio_priv)
{
	u32 lm_resp, ackid_stat, l_ackid, r_ackid;
	int i = 0;

	/*
	 * Clear valid bit in maintenance response register.
	 * Send both Input-Status Link-Request and PNA control symbols and
	 * wait for valid maintenance response
	 */
	krio_sp_read(krio_priv, sp[port].link_maint_resp);
	krio_phy_write(krio_priv, phy_sp[port].long_cs_tx1, 0x2003f044);

	do {
		if (++i > KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_err(krio_priv->dev,
				"port %d: Input-Status response timeout\n",
				port);
			return -1;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
		lm_resp = krio_sp_read(krio_priv, sp[port].link_maint_resp);
	} while (!(lm_resp & RIO_PORT_N_MNT_RSP_RVAL));

	dev_dbg(krio_priv->dev,
		"port %d: Input-Status response = 0x%08x\n", port, lm_resp);

	/* Set outbound ackID to the value expected by link partner */
	ackid_stat = krio_sp_read(krio_priv, sp[port].ackid_stat);
	dev_dbg(krio_priv->dev, "port %d: ackid_stat = 0x%08x\n",
		port, ackid_stat);

	l_ackid = (ackid_stat & RIO_PORT_N_ACK_INBOUND) >> 24;
	r_ackid = (lm_resp & RIO_PORT_N_MNT_RSP_ASTAT) >> 5;
	krio_sp_write(krio_priv, sp[port].ackid_stat, l_ackid << 24 | r_ackid);

	return (int)l_ackid;
}

static int keystone_rio_port_error_recovery(u32 port,
					    struct keystone_rio_data *krio_priv)
{
	int res;
	u32 err_stat;
	u32 err_det;
	u32 plm_status;
	int r_port = krio_priv->board_rio_cfg.ports_remote[port];

	if (unlikely(port >= KEYSTONE_RIO_MAX_PORT))
		return -EINVAL;

	err_stat   = krio_sp_read(krio_priv, sp[port].err_stat);
	err_det    = krio_err_read(krio_priv, sp_err[port].det);
	plm_status = krio_phy_read(krio_priv, phy_sp[port].status);

	dev_dbg(krio_priv->dev,
		"ER port %d: err_stat = 0x%08x, err_det = 0x%08x, plm_status = 0x%08x\n",
		port, err_stat, err_det, plm_status);

	if (unlikely(!(err_stat & RIO_PORT_N_ERR_STS_PORT_OK))) {
		dev_dbg(krio_priv->dev,
			"ER port %d not initialized - PORT_OK not set\n", port);
		return -EINVAL;
	}

	/* Acknowledge errors on this port */
	krio_sp_write(krio_priv, sp[port].err_stat,
		      err_stat & KEYSTONE_RIO_PORT_ERROR_MASK);
	krio_err_write(krio_priv, sp_err[port].det, 0);
	krio_phy_write(krio_priv, phy_sp[port].status,
		       plm_status & KEYSTONE_RIO_PORT_PLM_STATUS_ERRORS);

	if (err_stat & RIO_PORT_N_ERR_STS_PW_OUT_ES) {
		u32 ackid_stat, l_ackid, r_ackid;

		/* Sync ackID */
		res = keystone_rio_port_sync_ackid(port, krio_priv);
		if (res == -1)
			goto oes_rd_err;

		l_ackid = (u32)res;

		/*
		 * We do not know the remote port but we may be lucky where
		 * ackId did not changed...
		 */
		if (r_port < 0) {
			dev_dbg(krio_priv->dev,
				"ER port %d: remote port not yet detected!\n",
				port);
			return -EINVAL;
		}

		/* range values are by experiment */
		usleep_range(50, 200);

		/*
		 * Reread outbound ackID as it may have changed as a result of
		 * outstanding unacknowledged packets retransmission
		 */
		ackid_stat = krio_sp_read(krio_priv, sp[port].ackid_stat);
		dev_dbg(krio_priv->dev, "ER port %d: ackid_stat = 0x%08x\n",
			port, ackid_stat);

		r_ackid = ackid_stat & RIO_PORT_N_ACK_OUTBOUND;

		/*
		 * Set link partner inbound ackID to outbound ackID + 1.
		 * Set link partner outbound and outstanding ackID to inbound
		 * ackID.
		 */
		res = keystone_rio_maint_write(
			krio_priv,
			port,
			0xffff,
			krio_priv->board_rio_cfg.size,
			0,
			0x100 + RIO_PORT_N_ACK_STS_CSR(r_port),
			sizeof(u32),
			((++r_ackid << 24) & RIO_PORT_N_ACK_INBOUND) |
			(l_ackid << 8) | l_ackid);

		if (res < 0) {
			dev_dbg(krio_priv->dev,
				"ER port %d: failed to align ackIDs with link partner port %d\n",
				port, r_port);
		}
	}

oes_rd_err:

	if (err_stat & RIO_PORT_N_ERR_STS_PW_INP_ES) {
		if (r_port < 0) {
			dev_dbg(krio_priv->dev,
				"ER port %d: remote port not yet detected!\n",
				port);
			return -EINVAL;
		}

		dev_dbg(krio_priv->dev,
			"ER port %d: Input Error-Stopped recovery\n", port);

		res = keystone_rio_maint_write(
			krio_priv,
			port,
			0xffff,
			krio_priv->board_rio_cfg.size,
			0,
			0x100 + RIO_PORT_N_MNT_REQ_CSR(r_port),
			sizeof(u32),
			RIO_MNT_REQ_CMD_IS);

		if (res < 0) {
			dev_dbg(krio_priv->dev,
				"ER port %d: failed to issue Input-Status request from link partner port %d\n",
				port, r_port);
		}

		/* range values are by experiment */
		usleep_range(50, 200);
	}

	err_stat   = krio_sp_read(krio_priv, sp[port].err_stat);
	err_det    = krio_err_read(krio_priv, sp_err[port].det);
	plm_status = krio_phy_read(krio_priv, phy_sp[port].status);

	dev_dbg(krio_priv->dev,
		"ER port %d: ending with err_stat = 0x%08x, err_det = 0x%08x, plm_status = 0x%08x\n",
		port, err_stat, err_det, plm_status);

	return err_stat & KEYSTONE_RIO_PORT_ERRORS;
}

static void keystone_rio_pe_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv = container_of(
		to_delayed_work(work), struct keystone_rio_data, pe_work);
	u32 port;

	if (krio_priv->started == 0)
		return;

	dev_dbg(krio_priv->dev, "ER errors on ports: 0x%x\n",
		krio_priv->pe_ports);

	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		/* Skip port if we are currently registering it */
		if (krio_priv->ports_registering & BIT(port))
			continue;

		if (test_and_clear_bit(port, (void *)&krio_priv->pe_ports)) {
			/* Wait lanes to be OK */
			if (keystone_rio_lanes_init_and_wait(port, 0,
							     krio_priv)) {
				dev_dbg(krio_priv->dev,
					"ER port %d: lanes are not OK\n", port);
				krio_priv->pe_ports |= BIT(port);
			}

			/*  Recover from port error state */
			if (keystone_rio_port_error_recovery(port,
							     krio_priv)) {
				dev_dbg(krio_priv->dev,
					"ER port %d: failed to perform error recovery\n",
					port);
				krio_priv->pe_ports |= BIT(port);
			}
		}
	}

	/* If error recovery failed delay another one if there is time left */
	if (krio_priv->pe_ports) {
		if (krio_priv->pe_cnt-- > 1) {
			schedule_delayed_work(
				&krio_priv->pe_work,
				KEYSTONE_RIO_REGISTER_DELAY);
		} else {
			dev_err(krio_priv->dev,
				"ER port %d: failed to recover from errors\n",
				port);
		}
	}
}

/**
 * keystone_rio_port_status - Return if the port is OK or not
 * @port: index of the port
 *
 * Return %0 if the port is ready or %-EIO on failure.
 */
static int keystone_rio_port_status(int port,
				    struct keystone_rio_data *krio_priv)
{
	u32 path_mode = krio_priv->board_rio_cfg.path_mode;
	int lanes     = keystone_rio_get_lane_config(BIT(port), path_mode);
	u32 count     = 0;
	int res       = 0;
	int solid_ok  = 0;
	u32 value;

	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -EINVAL;

	/* Check port status */
	for (count = 0; count < 300; count++) {
		value = krio_sp_read(krio_priv, sp[port].err_stat);
		if (value & RIO_PORT_N_ERR_STS_PORT_OK) {
			solid_ok++;
			if (solid_ok == 100)
				break;
		} else {
			if (solid_ok) {
				dev_dbg(krio_priv->dev,
					"port %d (solid_ok = %d)\n",
					port, solid_ok);
				goto port_phy_error;
			}
			solid_ok = 0;
		}
		usleep_range(10, 50);
	}

	if (solid_ok == 100) {
		/* Sync ackID */
		res = keystone_rio_port_sync_ackid(port, krio_priv);
		if (res == -1)
			goto port_error;

		/* Check if we need to retrieve the corresponding remote port */
		if ((error_recovery) &&
		    (krio_priv->board_rio_cfg.ports_remote[port] < 0)) {
			int rport;

			rport = keystone_rio_get_remote_port(port, krio_priv);
			if (rport < 0) {
				dev_warn(krio_priv->dev,
					 "cannot retrieve remote port on port %d\n",
					 port);
			} else {
				dev_info(krio_priv->dev,
					 "detected remote port %d on port %d\n",
					 rport, port);
			}
			krio_priv->board_rio_cfg.ports_remote[port] = rport;
		}
	} else {
		dev_dbg(krio_priv->dev,
			"port %d is not initialized - port is not solid ok\n",
			port);

		goto port_error;
	}

	return 0; /* Port must be solid OK */

port_phy_error:
	dev_dbg(krio_priv->dev, "recover lane mask 0x%x for port %d\n",
		lanes, port);

	krio_priv->serdes.ops->recover_lanes(lanes, &krio_priv->serdes);
port_error:
	return -EIO;
}

/**
 * keystone_rio_port_disable - Disable a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_disable(u32 port,
				      struct keystone_rio_data *krio_priv)
{
	/* Disable port */
	krio_sp_write(krio_priv, sp[port].ctl, 0x800000);
}

/**
 * keystone_rio_port_enable - Enable a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_enable(u32 port,
				     struct keystone_rio_data *krio_priv)
{
	/* Enable port in input and output */
	krio_sp_write(krio_priv, sp[port].ctl, 0x600000);
}

/**
 * keystone_rio_port_init - Configure a RapidIO port
 * @port: index of the port to configure
 * @mode: serdes configuration
 */
static int keystone_rio_port_init(u32 port, u32 path_mode,
				  struct keystone_rio_data *krio_priv)
{
	u32 val;

	if (unlikely(port >= KEYSTONE_RIO_MAX_PORT))
		return -EINVAL;

	/* Silence and discovery timers */
	if ((port == 0) || (port == 2)) {
		krio_phy_write(krio_priv, phy_sp[port].silence_timer,
			       0x20000000);
		krio_phy_write(krio_priv, phy_sp[port].discovery_timer,
			       0x20000000);
	}

	/* Increase the number of valid code-groups required for sync */
	krio_phy_write(krio_priv, phy_sp[port].vmin_exp, 0x0f030300);

	/* Program channel allocation to ports (1x, 2x or 4x) */
	krio_phy_write(krio_priv, phy_sp[port].path_ctl, path_mode);

	/*
	 * Disable all errors reporting if using packet forwarding
	 * otherwise enable them.
	 */
	krio_err_write(krio_priv, sp_err[port].rate_en,
		       krio_priv->board_rio_cfg.pkt_forwarding ?
		       0 : 0xffffffff);

	/* Cleanup port error status */
	krio_sp_write(krio_priv, sp[port].err_stat,
		      KEYSTONE_RIO_PORT_ERROR_MASK);

	krio_err_write(krio_priv, sp_err[port].det, 0);

	/* Enable interrupt for reset request */
	val = krio_ev_read(krio_priv, evt_mgmt_rst_int_en);
	krio_ev_write(krio_priv, evt_mgmt_rst_int_en, val | BIT(port));

	/* Enable all PLM interrupts */
	krio_phy_write(krio_priv, phy_sp[port].int_enable, 0xffffffff);
	krio_phy_write(krio_priv, phy_sp[port].all_int_en, 1);

	/* Set unicast mode */
	krio_tp_write(krio_priv, transport_sp[port].control, 0x00109000);

	return 0;
}

/**
 * keystone_rio_port_set_routing - Configure routing for a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_set_routing(u32 port,
					  struct keystone_rio_data *krio_priv)
{
	u32 base_dev_id = krio_priv->board_rio_cfg.size ?
		krio_car_csr_read(krio_priv, base_dev_id) & 0xffff :
		(krio_car_csr_read(krio_priv, base_dev_id) >> 16) & 0xff;
	u32 brr = KEYSTONE_RIO_PKT_FW_BRR_NUM;

	/*
	 * Configure the Base Routing Register (BRR) to ensure that all packets
	 * matching our DevId are admitted.
	 */
	krio_tp_write(krio_priv,
		      transport_sp[port].base_route[brr].pattern_match,
		      (base_dev_id << 16)
		      | (krio_priv->board_rio_cfg.size ? 0xffff : 0xff));

	dev_dbg(krio_priv->dev, "pattern_match = 0x%x for BRR %d\n",
		krio_tp_read(krio_priv,
			     transport_sp[port].base_route[brr].pattern_match),
		brr);

	/* Enable routing to LLM for this BRR and port */
	krio_tp_write(krio_priv, transport_sp[port].base_route[brr].ctl,
		      0x84000000);

	/* Use next BRR */
	brr += 1;

	/*
	 * Configure the Base Routing Register (BRR) to ensure that all
	 * broadcast packets are admitted as well.
	 */
	krio_tp_write(krio_priv,
		      transport_sp[port].base_route[brr].pattern_match,
		      (0xffff << 16)
		      | (krio_priv->board_rio_cfg.size ? 0xffff : 0xff));

	dev_dbg(krio_priv->dev, "pattern_match = 0x%x for BRR %d\n",
		krio_tp_read(krio_priv,
			     transport_sp[port].base_route[brr].pattern_match),
		brr);

	/* Enable routing to LLM for this BRR and port */
	krio_tp_write(krio_priv, transport_sp[port].base_route[brr].ctl,
		      0x84000000);

	/* Set multicast and packet forwarding mode */
	krio_tp_write(krio_priv, transport_sp[port].control, 0x00209000);
}

/*------------------------- Configuration space mngt  ----------------------*/

/**
 * keystone_local_config_read - Generate a KeyStone local config space read
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a KeyStone local configuration space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int keystone_local_config_read(struct rio_mport *mport,
				      int index, u32 offset, int len, u32 *data)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	if (len != sizeof(u32))
		return -EINVAL; /* only 32-bit access is supported */

	if ((offset + len) > (krio_priv->board_rio_cfg.rio_regs_size
			      - KEYSTONE_RIO_CAR_CSR_REGS))
		return -EINVAL; /* only within the RIO regs range */

	/*
	 * Workaround for rionet: the processing element features must content
	 * RIO_PEF_INB_MBOX and RIO_PEF_INB_DOORBELL bits that cannot be set on
	 * KeyStone hardware. So cheat the read value in this case...
	 */
	if (unlikely(offset == RIO_PEF_CAR))
		*data = krio_priv->rio_pe_feat;
	else
		*data = krio_car_csr_read_ofs(krio_priv, offset);

	dev_dbg(krio_priv->dev,
		"local_conf_r: index %d offset 0x%x data 0x%x\n",
		index, offset, *data);

	return 0;
}

/**
 * keystone_local_config_write - Generate a KeyStone local config space write
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a KeyStone local configuration space write. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int keystone_local_config_write(struct rio_mport *mport,
				       int index, u32 offset, int len, u32 data)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	if (len != sizeof(u32))
		return -EINVAL; /* only 32-bit access is supported */

	if ((offset + len) > (krio_priv->board_rio_cfg.rio_regs_size
			      - KEYSTONE_RIO_CAR_CSR_REGS))
		return -EINVAL; /* only within the RIO regs range */

	dev_dbg(krio_priv->dev,
		"local_conf_w: index %d offset 0x%x data 0x%x\n",
		index, offset, data);

	krio_car_csr_write_ofs(krio_priv, offset, data);

	return 0;
}

/**
 * keystone_rio_config_read - Generate a KeyStone read maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a KeyStone read maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
keystone_rio_config_read(struct rio_mport *mport, int index, u16 destid,
			 u8 hopcount, u32 offset, int len, u32 *val)
{
	return keystone_rio_maint_read((struct keystone_rio_data *)mport->priv,
				       mport->index, destid, mport->sys_size,
				       hopcount, offset, len, val);
}

/**
 * keystone__rio_config_write - Generate a KeyStone write
 * maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates an KeyStone write maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
keystone_rio_config_write(struct rio_mport *mport, int index, u16 destid,
			  u8 hopcount, u32 offset, int len, u32 val)
{
	return keystone_rio_maint_write((struct keystone_rio_data *)mport->priv,
					mport->index, destid, mport->sys_size,
					hopcount, offset, len, val);
}

/*----------------------------- Port-Write management ------------------------*/

static int keystone_rio_port_write_enable(struct keystone_rio_data *krio_priv,
					  u32 port,
					  int enable)
{
	u32 val;

	/* Clear port-write reception capture */
	krio_pw_write(krio_priv, port_wr_rx_capt[port], 0);

	if (enable) {
		/*
		 * Enable generation of port-write requests
		 */
		krio_phy_write(krio_priv, phy_sp[port].port_wr_enable,
			       BIT(25) | BIT(26) | BIT(28));

		val = krio_phy_read(krio_priv, phy_sp[port].all_port_wr_en);
		krio_phy_write(krio_priv, phy_sp[port].all_port_wr_en,
			       val | BIT(0)); /* PW_EN */
	} else {
		/*
		 * Disable generation of port-write requests
		 */
		krio_phy_write(krio_priv, phy_sp[port].port_wr_enable, 0);

		val = krio_phy_read(krio_priv, phy_sp[port].all_port_wr_en);
		krio_phy_write(krio_priv, phy_sp[port].all_port_wr_en,
			       val & ~(BIT(0))); /* PW_EN */
	}

	return 0;
}

static void keystone_rio_pw_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv =
		container_of(work, struct keystone_rio_data, pw_work);
	union rio_pw_msg pwmsg;

	if (!krio_priv->started)
		return;

	/*
	 * Process port-write messages
	 */
	while (kfifo_out_spinlocked(&krio_priv->pw_fifo,
				    (unsigned char *)&pwmsg,
				    RIO_PW_MSG_SIZE,
				    &krio_priv->pw_fifo_lock)) {
		u32 port_id = pwmsg.em.is_port & 0xff;

		/* Pass the port-write message to RIO core for processing */
		if (krio_priv->mport[port_id])
			rio_inb_pwrite_handler(krio_priv->mport[port_id],
					       &pwmsg);
	}
}

/**
 *  keystone_rio_port_write_handler - KeyStone port write interrupt handler
 *
 * Handles port write interrupts. Parses a list of registered
 * port write event handlers and executes a matching event handler.
 */
static void keystone_rio_port_write_handler(struct keystone_rio_data *krio_priv)
{
	int pw;

	/* Check that we have a port-write-in case */
	pw = krio_pw_read(krio_priv, port_wr_rx_stat) & 0x1;

	/* Schedule deferred processing if PW was received */
	if (pw) {
		/*
		 * Retrieve PW message
		 */
		krio_priv->port_write_msg.msg.em.comptag =
			krio_pw_read(krio_priv, port_wr_rx_capt[0]);
		krio_priv->port_write_msg.msg.em.errdetect =
			krio_pw_read(krio_priv, port_wr_rx_capt[1]);
		krio_priv->port_write_msg.msg.em.is_port =
			krio_pw_read(krio_priv, port_wr_rx_capt[2]);
		krio_priv->port_write_msg.msg.em.ltlerrdet =
			krio_pw_read(krio_priv, port_wr_rx_capt[3]);

		/*
		 * Save PW message (if there is room in FIFO), otherwise
		 * discard it.
		 */
		if (kfifo_avail(&krio_priv->pw_fifo) >= RIO_PW_MSG_SIZE) {
			krio_priv->port_write_msg.msg_count++;
			kfifo_in(&krio_priv->pw_fifo,
				 (void const *)&krio_priv->port_write_msg.msg,
				 RIO_PW_MSG_SIZE);
		} else {
			krio_priv->port_write_msg.discard_count++;
			dev_warn(krio_priv->dev,
				 "ISR Discarded Port-Write Msg(s) (%d)\n",
				 krio_priv->port_write_msg.discard_count);
		}
		schedule_work(&krio_priv->pw_work);
	}
}

/**
 * keystone_rio_port_write_init - KeyStone port write interface init
 * @mport: Master port implementing the port write unit
 *
 * Initializes port write unit hardware and buffer
 * ring. Called from keystone_rio_setup(). Returns %0 on success
 * or %-ENOMEM on failure.
 */
static int keystone_rio_port_write_init(struct keystone_rio_data *krio_priv)
{
	int ret;
	int port;

	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		/* Disabling port write */
		keystone_rio_port_write_enable(krio_priv, port, 0);

		/* Clear port-write-in capture registers */
		krio_pw_write(krio_priv, port_wr_rx_capt[port], 0);
	}

	INIT_WORK(&krio_priv->pw_work, keystone_rio_pw_dpc);
	spin_lock_init(&krio_priv->pw_fifo_lock);

	ret = kfifo_alloc(&krio_priv->pw_fifo, RIO_PW_MSG_SIZE * 32,
			  GFP_KERNEL);
	if (ret) {
		dev_err(krio_priv->dev, "FIFO allocation failed\n");
		return -ENOMEM;
	}

	return 0;
}

/**
 * keystone_rio_pw_enable - enable/disable port-write interface init
 * @mport: Master port implementing the port write unit
 * @enable: 1=enable; 0=disable port-write message handling
 */
static int keystone_rio_pw_enable(struct rio_mport *mport, int enable)
{
	return keystone_rio_port_write_enable(mport->priv,
					      mport->index,
					      enable);
}

/*------------------------ Inbound memory region management  -----------------*/

/**
 * keystone_rio_map_inb_mem -- Mapping inbound memory region.
 * @mport: RapidIO master port
 * @lstart: Local memory space start address.
 * @rstart: RapidIO space start address.
 * @size: The mapping region size.
 * @flags: Flags for mapping. 0 for using default flags.
 *
 * Return: 0 -- Success.
 *
 * This function will create the inbound mapping
 * from rstart to lstart.
 */
static int keystone_rio_map_inb_mem(struct rio_mport *mport, dma_addr_t lstart,
				    u64 rstart, u32 size, u32 flags)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	dev_dbg(krio_priv->dev,
		"mapping inbound window 0x%x to RIO space 0x%llx with size 0x%x\n",
		lstart, rstart, size);

	/*
	 * Because we do not hw capability to map inbound mapping we need to
	 * ensure that caller is asking us for a 1:1 direct mapping.
	 */
	if ((dma_addr_t)rstart != lstart)
		return -EINVAL;

	return 0;
}

/**
 * keystone_rio_unmap_inb_mem -- Unmapping inbound memory region.
 * @mport: RapidIO master port
 * @lstart: Local memory space start address.
 */
static void keystone_rio_unmap_inb_mem(struct rio_mport *mport,
				       dma_addr_t lstart)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	dev_dbg(krio_priv->dev, "unmapping inbound window 0x%x\n", lstart);
}

/*------------------------ Main Linux driver functions -----------------------*/

static int keystone_rio_query_mport(struct rio_mport *mport,
				    struct rio_mport_attr *attr)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	u32 port = mport->index;
	u32 rval;

	if (!attr)
		return -EINVAL;

	rval = krio_sp_read(krio_priv, sp[port].err_stat);
	if (rval & RIO_PORT_N_ERR_STS_PORT_OK) {
		rval = krio_sp_read(krio_priv, sp[port].ctl2);
		attr->link_speed = (rval & RIO_PORT_N_CTL2_SEL_BAUD) >> 28;
		rval = krio_sp_read(krio_priv, sp[port].ctl);
		attr->link_width = (rval & RIO_PORT_N_CTL_IPW) >> 27;
	} else {
		attr->link_speed = RIO_LINK_DOWN;
	}

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	/* Supporting DMA but not HW SG mode*/
	attr->flags        = RIO_MPORT_DMA;

	attr->dma_max_sge  = KEYSTONE_RIO_DMA_MAX_DESC - 1;
	attr->dma_max_size = KEYSTONE_RIO_MAX_DIO_PKT_SIZE;
	attr->dma_align    = KEYSTONE_RIO_DIO_ALIGNMENT;
#else
	attr->flags        = 0;
	attr->dma_max_sge  = 0;
	attr->dma_max_size = 0;
	attr->dma_align    = 0;
#endif

	return 0;
}

static void keystone_rio_mport_release(struct device *dev)
{
	struct rio_mport *mport = to_rio_mport(dev);

	dev_dbg(dev, "%s %s id=%d\n", __func__, mport->name, mport->id);
}

struct rio_mport *keystone_rio_register_mport(
	u32 port_id,
	u32 size,
	struct keystone_rio_data *krio_priv)
{
	struct rio_ops   *ops;
	struct rio_mport *mport;
	int res;

	ops = kzalloc(sizeof(*ops), GFP_KERNEL);

	ops->lcread  = keystone_local_config_read;
	ops->lcwrite = keystone_local_config_write;
	ops->cread   = keystone_rio_config_read;
	ops->cwrite  = keystone_rio_config_write;
	ops->dsend   = keystone_rio_dbell_send;
	ops->open_outb_mbox   = keystone_rio_open_outb_mbox;
	ops->close_outb_mbox  = keystone_rio_close_outb_mbox;
	ops->open_inb_mbox    = keystone_rio_open_inb_mbox;
	ops->close_inb_mbox   = keystone_rio_close_inb_mbox;
	ops->add_outb_message = keystone_rio_hw_add_outb_message;
	ops->add_inb_buffer   = keystone_rio_hw_add_inb_buffer;
	ops->get_inb_message  = keystone_rio_hw_get_inb_message;
	ops->query_mport      = keystone_rio_query_mport;
	ops->map_inb	      = keystone_rio_map_inb_mem;
	ops->unmap_inb	      = keystone_rio_unmap_inb_mem;
	ops->pwenable	      = keystone_rio_pw_enable;

	mport = kzalloc(sizeof(*mport), GFP_KERNEL);

	/* Initialize the mport structure */
	res = rio_mport_initialize(mport);
	if (res) {
		kfree(mport);
		return NULL;
	}

	/*
	 * Set the SRIO port physical Id into the index field,
	 * the id field has been set by rio_mport_initialize() to
	 * the logical Id
	 */
	mport->index = port_id;
	mport->priv  = krio_priv;
	mport->dev.parent  = krio_priv->dev;
	mport->dev.release = keystone_rio_mport_release;
	INIT_LIST_HEAD(&mport->dbells);
	INIT_LIST_HEAD(&mport->pwrites);

	/*
	 * Make a dummy per port region as ports are not
	 * really separated on KeyStone
	 */
	mport->iores.start = (u32)(krio_priv->serial_port_regs) +
		offsetof(struct keystone_rio_serial_port_regs,
			 sp[port_id].link_maint_req);

	mport->iores.end = (u32)(krio_priv->serial_port_regs) +
		offsetof(struct keystone_rio_serial_port_regs,
			 sp[port_id].ctl);

	mport->iores.flags = IORESOURCE_MEM;

	rio_init_dbell_res(&mport->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	rio_init_mbox_res(&mport->riores[RIO_INB_MBOX_RESOURCE], 0,
			  KEYSTONE_RIO_MAX_MBOX);
	rio_init_mbox_res(&mport->riores[RIO_OUTB_MBOX_RESOURCE], 0,
			  KEYSTONE_RIO_MAX_MBOX);

	sprintf(mport->name, "RIO%d mport", port_id);

	mport->ops      = ops;
	mport->sys_size = size;
	mport->phy_type = RIO_PHY_SERIAL;

	/*
	 * Hard coded here because in rio_disc_mport(), it is used in
	 * rio_enum_complete() before it is retrieved in
	 * rio_disc_peer() => rio_setup_device()
	 */
	mport->phys_efptr = 0x100;

	/*
	 * Register the new mport
	 */
	res = rio_register_mport(mport);
	if (res) {
		kfree(mport);
		return NULL;
	}

	krio_priv->mport[port_id] = mport;

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	/*
	 * Register the DMA engine for DirectIO transfers
	 */
	keystone_rio_dma_register(mport,
				  krio_priv->board_rio_cfg.dma_channel_num);
	/*
	 * Reserve one channel for doorbells
	 */
	keystone_rio_lsu_dma_allocate_channel(mport);
#endif

	return mport;
}

static int krio_of_parse_mbox(int mbox, struct device_node *node_rio,
			      struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan =
					&krio_priv->rx_channels[mbox];
	struct keystone_rio_tx_chan_info *ktx_chan =
					&krio_priv->tx_channels[mbox];
	struct device_node *node;
	char node_name[24];
	u32 temp[2];

	snprintf(node_name, sizeof(node_name), "mbox-%d", mbox);
	node = of_get_child_by_name(node_rio, node_name);
	if (!node) {
		dev_err(krio_priv->dev, "could not find %s node\n", node_name);
		return -ENODEV;
	}

	dev_dbg(krio_priv->dev, "using node \"%s\"\n", node_name);

	/* DMA rx chan config */
	if (of_property_read_string(node, "rx-channel", &krx_chan->name) < 0) {
		dev_err(krio_priv->dev,
			"missing \"rx-channel\" parameter for mbox %d\n",
			mbox);
		of_node_put(node);
		return -ENOENT;
	}

	if (of_property_read_u32_array(node, "rx-pool", temp, 2)) {
		dev_err(krio_priv->dev,
			"missing \"rx-pool\" parameter for mbox %d\n",
			mbox);
		of_node_put(node);
		return -ENOENT;
	}

	krx_chan->pool_size = temp[0];
	krx_chan->pool_region_id = temp[1];

	if (of_property_read_u32_array(node, "rx-queue-depth",
				       krx_chan->queue_depths,
				       KNAV_DMA_FDQ_PER_CHAN) < 0) {
		dev_warn(krio_priv->dev,
			 "missing \"rx-queue-depth\" parameter for mbox %d\n",
			 mbox);
		krx_chan->queue_depths[0] = 128;
	}

	if (of_property_read_u32_array(node, "rx-buffer-size",
				       krx_chan->buffer_sizes,
				       KNAV_DMA_FDQ_PER_CHAN) < 0) {
		dev_warn(krio_priv->dev,
			 "missing \"rx-buffer-size\" parameter for mbox %d\n",
			 mbox);
		krx_chan->buffer_sizes[0] = 4096;
	}

	if (of_property_read_u32(node, "rx-queue",
				 &krx_chan->queue_id)) {
		dev_warn(krio_priv->dev, "missing \"rx-queue\" parameter for mbox %d, using qpend\n",
			 mbox);
		krx_chan->queue_id = KNAV_QUEUE_QPEND;
	}

	/*
	 * If stream_id is defined, this mbox is mapped to the corresponding
	 * streamid and the channel is for type 9 packets.
	 */
	if (of_property_read_u32(node, "stream-id",
				 &krx_chan->stream_id) < 0) {
		krx_chan->packet_type = RIO_PACKET_TYPE_MESSAGE;
		krx_chan->stream_id = -1;
	} else {
		krx_chan->packet_type = RIO_PACKET_TYPE_STREAM;
	}

	/* DMA tx chan config */
	if (of_property_read_string(node, "tx-channel", &ktx_chan->name) < 0) {
		dev_err(krio_priv->dev, "missing \"tx-channel\" parameter for mbox %d\n",
			mbox);
		of_node_put(node);
		return -ENOENT;
	}

	if (of_property_read_u32_array(node, "tx-pool", temp, 2)) {
		dev_err(krio_priv->dev, "missing \"tx-pool\" parameter for mbox %d\n",
			mbox);
		of_node_put(node);
		return -ENOENT;
	}

	ktx_chan->pool_size = temp[0];
	ktx_chan->pool_region_id = temp[1];

	if (of_property_read_u32(node, "tx-queue-depth",
				 &ktx_chan->queue_depth) < 0) {
		dev_warn(krio_priv->dev,
			 "missing \"tx-queue-depth\" parameter for mbox %d\n",
			 mbox);
		ktx_chan->queue_depth = 128;
	}

	if (of_property_read_u32(node, "tx-queue",
				 &ktx_chan->queue_id)) {
		dev_err(krio_priv->dev,
			"missing \"tx-queue\" parameter for mbox %d\n",
			mbox);
		of_node_put(node);
		return -ENOENT;
	}

	if (of_property_read_u32(node, "tx-completion-queue",
				 &ktx_chan->complet_queue_id)) {
		dev_warn(krio_priv->dev,
			 "missing \"tx-completion-queue\" parameter for mbox %d, using qpend\n",
			 mbox);
		ktx_chan->complet_queue_id = KNAV_QUEUE_QPEND;
	}

	if (of_property_read_u32(node, "tx-garbage-queue",
				 &ktx_chan->garbage_queue_id)) {
		dev_warn(krio_priv->dev,
			 "missing \"tx-garbage-queue\" parameter for mbox %d, using qpend\n",
			 mbox);
		ktx_chan->garbage_queue_id = KNAV_QUEUE_QPEND;
	}

	of_node_put(node);

	return 0;
}

static int krio_of_parse(struct device_node *node,
			 struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_board_controller_info *c =
		&krio_priv->board_rio_cfg;
	u32 temp[24];
	int i;
	int mbox;

	/* Get SRIO registers */
	i = of_property_match_string(node, "reg-names", "rio");
	if (i < 0) {
		dev_err(krio_priv->dev,
			"missing reg-names \"boot_config\" parameter\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1), &temp[0])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1) + 1, &temp[1])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}

	c->rio_regs_base = temp[0];
	c->rio_regs_size = temp[1];

	/* Get boot config registers */
	i = of_property_match_string(node, "reg-names", "boot_config");
	if (i < 0) {
		dev_err(krio_priv->dev,
			"missing reg-names \"boot_config\" parameter\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1), &temp[2])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1) + 1, &temp[3])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}

	c->boot_cfg_regs_base = temp[2];
	c->boot_cfg_regs_size = temp[3];

	/* Get SerDes registers */
	i = of_property_match_string(node, "reg-names", "serdes");
	if (i < 0) {
		dev_err(krio_priv->dev,
			"missing reg-names \"serdes\" parameter\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1), &temp[4])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}
	if (of_property_read_u32_index(node, "reg", (i << 1) + 1, &temp[5])) {
		dev_err(krio_priv->dev, "missing \"reg\" parameters\n");
		return -ENOENT;
	}

	c->serdes_cfg_regs_base = temp[4];
	c->serdes_cfg_regs_size = temp[5];

	if (of_property_read_u32 (node, "dev-id-size", &c->size))
		dev_warn(krio_priv->dev, "missing \"dev-id-size\" parameter\n");

	if (of_property_read_u32 (node, "ports", &c->ports))
		dev_warn(krio_priv->dev, "missing \"ports\" parameter\n");

	if (of_property_read_u32_array(node, "ports-remote", c->ports_remote,
				       KEYSTONE_RIO_MAX_PORT)) {
		/* Remote ports will be detected during port status */
		for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++)
			c->ports_remote[i] = -1;
	}

	/* SerDes config */
	if (!of_find_property(node, "keystone2-serdes", NULL)) {
		/* K1 setup*/
		c->serdes_type                     = KEYSTONE_SERDES_TYPE_K1;
		c->serdes_config.prescalar_srv_clk = 0x001e;
		c->serdes_config.do_phy_init_cfg   = 0;
		c->path_mode                       = 0x0000;
	} else {
		/* K2 setup*/
		c->serdes_type                     = KEYSTONE_SERDES_TYPE_K2;
		c->serdes_config.prescalar_srv_clk = 0x001f;
		c->serdes_config.do_phy_init_cfg   = 0;
		c->path_mode                       = 0x0004;

		if (of_property_read_u32(node, "baudrate",
					 &c->serdes_baudrate)) {
			dev_warn(krio_priv->dev,
				 "missing \"baudrate\" parameter, using 5Gbps\n");
			c->serdes_baudrate = KEYSTONE_SERDES_BAUD_5_000;
		}
	}

	/* Set if performing optional SerDes calibration sequence at boot */
	c->serdes_calibration = serdes_calibration;

	/* SerDes pre-1lsb, c1, c2, cm, att and vreg config */
	if (of_property_read_u32_array(node, "serdes-1lsb", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].pre_1lsb = 0;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].pre_1lsb = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes-c1", &temp[0], 4)) {
		if (c->serdes_baudrate == KEYSTONE_SERDES_BAUD_3_125) {
			for (i = 0; i < 4; i++)
				c->serdes_config.tx[i].c1_coeff = 4;
		} else {
			for (i = 0; i < 4; i++)
				c->serdes_config.tx[i].c1_coeff = 6;
		}
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].c1_coeff = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes-c2", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].c2_coeff = 0;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].c2_coeff = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes-cm", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].cm_coeff = 0;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].cm_coeff = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes-att", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].att = 12;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].att = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes-vreg", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].vreg = 4;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].vreg = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes-vdreg", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].vdreg = 1;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.tx[i].vdreg = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes-rx-att-start",
				       &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].start_att = 3;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].start_att = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes-rx-boost-start",
				       &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].start_boost = 3;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].start_boost = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes-rx-att", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			/* Use dynamic Rx calibration */
			c->serdes_config.rx[i].mean_att = -1;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].mean_att = temp[i];
	}

	if (of_property_read_u32_array(node, "serdes-rx-boost", &temp[0], 4)) {
		for (i = 0; i < 4; i++)
			/* Use dynamic Rx calibration */
			c->serdes_config.rx[i].mean_boost = -1;
	} else {
		for (i = 0; i < 4; i++)
			c->serdes_config.rx[i].mean_boost = temp[i];
	}

	/* Path mode config (mapping of SerDes lanes to port widths) */
	if (of_property_read_u32(node, "path-mode", &c->path_mode)) {
		dev_warn(krio_priv->dev,
			 "missing \"path-mode\" parameter\n");
	}

	/* Max possible ports configurations per path_mode */
	if ((c->path_mode == 0 &&
	     c->ports & ~KEYSTONE_RIO_MAX_PORTS_PATH_MODE_0) ||
	    (c->path_mode == 1 &&
	     c->ports & ~KEYSTONE_RIO_MAX_PORTS_PATH_MODE_1) ||
	    (c->path_mode == 2 &&
	     c->ports & ~KEYSTONE_RIO_MAX_PORTS_PATH_MODE_2) ||
	    (c->path_mode == 3 &&
	     c->ports & ~KEYSTONE_RIO_MAX_PORTS_PATH_MODE_3) ||
	    (c->path_mode == 4 &&
	     c->ports & ~KEYSTONE_RIO_MAX_PORTS_PATH_MODE_4)) {
		dev_err(krio_priv->dev,
			"\"path_mode\" and \"ports\" configuration mismatch\n");
		return -EINVAL;
	}

	/* Port register timeout */
	if (of_property_read_u32(node, "port-register-timeout",
				 &c->port_register_timeout)) {
		c->port_register_timeout = 30;
	}

	/* LSUs */
	if (of_property_read_u32_array(node, "lsu", &temp[0], 2)) {
		krio_priv->lsu_start = 0;
		krio_priv->lsu_end   = 0;
	} else {
		krio_priv->lsu_start = (u8)temp[0];
		krio_priv->lsu_end   = (u8)temp[1];
	}

	dev_dbg(krio_priv->dev, "using LSU %d - %d range\n",
		krio_priv->lsu_start, krio_priv->lsu_end);

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	/* DIO (virtual) DMA channels */
	if (of_property_read_u32(node, "num-dio-channels",
				 &c->dma_channel_num) < 0) {
		dev_warn(krio_priv->dev,
			 "missing \"num-dio-channels\" parameter\n");
		c->dma_channel_num = 8;
	}
#endif

	/* RXU mapping resources */
	if (of_property_read_u32_array(node, "rxu-map-range", &temp[0], 2)) {
		krio_priv->rxu_map_start = KEYSTONE_RIO_RXU_MAP_MIN;
		krio_priv->rxu_map_end   = KEYSTONE_RIO_RXU_MAP_MAX;
	} else {
		if ((temp[1] > KEYSTONE_RIO_RXU_MAP_MAX) ||
		    (temp[0] > temp[1])) {
			dev_err(krio_priv->dev,
				"invalid \"rxu-map-range\" parameter\n");
			return -EINVAL;
		}

		krio_priv->rxu_map_start = temp[0];
		krio_priv->rxu_map_end   = temp[1];
	}

	dev_dbg(krio_priv->dev, "using RXU map %lu - %lu range\n",
		krio_priv->rxu_map_start, krio_priv->rxu_map_end);

	/* Mailboxes configuration */
	if (of_property_read_u32(node, "num-mboxes",
				 &krio_priv->num_mboxes) < 0) {
		dev_warn(krio_priv->dev,
			 "missing \"num-mboxes\" parameter\n");
		krio_priv->num_mboxes = 1;
	}

	if (krio_priv->num_mboxes > KEYSTONE_RIO_MAX_MBOX) {
		dev_warn(krio_priv->dev,
			 "wrong \"num_mboxes\" parameter value %d, set to %d\n",
			 krio_priv->num_mboxes, KEYSTONE_RIO_MAX_MBOX);
		krio_priv->num_mboxes = KEYSTONE_RIO_MAX_MBOX;
	}

	/* Retrieve the per-mailboxes properties */
	for (mbox = 0; mbox < krio_priv->num_mboxes; mbox++) {
		int res;

		res = krio_of_parse_mbox(mbox, node, krio_priv);
		if (res)
			return res;
	}

	/* Interrupt config */
	c->rio_irq = irq_of_parse_and_map(node, 0);
	if (c->rio_irq < 0) {
		dev_err(krio_priv->dev, "missing \"rio_irq\" parameter\n");
		return -ENOENT;
	}

	c->lsu_irq = irq_of_parse_and_map(node, 1);
	if (c->lsu_irq < 0) {
		dev_err(krio_priv->dev, "missing \"lsu_irq\" parameter\n");
		return -ENOENT;
	}

	/* Packet forwarding */
	if (of_property_read_u32_array(node, "pkt-forward", &temp[0], 24)) {
		c->pkt_forwarding = 0;
	} else {
		c->pkt_forwarding = 1;

		for (i = 0; i < 8; i++) {
			c->routing_config[i].dev_id_low = (u16)temp[(i * 3)];
			c->routing_config[i].dev_id_high =
						(u16)temp[(i * 3) + 1];
			c->routing_config[i].port = (u8)temp[(i * 3) + 2];
		}
	}

	return 0;
}

static int keystone_rio_port_chk(struct keystone_rio_data *krio_priv, int init)
{
	unsigned long flags;
	u32 ports;
	u32 size  = krio_priv->board_rio_cfg.size;
	struct rio_mport *mport;

	/* first check those which are scan registering */
	ports = krio_priv->ports_scan_registering;
	while (ports) {
		u32 port = __ffs(ports);

		ports &= ~BIT(port);

		if (!krio_priv->mport[port] || !krio_priv->mport[port]->nscan)
			continue;

		spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
		krio_priv->ports_scan_registering &= ~BIT(port);
		spin_unlock_irqrestore(&krio_priv->port_chk_lock, flags);

		/* now finish the remaining steps */
		if (krio_priv->board_rio_cfg.pkt_forwarding)
			keystone_rio_port_set_routing(port, krio_priv);

		krio_priv->base_dev_id = krio_car_csr_read(krio_priv,
							   base_dev_id);
	}

	/* next check those which are registering */
	ports = krio_priv->ports_registering;
	while (ports) {
		int status;
		u32 port = __ffs(ports);

		ports &= ~BIT(port);

		/* Eventually start lanes and wait them to be OK and with SD */
		if (keystone_rio_lanes_init_and_wait(port, init, krio_priv))
			continue;

		/*
		 * Check the port status here before calling the generic RapidIO
		 * layer. Port status check is done in rio_mport_is_active() as
		 * well but we need to do it our way first due to some delays in
		 * hw initialization.
		 */
		status = keystone_rio_port_status(port, krio_priv);
		if (status == 0) {
			unsigned long flags;

			/*
			 * The link has been established from an hw standpoint
			 * so do not try to check the port again.
			 * Only mport registration may fail now.
			 */
			spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
			krio_priv->ports |= BIT(port);
			krio_priv->ports_registering &= ~BIT(port);
			spin_unlock_irqrestore(&krio_priv->port_chk_lock,
					       flags);

			/* Register mport only if this is initial port check */
			if (!krio_priv->mport[port]) {
				mport = keystone_rio_register_mport(
					port, size, krio_priv);

				if (!mport) {
					dev_err(krio_priv->dev,
						"failed to register mport %d\n",
						port);
					return -1;
				} else if (!mport->nscan) {
					dev_info(krio_priv->dev,
						 "mport RIO%d@%p hdid %d registering scan\n",
						 port, mport,
						 mport->host_deviceid);
					spin_lock_irqsave(
						&krio_priv->port_chk_lock,
						flags);
					krio_priv->ports_scan_registering |=
						BIT(port);
					spin_unlock_irqrestore(
						&krio_priv->port_chk_lock,
						flags);
					continue;
				} else {
					dev_info(krio_priv->dev,
						 "port RIO%d host_deviceid %d registered\n",
						 port, mport->host_deviceid);
				}
			} else {
				/* should not happen */
				dev_info(krio_priv->dev,
					 "port RIO%d host_deviceid %d ready\n",
					 port,
					 krio_priv->mport[port]->host_deviceid);
			}

			/*
			 * Update routing after discovery/enumeration
			 * with new dev id
			 */
			if (krio_priv->board_rio_cfg.pkt_forwarding)
				keystone_rio_port_set_routing(port, krio_priv);

			/* Save the current base dev Id */
			krio_priv->base_dev_id = krio_car_csr_read(krio_priv,
								   base_dev_id);
		} else {
			if (status == -EINVAL)
				return -1;

			dev_dbg(krio_priv->dev, "port %d not ready\n", port);
		}
	}

#ifdef CONFIG_RAPIDIO_ENUM_BASIC
	if (!krio_priv->ports_scan_registering &&
	    !krio_priv->ports_registering) {
		rio_init_mports();
	}
#endif

	return (krio_priv->ports_registering |
		krio_priv->ports_scan_registering);
}

static void keystone_rio_port_chk_task(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv =
			container_of(to_delayed_work(work),
				     struct keystone_rio_data,
				     port_chk_task);
	int res;

	res = keystone_rio_port_chk(krio_priv, 0);
	if (res) {
		unsigned long flags;

		if (res == -1)
			return;

		/* If port check failed schedule next check (if any) */
		spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
		if (krio_priv->port_chk_cnt-- > 1) {
			spin_unlock_irqrestore(&krio_priv->port_chk_lock,
					       flags);

			schedule_delayed_work(&krio_priv->port_chk_task,
					      KEYSTONE_RIO_REGISTER_DELAY);
		} else {
			spin_unlock_irqrestore(&krio_priv->port_chk_lock,
					       flags);

			dev_info(krio_priv->dev,
				 "RIO port register timeout, port mask 0x%x not ready",
				 krio_priv->ports_registering |
				 krio_priv->ports_scan_registering);
		}
	}
}

/*
 * Sysfs management
 */
static ssize_t keystone_rio_ports_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)
		dev->platform_data;
	unsigned long ports;
	unsigned long flags;

	if (kstrtoul(buf, 0, &ports))
		return -EINVAL;

	if (ports > (BIT(KEYSTONE_RIO_MAX_PORT) - 1))
		return -EINVAL;

	/*
	 * Only the ports defined in DTS and don't have a good hw port
	 * status yet can be rescanned because SerDes initialization
	 * is not restarted here, only link status check.
	 */
	ports &= krio_priv->board_rio_cfg.ports;

	spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
	krio_priv->ports_registering = (ports & ~krio_priv->ports);
	krio_priv->ports_scan_registering = 0;
	spin_unlock_irqrestore(&krio_priv->port_chk_lock, flags);

	if (krio_priv->ports_registering) {
		unsigned long flags;

		dev_dbg(dev, "initializing link for port mask 0x%x\n",
			krio_priv->ports_registering);

		spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
		krio_priv->port_chk_cnt =
			krio_priv->board_rio_cfg.port_register_timeout /
			(KEYSTONE_RIO_REGISTER_DELAY / HZ);
		spin_unlock_irqrestore(&krio_priv->port_chk_lock, flags);

		schedule_delayed_work(&krio_priv->port_chk_task, 0);
	} else {
		dev_info(dev, "No port to be reinitialized: port mask = 0x%x\n",
			 krio_priv->ports_registering);
	}

	return count;
}

static ssize_t keystone_rio_ports_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct keystone_rio_data *krio_priv =
			(struct keystone_rio_data *)dev->platform_data;

	if (!krio_priv)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "0x%x\n", krio_priv->ports);
}

static ssize_t keystone_rio_start_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)
		dev->platform_data;
	unsigned long new_start;

	if (kstrtoul(buf, 0, &new_start))
		return -EINVAL;

	/* Start SRIO peripheral if not started */
	if ((new_start) && (krio_priv->started == 0)) {
		keystone_rio_setup_controller(krio_priv);
		return count;
	}

	/* Stop SRIO peripheral if started */
	if ((new_start == 0) && (krio_priv->started == 1)) {
		keystone_rio_shutdown_controller(krio_priv);
		return count;
	}

	return count;
}

static ssize_t keystone_rio_start_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct keystone_rio_data *krio_priv =
			(struct keystone_rio_data *)dev->platform_data;

	if (!krio_priv)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%d\n", krio_priv->started);
}

static ssize_t keystone_rio_calibrate_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf,
					    size_t count)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)
		dev->platform_data;
	unsigned long new_calibrate;

	if (kstrtoul(buf, 0, &new_calibrate))
		return -EINVAL;

	/* Start SRIO calibration */
	if (new_calibrate && !krio_priv->started) {
		int res;
		u32 block;

		/* Enable RIO SerDes blocks */
		krio_write(krio_priv, gbl_en, 1);
		for (block = KEYSTONE_RIO_BLK_PORT0_ID;
		     block <= KEYSTONE_RIO_BLK_PORT3_ID; block++)
			krio_write(krio_priv, blk[block].enable, 1);

		/* Do SerDes initialization and calibration */
		res = keystone_rio_serdes_init(
			krio_priv->board_rio_cfg.serdes_baudrate,
			1,
			krio_priv);

		if (res < 0)
			dev_err(krio_priv->dev,
				"calibration of SerDes failed\n");
	}

	return count;
}

static ssize_t keystone_rio_calibrate_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct keystone_rio_data *krio_priv =
			(struct keystone_rio_data *)dev->platform_data;

	if (!krio_priv)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%d\n", krio_priv->calibrating);
}

static DEVICE_ATTR(ports,
		   S_IRUGO | S_IWUSR,
		   keystone_rio_ports_show,
		   keystone_rio_ports_store);

static DEVICE_ATTR(start,
		   S_IRUGO | S_IWUSR,
		   keystone_rio_start_show,
		   keystone_rio_start_store);

static DEVICE_ATTR(calibrate,
		   S_IRUGO | S_IWUSR,
		   keystone_rio_calibrate_show,
		   keystone_rio_calibrate_store);

static void keystone_rio_sysfs_remove(struct device *dev)
{
	device_remove_file(dev, &dev_attr_ports);
	device_remove_file(dev, &dev_attr_start);
	device_remove_file(dev, &dev_attr_calibrate);
}

static int keystone_rio_sysfs_create(struct device *dev)
{
	int res = 0;

	res = device_create_file(dev, &dev_attr_ports);
	if (res) {
		dev_err(dev, "unable create sysfs ports file\n");
		return res;
	}

	res = device_create_file(dev, &dev_attr_start);
	if (res) {
		dev_err(dev, "unable create sysfs start file\n");
		return res;
	}

	res = device_create_file(dev, &dev_attr_calibrate);
	if (res)
		dev_err(dev, "unable create sysfs calibrate file\n");

	return res;
}

/*
 * Platform configuration setup
 */
static int keystone_rio_setup_controller(struct keystone_rio_data *krio_priv)
{
	u32 ports;
	u32 p;
	u32 baud;
	u32 path_mode;
	u32 size = 0;
	int res = 0;
	char str[8];
	unsigned long flags;

	size      = krio_priv->board_rio_cfg.size;
	ports     = krio_priv->board_rio_cfg.ports;
	baud      = krio_priv->board_rio_cfg.serdes_baudrate;
	path_mode = krio_priv->board_rio_cfg.path_mode;

	krio_priv->started = 1;

	dev_dbg(krio_priv->dev, "size = %d, ports = 0x%x, baud = %d, path_mode = %d\n",
		size, ports, baud, path_mode);

	if (baud > KEYSTONE_SERDES_BAUD_5_000) {
		baud = KEYSTONE_SERDES_BAUD_5_000;
		dev_warn(krio_priv->dev,
			 "invalid baud rate, forcing it to 5Gbps\n");
	}

	switch (baud) {
	case KEYSTONE_SERDES_BAUD_1_250:
		snprintf(str, sizeof(str), "1.25");
		break;
	case KEYSTONE_SERDES_BAUD_2_500:
		snprintf(str, sizeof(str), "2.50");
		break;
	case KEYSTONE_SERDES_BAUD_3_125:
		snprintf(str, sizeof(str), "3.125");
		break;
	case KEYSTONE_SERDES_BAUD_5_000:
		snprintf(str, sizeof(str), "5.00");
		break;
	default:
		return -EINVAL;
	}

	dev_info(krio_priv->dev,
		 "initializing %s Gbps interface with port configuration %d\n",
		 str, path_mode);

	/* Hardware set up of the controller */
	res = keystone_rio_hw_init(baud, krio_priv);
	if (res < 0) {
		dev_err(krio_priv->dev,
			"initialization of SRIO hardware failed\n");
		return res;
	}

	/* Initialize port write interface */
	res = keystone_rio_port_write_init(krio_priv);
	if (res)
		return res;

	/* Disable all ports */
	for (p = 0; p < KEYSTONE_RIO_MAX_PORT; p++)
		keystone_rio_port_disable(p, krio_priv);

	/* Register all configured ports */
	krio_priv->ports_registering = krio_priv->board_rio_cfg.ports;
	krio_priv->ports_scan_registering = 0;

	/* Initialize interrupts */
	res = keystone_rio_interrupt_setup(krio_priv);
	if (res)
		return res;

	/* Start the controller */
	keystone_rio_start(krio_priv);

	while (ports) {
		u32 port = __ffs(ports);

		ports &= ~BIT(port);

		res = keystone_rio_port_init(port, path_mode, krio_priv);
		if (res < 0) {
			dev_err(krio_priv->dev,
				"initialization of port %d failed\n", port);
			return res;
		}

		/* Start the port */
		keystone_rio_port_enable(port, krio_priv);
	}

	/* Complete port initialization and wait link */
	res = keystone_rio_port_chk(krio_priv, 1);
	if (res) {
		if (res == -1)
			return -ENOMEM;

		/* If port check failed schedule asynchronous periodic check */
		spin_lock_irqsave(&krio_priv->port_chk_lock, flags);
		krio_priv->port_chk_cnt =
			krio_priv->board_rio_cfg.port_register_timeout /
			(KEYSTONE_RIO_REGISTER_DELAY / HZ);
		spin_unlock_irqrestore(&krio_priv->port_chk_lock, flags);

		schedule_delayed_work(&krio_priv->port_chk_task,
				      KEYSTONE_RIO_REGISTER_DELAY);
	}

	return res;
}

static int keystone_rio_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct keystone_rio_data *krio_priv;
	int r;
	u16 serdes_type;
	void __iomem *regs;

	dev_info(&pdev->dev, "KeyStone RapidIO driver %s\n", DRIVER_VER);

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}

	krio_priv = devm_kzalloc(&pdev->dev,
				 sizeof(struct keystone_rio_data),
				 GFP_KERNEL);
	if (!krio_priv) {
		dev_err(&pdev->dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, krio_priv);
	krio_priv->dev = &pdev->dev;

	/* Get default config from device tree */
	r = krio_of_parse(node, krio_priv);
	if (r < 0) {
		dev_err(&pdev->dev, "failed to get configuration\n");
		return r;
	}

	serdes_type = krio_priv->board_rio_cfg.serdes_type;

	/* SRIO main driver (global resources) */
	krio_priv->lsu_free = krio_priv->lsu_start;
	krio_priv->lsu_maint = keystone_rio_lsu_alloc(krio_priv);

	mutex_init(&krio_priv->lsu_lock_maint);

	spin_lock_init(&krio_priv->port_chk_lock);

	INIT_DELAYED_WORK(&krio_priv->port_chk_task,
			  keystone_rio_port_chk_task);
	INIT_DELAYED_WORK(&krio_priv->pe_work, keystone_rio_pe_dpc);
	INIT_WORK(&krio_priv->reset_work, keystone_rio_reset_dpc);

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	for (r = 0; r < KEYSTONE_RIO_LSU_NUM; r++)
		INIT_LIST_HEAD(&krio_priv->dma_channels[r]);
#endif

	/* Initial base dev Id */
	krio_priv->base_dev_id = 0x00ffffff;

	regs = ioremap(krio_priv->board_rio_cfg.boot_cfg_regs_base,
		       krio_priv->board_rio_cfg.boot_cfg_regs_size);

	krio_priv->jtagid_reg = regs + 0x0018;

	if (serdes_type == KEYSTONE_SERDES_TYPE_K1)
		krio_priv->serdes_sts_reg = regs + 0x154;

	regs = ioremap(krio_priv->board_rio_cfg.serdes_cfg_regs_base,
		       krio_priv->board_rio_cfg.serdes_cfg_regs_size);
	krio_priv->serdes_regs = regs;

	regs = ioremap(krio_priv->board_rio_cfg.rio_regs_base,
		       krio_priv->board_rio_cfg.rio_regs_size);
	krio_priv->regs		     = regs;
	krio_priv->car_csr_regs	     = regs + KEYSTONE_RIO_CAR_CSR_REGS;
	krio_priv->serial_port_regs  = regs + KEYSTONE_RIO_SERIAL_PORT_REGS;
	krio_priv->err_mgmt_regs     = regs + KEYSTONE_RIO_ERR_MGMT_REGS;
	krio_priv->phy_regs	     = regs + KEYSTONE_RIO_PHY_REGS;
	krio_priv->transport_regs    = regs + KEYSTONE_RIO_TRANSPORT_REGS;
	krio_priv->pkt_buf_regs	     = regs + KEYSTONE_RIO_PKT_BUF_REGS;
	krio_priv->evt_mgmt_regs     = regs + KEYSTONE_RIO_EVT_MGMT_REGS;
	krio_priv->port_write_regs   = regs + KEYSTONE_RIO_PORT_WRITE_REGS;
	krio_priv->link_regs	     = regs + KEYSTONE_RIO_LINK_REGS;
	krio_priv->fabric_regs	     = regs + KEYSTONE_RIO_FABRIC_REGS;

	/* Register SerDes */
	r = keystone_rio_serdes_register(
		serdes_type,
		krio_priv->serdes_regs,
		krio_priv->serdes_sts_reg,
		&pdev->dev,
		&krio_priv->serdes,
		&krio_priv->board_rio_cfg.serdes_config);

	if (r < 0) {
		dev_err(&pdev->dev, "cannot register SerDes type %d\n",
			serdes_type);
		return -EINVAL;
	}

	dev_info(&pdev->dev, "using K%d SerDes\n",
		 (serdes_type == KEYSTONE_SERDES_TYPE_K2) ? 2 : 1);

	/* Enable SRIO clock */
	krio_priv->clk = clk_get(&pdev->dev, "clk_srio");
	if (IS_ERR(krio_priv->clk)) {
		dev_err(&pdev->dev, "Unable to get Keystone SRIO clock\n");
		return -EBUSY;
	}

	/* Workaround for K1 SRIO clocks */
	clk_prepare_enable(krio_priv->clk);
	ndelay(100);
	clk_disable_unprepare(krio_priv->clk);
	ndelay(100);
	clk_prepare_enable(krio_priv->clk);

	pdev->dev.platform_data = (void *)krio_priv;

	keystone_rio_sysfs_create(&pdev->dev);

	/* Setup the SRIO controller */
	if (enable_ports) {
		r = keystone_rio_setup_controller(krio_priv);
		if (r < 0) {
			clk_disable_unprepare(krio_priv->clk);
			clk_put(krio_priv->clk);
			return r;
		}
	}

	return 0;
}

static void keystone_rio_shutdown_controller(
	struct keystone_rio_data *krio_priv)
{
	u32 lanes = krio_priv->board_rio_cfg.lanes;
	int i;

	dev_dbg(krio_priv->dev, "shutdown controller\n");

	/* Unregister interrupt handlers */
	keystone_rio_interrupt_release(krio_priv);

	/* Shutdown associated SerDes */
	krio_priv->serdes.ops->shutdown_lanes(lanes, &krio_priv->serdes);

	/* Stop the hw controller */
	keystone_rio_stop(krio_priv);

	/* Disable blocks */
	krio_write(krio_priv, gbl_en, 0);
	for (i = 0; i < KEYSTONE_RIO_BLK_NUM; i++) {
		krio_write(krio_priv, blk[i].enable, 0);
		while (krio_read(krio_priv, blk[i].status) & 0x1)
			usleep_range(10, 50);
	}

	krio_priv->started = 0;
}

static void keystone_rio_shutdown(struct platform_device *pdev)
{
	struct keystone_rio_data *krio_priv = platform_get_drvdata(pdev);

	if (krio_priv->started)
		keystone_rio_shutdown_controller(krio_priv);

	/* Wait current DMA transfers to finish */
	mdelay(10);

	if (krio_priv->clk) {
		clk_disable_unprepare(krio_priv->clk);
		clk_put(krio_priv->clk);
	}
}

static int keystone_rio_remove(struct platform_device *pdev)
{
	struct keystone_rio_data *krio_priv = platform_get_drvdata(pdev);
	u32 ports = krio_priv->board_rio_cfg.ports;

	/* Shutdown the hw controller */
	keystone_rio_shutdown(pdev);

	flush_scheduled_work();

	/* Retrieve all registered mports */
	ports = krio_priv->board_rio_cfg.ports;
	while (ports) {
		struct rio_mport *mport;
		u32 port = __ffs(ports);

		ports &= ~BIT(port);

		mport = krio_priv->mport[port];

		if (mport) {
#ifdef CONFIG_RAPIDIO_DMA_ENGINE
			keystone_rio_lsu_dma_free_channel(mport);
			keystone_rio_dma_unregister(mport);
#endif
		}
	}

	/* Remove io mapping */
	iounmap(krio_priv->jtagid_reg);
	iounmap(krio_priv->serdes_regs);
	iounmap(krio_priv->regs);

	/* Unregister sysfs and free mport private structures */
	keystone_rio_serdes_unregister(&pdev->dev, &krio_priv->serdes);
	keystone_rio_sysfs_remove(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	kfree(krio_priv);

	return 0;
}

static const struct of_device_id of_match[] = {
	{ .compatible = "ti,keystone-rapidio", },
	{},
};

MODULE_DEVICE_TABLE(of, keystone_hwqueue_of_match);

static struct platform_driver keystone_rio_driver = {
	.driver = {
		.name	        = "keystone-rapidio",
		.of_match_table	= of_match,
	},
	.probe	= keystone_rio_probe,
	.remove = keystone_rio_remove,
	.shutdown = keystone_rio_shutdown,
};
module_platform_driver(keystone_rio_driver);

MODULE_AUTHOR("Aurelien Jacquiot");
MODULE_DESCRIPTION("TI KeyStone RapidIO device driver");
MODULE_LICENSE("GPL");
