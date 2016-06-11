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
#ifndef KEYSTONE_RIO_H
#define KEYSTONE_RIO_H

#include <asm/setup.h>
#include <linux/cache.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/soc/ti/knav_qmss.h>
#include <linux/soc/ti/knav_dma.h>

#define KEYSTONE_RIO_MAP_FLAG_SEGMENT	  BIT(0)
#define KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC BIT(1)
#define KEYSTONE_RIO_MAP_FLAG_TT_16	  BIT(13)
#define KEYSTONE_RIO_MAP_FLAG_DST_PROMISC BIT(15)
#define KEYSTONE_RIO_DESC_FLAG_TT_16	  BIT(9)

#define KEYSTONE_RIO_BOOT_COMPLETE	  BIT(24)
#define KEYSTONE_RIO_PER_RESTORE          BIT(4)
#define KEYSTONE_RIO_PER_EN		  BIT(2)
#define KEYSTONE_RIO_PER_FREE		  BIT(0)
#define KEYSTONE_RIO_PEF_FLOW_CONTROL	  BIT(7)

/*
 * Packet types
 */
#define KEYSTONE_RIO_PACKET_TYPE_NREAD    0x24
#define KEYSTONE_RIO_PACKET_TYPE_NWRITE   0x54
#define KEYSTONE_RIO_PACKET_TYPE_NWRITE_R 0x55
#define KEYSTONE_RIO_PACKET_TYPE_SWRITE   0x60
#define KEYSTONE_RIO_PACKET_TYPE_DBELL    0xa0
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_R  0x80
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_W  0x81
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_RR 0x82
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_WR 0x83
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_PW 0x84

#define KEYSTONE_RIO_PACKET_TYPE_MASK     0xff

#define KEYSTONE_RIO_PACKET_TYPE(t)       ((t) & KEYSTONE_RIO_PACKET_TYPE_MASK)
#define IS_KEYSTONE_RIO_PACKET_TYPE(t, p) (KEYSTONE_RIO_PACKET_TYPE(t) == p)

/*
 * LSU defines
 */
#define KEYSTONE_RIO_LSU_PRIO             0
#define KEYSTONE_RIO_LSU_NUM              8

#define KEYSTONE_RIO_LSU_BUSY_MASK        BIT(31)
#define KEYSTONE_RIO_LSU_FULL_MASK        BIT(30)

#define KEYSTONE_RIO_LSU_CC_MASK          0x0f
#define KEYSTONE_RIO_LSU_CC_TIMEOUT       0x01
#define KEYSTONE_RIO_LSU_CC_XOFF          0x02
#define KEYSTONE_RIO_LSU_CC_ERROR         0x03
#define KEYSTONE_RIO_LSU_CC_INVALID       0x04
#define KEYSTONE_RIO_LSU_CC_DMA           0x05
#define KEYSTONE_RIO_LSU_CC_RETRY         0x06
#define KEYSTONE_RIO_LSU_CC_CANCELED      0x07

/*
 * Max ports configuration per path modes
 */
#define KEYSTONE_RIO_MAX_PORTS_PATH_MODE_0	0xf /* 4 ports:    4-1x    */
#define KEYSTONE_RIO_MAX_PORTS_PATH_MODE_1	0xd /* 3 ports: 2-1x, 1-2x */
#define KEYSTONE_RIO_MAX_PORTS_PATH_MODE_2	0x7 /* 3 ports: 1-2x, 2-1x */
#define KEYSTONE_RIO_MAX_PORTS_PATH_MODE_3	0x5 /* 2 ports: 1-2x, 1-2x */
#define KEYSTONE_RIO_MAX_PORTS_PATH_MODE_4      0x1 /* 1 ports:       1-4x */

/*
 * Register range defines
 */
#define KEYSTONE_RIO_CAR_CSR_REGS         0x0b000
#define KEYSTONE_RIO_SERIAL_PORT_REGS     0x0b100
#define KEYSTONE_RIO_ERR_MGMT_REGS        0x0c000
#define KEYSTONE_RIO_PHY_REGS             0x1b000
#define KEYSTONE_RIO_TRANSPORT_REGS       0x1b300
#define KEYSTONE_RIO_PKT_BUF_REGS         0x1b600
#define KEYSTONE_RIO_EVT_MGMT_REGS        0x1b900
#define KEYSTONE_RIO_PORT_WRITE_REGS      0x1ba00
#define KEYSTONE_RIO_LINK_REGS            0x1bd00
#define KEYSTONE_RIO_FABRIC_REGS          0x1be00

/*
 * Various RIO defines
 */
#define KEYSTONE_RIO_DBELL_NUMBER         4
#define KEYSTONE_RIO_DBELL_VALUE_MAX      (KEYSTONE_RIO_DBELL_NUMBER * 16)
#define KEYSTONE_RIO_DBELL_MASK           (KEYSTONE_RIO_DBELL_VALUE_MAX - 1)

#define KEYSTONE_RIO_TIMEOUT_CNT	  1000
#define KEYSTONE_RIO_TIMEOUT_MSEC         100
#define KEYSTONE_RIO_TIMEOUT_NSEC         1000
#define KEYSTONE_RIO_RETRY_CNT            1000
#define KEYSTONE_RIO_REGISTER_DELAY	  (3 * HZ)

/*
 * RIO error, reset and special event interrupt defines
 */
#define KEYSTONE_RIO_PORT_ERROR_OUT_PKT_DROP		BIT(26)
#define KEYSTONE_RIO_PORT_ERROR_OUT_FAILED		BIT(25)
#define KEYSTONE_RIO_PORT_ERROR_OUT_DEGRADED		BIT(24)
#define KEYSTONE_RIO_PORT_ERROR_OUT_RETRY		BIT(20)
#define KEYSTONE_RIO_PORT_ERROR_OUT_ERROR		BIT(17)
#define KEYSTONE_RIO_PORT_ERROR_IN_ERROR		BIT(9)
#define KEYSTONE_RIO_PORT_ERROR_PW_PENDING		BIT(4)
#define KEYSTONE_RIO_PORT_ERROR_PORT_ERR		BIT(2)

#define KEYSTONE_RIO_PORT_ERROR_MASK			\
	(KEYSTONE_RIO_PORT_ERROR_OUT_PKT_DROP	|	\
	 KEYSTONE_RIO_PORT_ERROR_OUT_FAILED	|	\
	 KEYSTONE_RIO_PORT_ERROR_OUT_DEGRADED	|	\
	 KEYSTONE_RIO_PORT_ERROR_OUT_RETRY	|	\
	 KEYSTONE_RIO_PORT_ERROR_OUT_ERROR	|	\
	 KEYSTONE_RIO_PORT_ERROR_IN_ERROR	|	\
	 KEYSTONE_RIO_PORT_ERROR_PW_PENDING	|	\
	 KEYSTONE_RIO_PORT_ERROR_PORT_ERR)

#define KEYSTONE_RIO_PORT_ERRORS		\
	(RIO_PORT_N_ERR_STS_PW_OUT_ES	|	\
	 RIO_PORT_N_ERR_STS_PW_INP_ES	|	\
	 RIO_PORT_N_ERR_STS_PW_PEND	|	\
	 RIO_PORT_N_ERR_STS_PORT_ERR)

/* RIO PLM port event status defines */
#define KEYSTONE_RIO_PORT_PLM_STATUS_MAX_DENIAL        BIT(31)
#define KEYSTONE_RIO_PORT_PLM_STATUS_LINK_INIT         BIT(28)
#define KEYSTONE_RIO_PORT_PLM_STATUS_DLT               BIT(27)
#define KEYSTONE_RIO_PORT_PLM_STATUS_PORT_ERR          BIT(26)
#define KEYSTONE_RIO_PORT_PLM_STATUS_OUTPUT_FAIL       BIT(25)
#define KEYSTONE_RIO_PORT_PLM_STATUS_OUTPUT_DEGR       BIT(24)
#define KEYSTONE_RIO_PORT_PLM_STATUS_RST_REQ           BIT(16)
#define KEYSTONE_RIO_PORT_PLM_STATUS_MECS              BIT(12)

#define KEYSTONE_RIO_PORT_PLM_STATUS_ERRORS			\
	(KEYSTONE_RIO_PORT_PLM_STATUS_MAX_DENIAL      |		\
	 KEYSTONE_RIO_PORT_PLM_STATUS_PORT_ERR        |		\
	 KEYSTONE_RIO_PORT_PLM_STATUS_OUTPUT_FAIL     |		\
	 KEYSTONE_RIO_PORT_PLM_STATUS_OUTPUT_DEGR)

#define KEYSTONE_RIO_SP_HDR_NEXT_BLK_PTR	0x1000
#define KEYSTONE_RIO_SP_HDR_EP_REC_ID		0x0002
#define KEYSTONE_RIO_ERR_HDR_NEXT_BLK_PTR	0x3000
#define KEYSTONE_RIO_ERR_EXT_FEAT_ID		0x0007

/*
 * RapidIO global definitions
 */
#define KEYSTONE_RIO_MAX_PORT		4
#define KEYSTONE_RIO_MAX_MBOX		4    /* 4 in multi-segment,
					      * 64 in single-seg
					      */
#define KEYSTONE_RIO_MAX_PKT_FW_ENTRIES 8    /* max of packet forwarding
					      * mapping entries
					      */
#define KEYSTONE_RIO_MAINT_BUF_SIZE	64
#define KEYSTONE_RIO_MSG_SSIZE		0xe
#define KEYSTONE_RIO_SGLIST_SIZE	3

#define KEYSTONE_RIO_PKT_FW_BRR_NUM     1    /* BRR used for packet fwd */

#define KEYSTONE_RIO_RXU_MAP_MIN	0
#define KEYSTONE_RIO_RXU_MAP_MAX	63   /* RXU mapping range */

/*
 * RapidIO logical block definitions
 */
#define KEYSTONE_RIO_BLK_MMR_ID         0
#define KEYSTONE_RIO_BLK_LSU_ID         1
#define KEYSTONE_RIO_BLK_MAU_ID         2
#define KEYSTONE_RIO_BLK_TXU_ID         3
#define KEYSTONE_RIO_BLK_RXU_ID         4
#define KEYSTONE_RIO_BLK_PORT0_ID       5
#define KEYSTONE_RIO_BLK_PORT1_ID       6
#define KEYSTONE_RIO_BLK_PORT2_ID       7
#define KEYSTONE_RIO_BLK_PORT3_ID       8

#define KEYSTONE_RIO_BLK_NUM		9

/*
 * DMA engine definition
 */
#define KEYSTONE_RIO_DMA_MAX_CHANNEL    8    /* Num of (virtual) DMA channels */

/* Max number of virtual DMA descriptors */
#ifdef ARCH_HAS_SG_CHAIN
#define KEYSTONE_RIO_DMA_MAX_DESC       1024
#else
#define KEYSTONE_RIO_DMA_MAX_DESC       SG_MAX_SINGLE_ALLOC
#endif

/*
 * Dev Id and dev revision
 */
#define KEYSTONE_RIO_DEV_ID_VAL	\
	((((__raw_readl(krio_priv->jtagid_reg)) << 4)  & 0xffff0000) | 0x30)

#define KEYSTONE_RIO_DEV_INFO_VAL \
	(((__raw_readl(krio_priv->jtagid_reg)) >> 28) & 0xf)

#define KEYSTONE_RIO_ID_TI		(0x00000030)
#define KEYSTONE_RIO_EXT_FEAT_PTR	(0x00000100)

#define KEYSTONE_RIO_MAX_DIO_PKT_SIZE   0x100000 /* hw support up to 1MB */
#define KEYSTONE_RIO_DIO_ALIGNMENT      8        /* hw required alignment */

/*
 * RIO error, reset and special event interrupt defines
 */
#define KEYSTONE_RIO_ERR_RST_EVNT_MASK  0x00010f07

/* Refer to bits in KEYSTONE_RIO_ERR_RST_EVNT_MASK */
#define KEYSTONE_RIO_RESET_INT          16  /* reset interrupt on any port */
#define KEYSTONE_RIO_PORT3_ERROR_INT    11  /* port 3 error */
#define KEYSTONE_RIO_PORT2_ERROR_INT    10  /* port 2 error */
#define KEYSTONE_RIO_PORT1_ERROR_INT    9   /* port 1 error */
#define KEYSTONE_RIO_PORT0_ERROR_INT    8   /* port 0 error */
#define KEYSTONE_RIO_EVT_CAP_ERROR_INT  2   /* error management event capture */
#define KEYSTONE_RIO_PORT_WRITEIN_INT   1   /* port-write-in request received */
#define KEYSTONE_RIO_MCAST_EVT_INT      0   /* multicast event control symbol */

/*
 * Interrupts and DMA event mapping
 */
#define KEYSTONE_GEN_RIO_INT            0   /* RIO int for generic events */
#define KEYSTONE_LSU_RIO_INT            1   /* RIO int for LSU events */

/* Mask for error and good completion LSU interrupts */
#define KEYSTONE_RIO_ICSR_LSU0_ERROR_MASK     0xffff0000
#define KEYSTONE_RIO_ICSR_LSU0_COMPLETE_MASK  0x0000ffff
#define KEYSTONE_RIO_ICSR_LSU1_COMPLETE_MASK  0x000000ff

/*
 * Definition of the different RapidIO packet types according to the RapidIO
 * specification 2.0
 */
#define RIO_PACKET_TYPE_STREAM          9  /* Data Streaming */
#define RIO_PACKET_TYPE_MESSAGE         11 /* Message */

/*
 * Routing configuration for packet forwarding
 */
struct keystone_routing_config {
	u16 dev_id_low;
	u16 dev_id_high;
	u8  port;
};

/*
 * Per board RIO devices controller configuration
 */
struct keystone_rio_board_controller_info {
	u32		rio_regs_base;
	u32		rio_regs_size;

	u32		boot_cfg_regs_base;
	u32		boot_cfg_regs_size;

	u32		serdes_cfg_regs_base;
	u32		serdes_cfg_regs_size;

	/* bitfield of port(s) to probe on this controller */
	u32             ports;
	/* remote link partner port numbers */
	int             ports_remote[KEYSTONE_RIO_MAX_PORT];

	u32             id;     /* host id */
	u32             size;   /* RapidIO common transport system size.
				 * 0 - Small size. 256 devices.
				 * 1 - Large size, 65536 devices.
				 */
	u16             serdes_type;
	u32             serdes_baudrate;
	u32             serdes_calibration;
	u32             lanes;
	u32             path_mode;
	u32             port_register_timeout;
	u32             pkt_forwarding;
	u32             dma_channel_num; /* DMA channels for DIO transfers */

	int             rio_irq;
	int             lsu_irq;

	struct keystone_serdes_config  serdes_config;
	struct keystone_routing_config routing_config[8];
};

struct keystone_rio_data;

struct keystone_rio_packet {
	struct scatterlist		sg[KEYSTONE_RIO_SGLIST_SIZE];
	int				sg_ents;
	u32				epib[4];
	u32				psdata[2];
	u32				mbox;
	u32                             slot;
	void			       *buff;
	struct keystone_rio_data       *priv;
	enum dma_status			status;
	dma_cookie_t			cookie;
};

struct keystone_rio_mbox_info {
	struct rio_mport *port;
	u32               running;
	u32               entries;
	atomic_t          slot;
	void             *dev_id;
	int		  rxu_map_id[2];
};

struct keystone_rio_rx_chan_info {
	struct keystone_rio_data *priv;
	void                     *channel;
	void			 *queue;      /* Rx completion queue */
	const char		 *name;
	struct tasklet_struct	  tasklet;
	void			 *pool;
	u32                       pool_size;
	u32                       pool_region_id;
	int                       mbox_id;
	void                     *fdq[KNAV_DMA_FDQ_PER_CHAN];
	u32			  queue_depths[KNAV_DMA_FDQ_PER_CHAN];
	u32			  buffer_sizes[KNAV_DMA_FDQ_PER_CHAN];
	int			  flow_id;
	int			  queue_id;   /* Rx completion queue */
	u32                       packet_type;
	int                       stream_id;  /* stream id for type 9 packet */
};

struct keystone_rio_tx_chan_info {
	struct keystone_rio_data *priv;
	void                     *channel;
	void			 *complet_queue;    /* Tx completion queue */
	void                     *queue;            /* Tx submit queue */
	void                     *garbage_queue;    /* Tx garbage queue */
	const char	         *name;
	void			 *pool;
	u32                       pool_size;
	u32                       pool_region_id;
	int                       mbox_id;
	u32			  queue_depth;
	u32                       complet_queue_id; /* Tx completion queue */
	u32                       queue_id;         /* Tx submit queue */
	u32                       garbage_queue_id; /* Tx garbage queue */
};

struct port_write_msg {
	union rio_pw_msg msg;
	u32              msg_count;
	u32              err_count;
	u32              discard_count;
};

#ifdef CONFIG_RAPIDIO_DMA_ENGINE

/*
 * Special DMA device control operation to prepare a raw packet
 * (used for dbell)
 */
#define DMA_KEYSTONE_RIO_PREP_RAW_PACKET 0x1000

enum keystone_rio_chan_state {
	RIO_CHAN_STATE_UNUSED,
	RIO_CHAN_STATE_ACTIVE,
	RIO_CHAN_STATE_RUNNING,
	RIO_CHAN_STATE_WAITING,
};

/*
 * DMA engine data
 */
struct keystone_rio_dma_desc {
	struct list_head	        node;
	struct list_head	        tx_list;
	bool			        last;
	struct dma_async_tx_descriptor  adesc;
	enum dma_status		        status;
	u32                             retry_count;
	dma_addr_t                      buff_addr;
	u32                             size;
	u16                             port_id;
	u16			        dest_id;
	u64			        rio_addr;
	u8			        rio_addr_u;
	u8                              sys_size;
	u32                             lsu_context;
	u32                             packet_type;
} ____cacheline_aligned;

struct keystone_rio_dma_chan {
	struct list_head                node;
	enum dma_transfer_direction     direction;
	atomic_t		        state;
	struct dma_chan		        dchan;
	u8                              lsu;
	struct list_head	        active_list;
	struct list_head	        queue;
	struct keystone_rio_dma_desc   *current_transfer;
	dma_cookie_t		        completed_cookie;
	/* to protect against concurrent access to dma channel */
	spinlock_t		        lock;
	struct tasklet_struct	        tasklet;
	struct keystone_rio_data       *krio;
};

struct keystone_rio_dma_packet_raw {
	u16                             port_id;
	u16			        dest_id;
	u64			        rio_addr;
	u8			        rio_addr_u;
	dma_addr_t                      buff_addr;
	u32                             size;
	u8                              sys_size;
	u32                             packet_type;
	struct dma_async_tx_descriptor *tx;
};

#define from_dma_chan(ch) container_of(ch, struct keystone_rio_dma_chan, dchan)
#define to_dma_chan(ch)	  (&(ch)->dchan)
#define chan_dev(ch)	  (&to_dma_chan(ch)->dev->device)
#define chan_id(ch)	  (to_dma_chan(ch)->chan_id)
#define chan_name(ch)	  ((ch)->dchan.name)

void keystone_rio_dma_interrupt_handler(struct keystone_rio_data *krio_priv,
					u32 lsu, u32 error);

int keystone_rio_dma_prep_raw_packet(struct dma_chan *dchan,
				     struct keystone_rio_dma_packet_raw *pkt);

int keystone_rio_dma_register(struct rio_mport *mport, int channel_num);

void keystone_rio_dma_unregister(struct rio_mport *mport);

u8 keystone_rio_lsu_alloc(struct keystone_rio_data *krio_priv);

int keystone_rio_lsu_complete_transfer(int lsu, u32 lsu_context,
				       struct keystone_rio_data *krio_priv);

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
				    struct keystone_rio_data *krio_priv);
#endif /* CONFIG_RAPIDIO_DMA_ENGINE */

/*
 * Main KeyStone RapidIO driver data
 *   ports: port mask of those ports defined in dts and
 *          have good hw port status
 */
struct keystone_rio_data {
	struct device	       *dev;
	struct rio_mport       *mport[KEYSTONE_RIO_MAX_PORT];
	struct clk	       *clk;

	u32                     started;
	u32                     calibrating;

	/* To protect against concurrent lsu maint access */
	struct mutex		lsu_lock_maint;
	u8                      lsu_start;
	u8                      lsu_end;
	u8                      lsu_free;
	u8                      lsu_maint;

	u32			rio_pe_feat;
	u32                     base_dev_id;

	struct port_write_msg	port_write_msg;
	struct work_struct	pw_work;
	struct kfifo		pw_fifo;
	/* to protect against concurrent port write */
	spinlock_t		pw_fifo_lock;

	u32                     pe_ports;
	u32                     pe_cnt;
	struct delayed_work     pe_work;

	struct work_struct      reset_work;

	u32                     ports;
	u32			ports_scan_registering;
	u32			ports_registering;

	/* to protect against concurrent port check */
	spinlock_t		port_chk_lock;
	u32			port_chk_cnt;
	struct delayed_work	port_chk_task;

	unsigned long		rxu_map_start;
	unsigned long		rxu_map_end;
	unsigned long		rxu_map_bitmap[2];

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	struct list_head        dma_channels[KEYSTONE_RIO_LSU_NUM];
	struct dma_chan        *dma_chan[KEYSTONE_RIO_MAX_PORT];
#endif

	u32                              num_mboxes;
	struct keystone_rio_mbox_info    tx_mbox[KEYSTONE_RIO_MAX_MBOX];
	struct keystone_rio_mbox_info    rx_mbox[KEYSTONE_RIO_MAX_MBOX];
	struct keystone_rio_rx_chan_info rx_channels[KEYSTONE_RIO_MAX_MBOX];
	struct keystone_rio_tx_chan_info tx_channels[KEYSTONE_RIO_MAX_MBOX];

	u32 __iomem					*jtagid_reg;
	u32 __iomem					*serdes_sts_reg;
	void __iomem	                                *serdes_regs;
	struct keystone_rio_regs __iomem	        *regs;

	struct keystone_rio_car_csr_regs __iomem	*car_csr_regs;
	struct keystone_rio_serial_port_regs __iomem	*serial_port_regs;
	struct keystone_rio_err_mgmt_regs __iomem	*err_mgmt_regs;
	struct keystone_rio_phy_layer_regs __iomem	*phy_regs;
	struct keystone_rio_transport_layer_regs __iomem *transport_regs;
	struct keystone_rio_pkt_buf_regs __iomem	*pkt_buf_regs;
	struct keystone_rio_evt_mgmt_regs __iomem	*evt_mgmt_regs;
	struct keystone_rio_port_write_regs __iomem	*port_write_regs;
	struct keystone_rio_link_layer_regs __iomem	*link_regs;
	struct keystone_rio_fabric_regs __iomem		*fabric_regs;

	struct keystone_rio_board_controller_info	 board_rio_cfg;

	struct keystone_serdes_data                      serdes;
	/* To protect against concurrent accesses */
	struct mutex					lsu_lock_dbell;
	u8						lsu_dbell;
};

struct keystone_lane_config {
	int start; /* lane start number of the port */
	int end;   /* lane end number of the port */
};

/*
 * RapidIO Registers
 */

/* RIO Registers  0000 - 2fff */
struct keystone_rio_regs {
/* Required Peripheral Registers */
	u32	pid;			/* 0000 */
	u32	pcr;			/* 0004 */
	u32	__rsvd0[3];		/* 0008 - 0010 */

/* Peripheral Setting Control Registers */
	u32	per_set_cntl;		/* 0014 */
	u32	per_set_cntl1;		/* 0018 */

	u32	__rsvd1[2];		/* 001c - 0020 */

	u32	gbl_en;			/* 0024 */
	u32	gbl_en_stat;		/* 0028 */

	struct {
		u32 enable;		/* 002c */
		u32 status;		/* 0030 */
	} blk[10];			/* 002c - 0078 */

	/* ID Registers */
	u32	__rsvd2[17];		/* 007c - 00bc */
	u32	multiid_reg[8];		/* 00c0 - 00dc */

/* Hardware Packet Forwarding Registers */
	struct {
		u32	pf_16b;
		u32	pf_8b;
	} pkt_fwd_cntl[8];		/* 00e0 - 011c */

	u32	__rsvd3[24];		/* 0120 - 017c */

/* Interrupt Registers */
	struct {
		u32	status;
		u32	__rsvd0;
		u32	clear;
		u32	__rsvd1;
	} doorbell_int[4];		/* 0180 - 01bc */

	struct {
		u32	status;
		u32	__rsvd0;
		u32	clear;
		u32	__rsvd1;
	} lsu_int[2];			/* 01c0 - 01dc */

	u32	err_rst_evnt_int_stat;	/* 01e0 */
	u32	__rsvd4;
	u32	err_rst_evnt_int_clear;	/* 01e8 */
	u32	__rsvd5;

	u32	__rsvd6[4];		/* 01f0 - 01fc */

	struct {
		u32 route;		/* 0200 */
		u32 route2;		/* 0204 */
		u32 __rsvd;		/* 0208 */
	} doorbell_int_route[4];	/* 0200 - 022c */

	u32	lsu0_int_route[4];		/* 0230 - 023c */
	u32	lsu1_int_route1;		/* 0240 */

	u32	__rsvd7[3];		/* 0244 - 024c */

	u32	err_rst_evnt_int_route[3];	/* 0250 - 0258 */

	u32	__rsvd8[2];		/* 025c - 0260 */

	u32	interrupt_ctl;		/* 0264 */

	u32	__rsvd9[26];		/* 0268, 026c, 0270 - 02cc */

	u32	intdst_rate_cntl[16];	/* 02d0 - 030c */
	u32	intdst_rate_disable;	/* 0310 */

	u32	__rsvd10[59];		/* 0314 - 03fc */

/* RXU Registers */
	struct {
		u32	ltr_mbox_src;
		u32	dest_prom_seg;
		u32	flow_qid;
	} rxu_map[64];			/* 0400 - 06fc */

	struct {
		u32	cos_src;
		u32	dest_prom;
		u32	stream;
	} rxu_type9_map[64];		/* 0700 - 09fc */

	u32	__rsvd11[192];		/* 0a00 - 0cfc */

/* LSU/MAU Registers */
	struct {
		u32 addr_msb;		/* 0d00 */
		u32 addr_lsb_cfg_ofs;	/* 0d04 */
		u32 phys_addr;		/* 0d08 */
		u32 dbell_val_byte_cnt;	/* 0d0c */
		u32 destid;		/* 0d10 */
		u32 dbell_info_fttype;	/* 0d14 */
		u32 busy_full;		/* 0d18 */
	} lsu_reg[8];			/* 0d00 - 0ddc */

	u32	lsu_setup_reg[2];	/* 0de0 - 0de4 */
	u32	lsu_stat_reg[6];	/* 0de8 - 0dfc */
	u32	lsu_flow_masks[4];	/* 0e00 - 0e0c */

	u32	__rsvd12[16];		/* 0e10 - 0e4c */

/* Flow Control Registers */
	u32	flow_cntl[16];		/* 0e50 - 0e8c */
	u32	__rsvd13[8];		/* 0e90 - 0eac */

/* TXU Registers 0eb0 - 0efc */
	u32	tx_cppi_flow_masks[8];	/* 0eb0 - 0ecc */
	u32	tx_queue_sch_info[4];	/* 0ed0 - 0edc */
	u32	garbage_coll_qid[3];	/* 0ee0 - 0ee8 */

	u32	__rsvd14[69];		/* 0eec, 0ef0 - 0ffc */

};

/* CDMAHP Registers 1000 - 2ffc */
struct keystone_rio_pktdma_regs {
	u32	__rsvd[2048];		/* 1000 - 2ffc */
};

/* CSR/CAR Registers  b000+ */
struct keystone_rio_car_csr_regs {
	u32	dev_id;			/* b000 */
	u32	dev_info;		/* b004 */
	u32	assembly_id;		/* b008 */
	u32	assembly_info;		/* b00c */
	u32	pe_feature;		/* b010 */

	u32	sw_port;		/* b014 */

	u32	src_op;			/* b018 */
	u32	dest_op;		/* b01c */

	u32	__rsvd1[7];		/* b020 - b038 */

	u32	data_stm_info;		/* b03c */

	u32	__rsvd2[2];		/* b040 - b044 */

	u32	data_stm_logical_ctl;	/* b048 */
	u32	pe_logical_ctl;		/* b04c */

	u32	__rsvd3[2];		/* b050 - b054 */

	u32	local_cfg_hbar;		/* b058 */
	u32	local_cfg_bar;		/* b05c */

	u32	base_dev_id;		/* b060 */
	u32	__rsvd4;
	u32	host_base_id_lock;	/* b068 */
	u32	component_tag;		/* b06c */
					/* b070 - b0fc */
};

struct keystone_rio_serial_port_regs {
	u32	sp_maint_blk_hdr;	/* b100 */
	u32	__rsvd6[7];		/* b104 - b11c */

	u32	sp_link_timeout_ctl;	/* b120 */
	u32	sp_rsp_timeout_ctl;	/* b124 */
	u32	__rsvd7[5];		/* b128 - b138 */
	u32	sp_gen_ctl;		/* b13c */

	struct {
		u32	link_maint_req;	/* b140 */
		u32	link_maint_resp;/* b144 */
		u32	ackid_stat;	/* b148 */
		u32	__rsvd[2];	/* b14c - b150 */
		u32	ctl2;		/* b154 */
		u32	err_stat;	/* b158 */
		u32	ctl;		/* b15c */
	} sp[4];			/* b140 - b1bc */

					/* b1c0 - bffc */
};

struct keystone_rio_err_mgmt_regs {
	u32	err_report_blk_hdr;	/* c000 */
	u32	__rsvd9;
	u32	err_det;		/* c008 */
	u32	err_en;			/* c00c */
	u32	h_addr_capt;		/* c010 */
	u32	addr_capt;		/* c014 */
	u32	id_capt;		/* c018 */
	u32	ctrl_capt;		/* c01c */
	u32	__rsvd10[2];		/* c020 - c024 */
	u32	port_write_tgt_id;	/* c028 */
	u32	__rsvd11[5];		/* c02c - c03c */

	struct {
		u32	det;		/* c040 */
		u32	rate_en;	/* c044 */
		u32	attr_capt_dbg0;	/* c048 */
		u32	capt_0_dbg1;	/* c04c */
		u32	capt_1_dbg2;	/* c050 */
		u32	capt_2_dbg3;	/* c054 */
		u32	capt_3_dbg4;	/* c058 */
		u32	__rsvd0[3];	/* c05c - c064 */
		u32	rate;		/* c068 */
		u32	thresh;		/* c06c */
		u32	__rsvd1[4];	/* c070 - c07c */
	} sp_err[4];			/* c040 - c13c */

	u32	__rsvd12[1972];		/* c140 - e00c */

	struct {
		u32	stat0;		/* e010 */
		u32	stat1;		/* e014 */
		u32	__rsvd[6];	/* e018 - e02c */
	} lane_stat[4];			/* e010 - e08c */

					/* e090 - 1affc */
};

struct keystone_rio_phy_layer_regs {
	u32	phy_blk_hdr;		/* 1b000 */
	u32	__rsvd14[31];		/* 1b004 - 1b07c */
	struct {
		u32	imp_spec_ctl;	/* 1b080 */
		u32	pwdn_ctl;	/* 1b084 */
		u32	__rsvd0[2];

		u32	status;		/* 1b090 */
		u32	int_enable;	/* 1b094 */
		u32	port_wr_enable;	/* 1b098 */
		u32	event_gen;	/* 1b09c */

		u32	all_int_en;	/* 1b0a0 */
		u32	all_port_wr_en;	/* 1b0a4 */
		u32	__rsvd1[2];

		u32	path_ctl;	/* 1b0b0 */
		u32	discovery_timer;/* 1b0b4 */
		u32	silence_timer;	/* 1b0b8 */
		u32	vmin_exp;	/* 1b0bc */

		u32	pol_ctl;	/* 1b0c0 */
		u32	__rsvd2;
		u32	denial_ctl;	/* 1b0c8 */
		u32	__rsvd3;

		u32	rcvd_mecs;	/* 1b0d0 */
		u32	__rsvd4;
		u32	mecs_fwd;	/* 1b0d8 */
		u32	__rsvd5;

		u32	long_cs_tx1;	/* 1b0e0 */
		u32	long_cs_tx2;	/* 1b0e4 */
		u32	__rsvd[6];	/* 1b0e8, 1b0ec, 1b0f0 - 1b0fc */
	} phy_sp[4];			/* 1b080 - 1b27c */

					/* 1b280 - 1b2fc */
};

struct keystone_rio_transport_layer_regs {
	u32	transport_blk_hdr;	/* 1b300 */
	u32	__rsvd16[31];		/* 1b304 - 1b37c */

	struct {
		u32	control;	/*1b380 */
		u32	__rsvd0[3];

		u32	status;		/* 1b390 */
		u32	int_enable;	/* 1b394 */
		u32	port_wr_enable;	/* 1b398 */
		u32	event_gen;	/* 1b39c */

		struct {
			u32	ctl;		/* 1b3a0 */
			u32	pattern_match;	/* 1b3a4 */
			u32	__rsvd[2];	/* 1b3a8 - 1b3ac */
		} base_route[4];		/* 1b3a0 - 1b3dc */

		u32	__rsvd1[8];		/* 1b3e0 - 1b3fc */

	} transport_sp[4];			/* 1b380 - 1b57c */

						/* 1b580 - 1b5fc */
};

struct keystone_rio_pkt_buf_regs {
	u32	pkt_buf_blk_hdr;	/* 1b600 */
	u32	__rsvd18[31];		/* 1b604 - 1b67c */

	struct {
		u32	control;	/* 1b680 */
		u32	__rsvd0[3];

		u32	status;		/* 1b690 */
		u32	int_enable;	/* 1b694 */
		u32	port_wr_enable;	/* 1b698 */
		u32	event_gen;	/* 1b69c */

		u32	ingress_rsc;	/* 1b6a0 */
		u32	egress_rsc;	/* 1b6a4 */
		u32	__rsvd1[2];

		u32	ingress_watermark[4];	/* 1b6b0 - 1b6bc */
		u32	__rsvd2[16];	/* 1b6c0 - 1b6fc */

	} pkt_buf_sp[4];		/* 1b680 - 1b87c */

					/* 1b880 - 1b8fc */
};

struct keystone_rio_evt_mgmt_regs {
	u32	evt_mgmt_blk_hdr;	/* 1b900 */
	u32	__rsvd20[3];

	u32	evt_mgmt_int_stat;	/* 1b910 */
	u32	evt_mgmt_int_enable;	/* 1b914 */
	u32	evt_mgmt_int_port_stat;	/* 1b918 */
	u32	__rsvd21;

	u32	evt_mgmt_port_wr_stat;	/* 1b920 */
	u32	evt_mgmt_port_wr_enable;/* 1b924 */
	u32	evt_mgmt_port_wr_port_stat;	/* 1b928 */
	u32	__rsvd22;

	u32	evt_mgmt_dev_int_en;	/* 1b930 */
	u32	evt_mgmt_dev_port_wr_en;	/* 1b934 */
	u32	__rsvd23;
	u32	evt_mgmt_mecs_stat;	/* 1b93c */

	u32	evt_mgmt_mecs_int_en;	/* 1b940 */
	u32	evt_mgmt_mecs_cap_en;	/* 1b944 */
	u32	evt_mgmt_mecs_trig_en;	/* 1b948 */
	u32	evt_mgmt_mecs_req;	/* 1b94c */

	u32	evt_mgmt_mecs_port_stat;/* 1b950 */
	u32	__rsvd24[2];
	u32	evt_mgmt_mecs_event_gen;/* 1b95c */

	u32	evt_mgmt_rst_port_stat;	/* 1b960 */
	u32	__rsvd25;
	u32	evt_mgmt_rst_int_en;	/* 1b968 */
	u32	__rsvd26;

	u32	evt_mgmt_rst_port_wr_en;/* 1b970 */
					/* 1b974 - 1b9fc */
};

struct keystone_rio_port_write_regs {
	u32	port_wr_blk_hdr;	/* 1ba00 */
	u32	port_wr_ctl;		/* 1ba04 */
	u32	port_wr_route;		/* 1ba08 */
	u32	__rsvd28;

	u32	port_wr_rx_stat;	/* 1ba10 */
	u32	port_wr_rx_event_gen;	/* 1ba14 */
	u32	__rsvd29[2];

	u32	port_wr_rx_capt[4];	/* 1ba20 - 1ba2c */
					/* 1ba30 - 1bcfc */
};

struct keystone_rio_link_layer_regs {
	u32	link_blk_hdr;		/* 1bd00 */
	u32	__rsvd31[8];		/* 1bd04 - 1bd20 */
	u32	whiteboard;		/* 1bd24 */
	u32	port_number;		/* 1bd28 */

	u32	__rsvd32;		/* 1bd2c */

	u32	prescalar_srv_clk;	/* 1bd30 */
	u32	reg_rst_ctl;		/* 1bd34 */
	u32	__rsvd33[4];		/* 1bd38, 1bd3c, 1bd40, 1bd44 */
	u32	local_err_det;		/* 1bd48 */
	u32	local_err_en;		/* 1bd4c */

	u32	local_h_addr_capt;	/* 1bd50 */
	u32	local_addr_capt;	/* 1bd54 */
	u32	local_id_capt;		/* 1bd58 */
	u32	local_ctrl_capt;	/* 1bd5c */

					/* 1bd60 - 1bdfc */
};

struct keystone_rio_fabric_regs {
	u32	fabric_hdr;		/* 1be00 */
	u32	__rsvd35[3];		/* 1be04 - 1be0c */

	u32	fabric_csr;		/* 1be10 */
	u32	__rsvd36[11];		/* 1be14 - 1be3c */

	u32	sp_fabric_status[4];	/* 1be40 - 1be4c */
};

/* Message Passing management functions */
int keystone_rio_open_outb_mbox(struct rio_mport *mport, void *dev_id,
				int mbox, int entries);

void keystone_rio_close_outb_mbox(struct rio_mport *mport, int mbox);

int keystone_rio_hw_add_outb_message(struct rio_mport *mport,
				     struct rio_dev *rdev, int mbox,
				     void *buffer, const size_t len);

int keystone_rio_open_inb_mbox(struct rio_mport *mport, void *dev_id,
			       int mbox, int entries);

void keystone_rio_close_inb_mbox(struct rio_mport *mport, int mbox);

int keystone_rio_hw_add_inb_buffer(struct rio_mport *mport,
				   int mbox, void *buffer);

void *keystone_rio_hw_get_inb_message(struct rio_mport *mport, int mbox);
#endif /* KEYSTONE_RIO_H */
