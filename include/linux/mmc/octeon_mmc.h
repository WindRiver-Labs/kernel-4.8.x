/*
 * Driver for MMC and SSD cards for Cavium OCTEON SOCs.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012-2016 Cavium Inc.
 */
#include <linux/io.h>
#include <linux/scatterlist.h>
#include <linux/mmc/host.h>

#include <asm/octeon/octeon.h>

#define OCTEON_MAX_MMC			4

#define OCT_MIO_NDF_DMA_CFG		0x00
#define OCT_MIO_EMM_DMA_ADR		0x08

#define OCT_MIO_EMM_CFG			0x00
#define OCT_MIO_EMM_SWITCH		0x48
#define OCT_MIO_EMM_DMA			0x50
#define OCT_MIO_EMM_CMD			0x58
#define OCT_MIO_EMM_RSP_STS		0x60
#define OCT_MIO_EMM_RSP_LO		0x68
#define OCT_MIO_EMM_RSP_HI		0x70
#define OCT_MIO_EMM_INT			0x78
#define OCT_MIO_EMM_INT_EN		0x80
#define OCT_MIO_EMM_WDOG		0x88
#define OCT_MIO_EMM_SAMPLE		0x90
#define OCT_MIO_EMM_STS_MASK		0x98
#define OCT_MIO_EMM_RCA			0xa0
#define OCT_MIO_EMM_BUF_IDX		0xe0
#define OCT_MIO_EMM_BUF_DAT		0xe8

struct octeon_mmc_host {
	void __iomem	*base;
	void __iomem	*ndf_base;
	u64	emm_cfg;
	u64	n_minus_one;  /* OCTEON II workaround location */
	int	last_slot;

	struct semaphore mmc_serializer;
	struct mmc_request	*current_req;
	unsigned int		linear_buf_size;
	void			*linear_buf;
	struct sg_mapping_iter smi;
	int sg_idx;
	bool dma_active;

	struct platform_device	*pdev;
	struct gpio_desc *global_pwr_gpiod;
	bool dma_err_pending;
	bool big_dma_addr;
	bool need_irq_handler_lock;
	spinlock_t irq_handler_lock;
	bool has_ciu3;

	struct octeon_mmc_slot	*slot[OCTEON_MAX_MMC];
};

struct octeon_mmc_slot {
	struct mmc_host         *mmc;	/* slot-level mmc_core object */
	struct octeon_mmc_host	*host;	/* common hw for all 4 slots */

	unsigned int		clock;
	unsigned int		sclock;

	u64			cached_switch;
	u64			cached_rca;

	unsigned int		cmd_cnt; /* sample delay */
	unsigned int		dat_cnt; /* sample delay */

	int			bus_width;
	int			bus_id;
};

struct octeon_mmc_cr_type {
	u8 ctype;
	u8 rtype;
};

struct octeon_mmc_cr_mods {
	u8 ctype_xor;
	u8 rtype_xor;
};

extern void l2c_lock_mem_region(u64 start, u64 len);
extern void l2c_unlock_mem_region(u64 start, u64 len);
extern void octeon_mmc_acquire_bus(struct octeon_mmc_host *host);
extern void octeon_mmc_release_bus(struct octeon_mmc_host *host);
