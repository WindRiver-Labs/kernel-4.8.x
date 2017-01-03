/*
 * Driver for MMC and SSD cards for Cavium OCTEON SOCs.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012-2016 Cavium Inc.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/gpio/consumer.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/mmc/octeon_mmc.h>

#include <asm/octeon/octeon.h>

#define DRV_NAME	"octeon_mmc"

#define CVMX_MIO_BOOT_CTL      CVMX_ADD_IO_SEG(0x00011800000000D0ull)

extern void l2c_lock_mem_region(u64 start, u64 len);
extern void l2c_unlock_mem_region(u64 start, u64 len);

static void octeon_mmc_acquire_bus(struct octeon_mmc_host *host)
{
	if (!host->has_ciu3) {
		/* Switch the MMC controller onto the bus. */
		down(&octeon_bootbus_sem);
		writeq(0, (void __iomem *)CVMX_MIO_BOOT_CTL);
	} else {
		down(&host->mmc_serializer);
	}
}

static void octeon_mmc_release_bus(struct octeon_mmc_host *host)
{
	if (!host->has_ciu3)
		up(&octeon_bootbus_sem);
	else
		up(&host->mmc_serializer);
}

static void octeon_mmc_int_enable(struct octeon_mmc_host *host, u64 val)
{
	writeq(val, host->base + OCT_MIO_EMM_INT);
	if (!host->dma_active || (host->dma_active && !host->has_ciu3))
		writeq(val, host->base + OCT_MIO_EMM_INT_EN);
}

static void octeon_mmc_dmar_fixup(struct octeon_mmc_host *host,
				  struct mmc_command *cmd,
				  struct mmc_data *data,
				  u64 addr)
{
	if (!OCTEON_IS_MODEL(OCTEON_CN6XXX) &&
	    !OCTEON_IS_MODEL(OCTEON_CNF7XXX))
		return;

	if (cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK &&
	    (data->blksz * data->blocks) > 1024) {
		host->n_minus_one = addr + (data->blksz * data->blocks) - 1024;
		host->lock_region(host->n_minus_one, 512);
	}
}

static int octeon_mmc_probe(struct platform_device *pdev)
{
	struct octeon_mmc_host *host;
	struct resource	*res;
	void __iomem *base;
	int mmc_irq[9];
	int i;
	int ret = 0;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *cn;
	u64 t;

	host = devm_kzalloc(&pdev->dev, sizeof(*host), GFP_KERNEL);
	if (!host) {
		dev_err(&pdev->dev, "devm_kzalloc failed\n");
		return -ENOMEM;
	}

	spin_lock_init(&host->irq_handler_lock);
	sema_init(&host->mmc_serializer, 1);

	host->acquire_bus = octeon_mmc_acquire_bus;
	host->release_bus = octeon_mmc_release_bus;
	host->lock_region = l2c_lock_mem_region;
	host->unlock_region = l2c_unlock_mem_region;
	host->dmar_fixup = octeon_mmc_dmar_fixup;

	host->sys_freq = octeon_get_io_clock_rate();

	if (of_device_is_compatible(node, "cavium,octeon-7890-mmc")) {
		host->big_dma_addr = true;
		host->need_irq_handler_lock = true;
		host->has_ciu3 = true;
		/*
		 * First seven are the EMM_INT bits 0..6, then two for
		 * the EMM_DMA_INT bits
		 */
		for (i = 0; i < 9; i++) {
			mmc_irq[i] = platform_get_irq(pdev, i);
			if (mmc_irq[i] < 0)
				return mmc_irq[i];

			/* work around legacy u-boot device trees */
			irq_set_irq_type(mmc_irq[i], IRQ_TYPE_EDGE_RISING);
		}
	} else {
		host->big_dma_addr = false;
		host->need_irq_handler_lock = false;
		host->has_ciu3 = false;
		/* First one is EMM second DMA */
		for (i = 0; i < 2; i++) {
			mmc_irq[i] = platform_get_irq(pdev, i);
			if (mmc_irq[i] < 0)
				return mmc_irq[i];
		}
	}
	host->last_slot = -1;

	/* 256KB DMA linearized buffer (maximum transfer size). */
	host->linear_buf_size = (1 << 18);
	host->linear_buf = devm_kzalloc(&pdev->dev, host->linear_buf_size,
					GFP_KERNEL);

	if (!host->linear_buf) {
		dev_err(&pdev->dev, "devm_kzalloc failed\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Platform resource[0] is missing\n");
		return -ENXIO;
	}
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	host->base = (void __iomem *)base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "Platform resource[1] is missing\n");
		return -EINVAL;
	}
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	host->dma_base = (void __iomem *)base;

	/*
	 * Clear out any pending interrupts that may be left over from
	 * bootloader.
	 */
	t = readq(host->base + OCT_MIO_EMM_INT);
	writeq(t, host->base + OCT_MIO_EMM_INT);
	if (host->has_ciu3) {
		/* Only CMD_DONE, DMA_DONE, CMD_ERR, DMA_ERR */
		for (i = 1; i <= 4; i++) {
			ret = devm_request_irq(&pdev->dev, mmc_irq[i],
					       octeon_mmc_interrupt,
					       0, DRV_NAME, host);
			if (ret < 0) {
				dev_err(&pdev->dev, "Error: devm_request_irq %d\n",
					mmc_irq[i]);
				return ret;
			}
		}
	} else {
		ret = devm_request_irq(&pdev->dev, mmc_irq[0],
				       octeon_mmc_interrupt, 0, DRV_NAME, host);
		if (ret < 0) {
			dev_err(&pdev->dev, "Error: devm_request_irq %d\n",
				mmc_irq[0]);
			return ret;
		}
	}

	host->global_pwr_gpiod = devm_gpiod_get_optional(&pdev->dev, "power",
								GPIOD_OUT_HIGH);
	if (IS_ERR(host->global_pwr_gpiod)) {
		dev_err(&pdev->dev, "Invalid POWER GPIO\n");
		return PTR_ERR(host->global_pwr_gpiod);
	}

	platform_set_drvdata(pdev, host);

	for_each_child_of_node(node, cn) {
		struct platform_device *slot_pdev;

		slot_pdev = of_platform_device_create(cn, NULL, &pdev->dev);
		ret = octeon_mmc_slot_probe(&slot_pdev->dev, host);
		if (ret) {
			dev_err(&pdev->dev, "Error populating slots\n");
			gpiod_set_value_cansleep(host->global_pwr_gpiod, 0);
			return ret;
		}
	}

	return 0;
}

static int octeon_mmc_remove(struct platform_device *pdev)
{
	union cvmx_mio_emm_dma_cfg emm_dma_cfg;
	struct octeon_mmc_host *host = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < OCTEON_MAX_MMC; i++) {
		if (host->slot[i])
			octeon_mmc_slot_remove(host->slot[i]);
	}

	emm_dma_cfg.u64 = readq(host->dma_base + OCT_MIO_EMM_DMA_CFG);
	emm_dma_cfg.s.en = 0;
	writeq(emm_dma_cfg.u64, host->dma_base + OCT_MIO_EMM_DMA_CFG);

	gpiod_set_value_cansleep(host->global_pwr_gpiod, 0);

	return 0;
}

static const struct of_device_id octeon_mmc_match[] = {
	{
		.compatible = "cavium,octeon-6130-mmc",
	},
	{
		.compatible = "cavium,octeon-7890-mmc",
	},
	{},
};
MODULE_DEVICE_TABLE(of, octeon_mmc_match);

static struct platform_driver octeon_mmc_driver = {
	.probe		= octeon_mmc_probe,
	.remove		= octeon_mmc_remove,
	.driver		= {
		.name	= DRV_NAME,
		.of_match_table = octeon_mmc_match,
	},
};

static int __init octeon_mmc_init(void)
{
	return platform_driver_register(&octeon_mmc_driver);
}

static void __exit octeon_mmc_cleanup(void)
{
	platform_driver_unregister(&octeon_mmc_driver);
}

module_init(octeon_mmc_init);
module_exit(octeon_mmc_cleanup);

MODULE_AUTHOR("Cavium Inc. <support@cavium.com>");
MODULE_DESCRIPTION("low-level driver for Cavium OCTEON MMC/SSD card");
MODULE_LICENSE("GPL");
