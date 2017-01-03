/*
 * Driver for MMC and SSD cards for Cavium ThunderX SOCs.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2016 Cavium Inc.
 * Authors: Jan Glauber <jglauber@cavium.com>
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/gpio/consumer.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/mmc/octeon_mmc.h>

#define DRV_NAME "thunderx_mmc"

static void thunder_mmc_acquire_bus(struct octeon_mmc_host *host)
{
	down(&host->mmc_serializer);
}

static void thunder_mmc_release_bus(struct octeon_mmc_host *host)
{
	up(&host->mmc_serializer);
}

static void thunder_mmc_int_enable(struct octeon_mmc_host *host, u64 val)
{
	writeq(val, host->base + OCT_MIO_EMM_INT);
	writeq(val, host->base + OCT_MIO_EMM_INT_EN_SET);
}

static int thunder_mmc_register_interrupts(struct octeon_mmc_host *host, struct pci_dev *pdev)
{
	int ret, i;

	host->msix_count = pci_msix_vec_count(pdev);
	host->mmc_msix = devm_kzalloc(&pdev->dev,
		(sizeof(struct msix_entry)) * host->msix_count, GFP_KERNEL);
	if (!host->mmc_msix)
                return -ENOMEM;

	for (i = 0; i < host->msix_count; i++)
		host->mmc_msix[i].entry = i;

	ret = pci_enable_msix(pdev, host->mmc_msix, host->msix_count);
	if (ret)
		return ret;

	/* register interrupts */
	for (i = 0; i < host->msix_count; i++) {
		ret = devm_request_irq(&pdev->dev, host->mmc_msix[i].vector,
				       octeon_mmc_interrupt,
				       0, DRV_NAME, host);
		if (ret)
			return ret;
	}
	return 0;
}

static int thunder_mmc_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	struct octeon_mmc_host *host;
	int ret = 0;
	struct device_node *child_node;
	u64 val;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	pci_set_drvdata(pdev, host);
	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	ret = pci_request_regions(pdev, DRV_NAME);
	if (ret)
		return ret;

	host->base = pcim_iomap(pdev, 0, pci_resource_len(pdev, 0));
	if (!host->base)
		return -EINVAL;
	/* On ThunderX these are identical */
	host->dma_base = host->base;

	host->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(host->clk))
                return PTR_ERR(host->clk);

        ret = clk_prepare_enable(host->clk);
        if (ret)
                return ret;
        host->sys_freq = clk_get_rate(host->clk);

	spin_lock_init(&host->irq_handler_lock);
	sema_init(&host->mmc_serializer, 1);

	host->acquire_bus = thunder_mmc_acquire_bus;
	host->release_bus = thunder_mmc_release_bus;
	host->int_enable = thunder_mmc_int_enable;

	host->big_dma_addr = true;
	host->need_irq_handler_lock = true;
	host->last_slot = -1;

	host->linear_buf_size = 1 << 22;
	host->linear_buf = devm_kzalloc(dev, host->linear_buf_size, GFP_KERNEL);
	if (!host->linear_buf)
		return -ENOMEM;

	/*
	 * Clear out any pending interrupts that may be left over from
	 * bootloader.
	 */
	val = readq(host->base + OCT_MIO_EMM_INT);
	writeq_relaxed(0, host->base + OCT_MIO_EMM_INT_EN);

	ret = thunder_mmc_register_interrupts(host, pdev);
	if (ret)
		return ret;

	host->global_pwr_gpiod = devm_gpiod_get_optional(&pdev->dev, "power",
							 GPIOD_OUT_LOW);
	if (IS_ERR(host->global_pwr_gpiod))
		return PTR_ERR(host->global_pwr_gpiod);

	for_each_child_of_node(node, child_node) {
		/*
		 * XXX hack: mmc_of_parse looks only at the current device's
		 * DT node. That means we require one device per slot with
		 * it's node pointing to the slot. The easiest way to get this
		 * is using of_platform_device_create. Not sure what a proper
		 * solution is, maybe extend mmc_of_parse to handle multiple
		 * slots? --jang
		 */
		struct platform_device *slot_pdev;

		slot_pdev = of_platform_device_create(child_node, NULL, &pdev->dev);
		ret = octeon_mmc_slot_probe(&slot_pdev->dev, host);
		if (ret) {
			gpiod_set_value_cansleep(host->global_pwr_gpiod, 0);
			return ret;
		}
	}
	dev_info(dev, "probed\n");
	return 0;
}

static void thunder_mmc_remove(struct pci_dev *pdev)
{
	struct octeon_mmc_host *host = pci_get_drvdata(pdev);
	union cvmx_mio_emm_dma_cfg dma_cfg;
	int i;

	for (i = 0; i < OCTEON_MAX_MMC; i++)
		if (host->slot[i])
			octeon_mmc_slot_remove(host->slot[i]);

	dma_cfg.u64 = readq(host->dma_base + OCT_MIO_EMM_DMA_CFG);
	dma_cfg.s.en = 0;
	writeq(dma_cfg.u64, host->dma_base + OCT_MIO_EMM_DMA_CFG);

	gpiod_set_value_cansleep(host->global_pwr_gpiod, 0);
}

static const struct pci_device_id thunder_mmc_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, 0xa010 ) },
	{ 0, }  /* end of table */
};

MODULE_AUTHOR("Cavium Inc.");
MODULE_DESCRIPTION("Cavium ThunderX eMMC Driver");
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pci, thunder_mmc_id_table);

static struct pci_driver thunder_mmc_driver = {
	.name = DRV_NAME,
	.id_table = thunder_mmc_id_table,
	.probe = thunder_mmc_probe,
	.remove = thunder_mmc_remove,
};

static int __init thunder_mmc_init_module(void)
{
	return pci_register_driver(&thunder_mmc_driver);
}

static void __exit thunder_mmc_exit_module(void)
{
	pci_unregister_driver(&thunder_mmc_driver);
}

module_init(thunder_mmc_init_module);
module_exit(thunder_mmc_exit_module);
