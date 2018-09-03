/*
 * AMD Cryptographic Coprocessor (CCP) driver
 *
 * Copyright (C) 2014,2016 Advanced Micro Devices, Inc.
 *
 * Author: Tom Lendacky <thomas.lendacky@amd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/ccp.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/acpi.h>

#include "ccp-dev.h"

struct ccp_platform {
	int coherent;
};

static const struct acpi_device_id ccp_acpi_match[];
static const struct of_device_id ccp_of_match[];

static struct sp_dev_vdata *ccp_get_of_version(struct platform_device *pdev)
{
#ifdef CONFIG_OF
	const struct of_device_id *match;

	match = of_match_node(ccp_of_match, pdev->dev.of_node);
	if (match && match->data)
		return (struct sp_dev_vdata *)match->data;
#endif
	return NULL;
}

static struct sp_dev_vdata *ccp_get_acpi_version(struct platform_device *pdev)
{
#ifdef CONFIG_ACPI
	const struct acpi_device_id *match;

	match = acpi_match_device(ccp_acpi_match, &pdev->dev);
	if (match && match->driver_data)
		return (struct sp_dev_vdata *)match->driver_data;
#endif
	return NULL;
}

static int ccp_get_irq(struct ccp_device *ccp)
{
	struct device *dev = ccp->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int ret;

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_notice(dev, "unable to get IRQ (%d)\n", ret);
		return ret;
	}

	ccp->irq = ret;
	ret = request_irq(ccp->irq, ccp->vdata->perform->irqhandler, 0,
			  ccp->name, ccp);
	if (ret) {
		dev_notice(dev, "unable to allocate IRQ (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int ccp_get_irqs(struct ccp_device *ccp)
{
	struct device *dev = ccp->dev;
	int ret;

	ret = ccp_get_irq(ccp);
	if (!ret)
		return 0;

	/* Couldn't get an interrupt */
	dev_notice(dev, "could not enable interrupts (%d)\n", ret);

	return ret;
}

static void ccp_free_irqs(struct ccp_device *ccp)
{
	free_irq(ccp->irq, ccp);
}

static int ccp_platform_probe(struct platform_device *pdev)
{
	struct sp_device *sp;
	struct ccp_platform *ccp_platform;
	struct device *dev = &pdev->dev;
	enum dev_dma_attr attr;
	struct resource *ior;
	int ret;

	ret = -ENOMEM;
	sp = sp_alloc_struct(dev);
	if (!sp)
		goto e_err;

	ccp_platform = devm_kzalloc(dev, sizeof(*ccp_platform), GFP_KERNEL);
	if (!ccp_platform)
		goto e_err;

	sp->dev_specific = ccp_platform;
	sp->dev_vdata = pdev->dev.of_node ? ccp_get_of_version(pdev)
					 : ccp_get_acpi_version(pdev);
	if (!sp->dev_vdata) {
		ret = -ENODEV;
		dev_err(dev, "missing driver data\n");
		goto e_err;
	}
	sp->get_irq = ccp_get_irqs;
	sp->free_irq = ccp_free_irqs;

	ior = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sp->io_map = devm_ioremap_resource(dev, ior);
	if (IS_ERR(sp->io_map)) {
		ret = PTR_ERR(sp->io_map);
		goto e_err;
	}

	attr = device_get_dma_attr(dev);
	if (attr == DEV_DMA_NOT_SUPPORTED) {
		dev_err(dev, "DMA is not supported");
		goto e_err;
	}

	ccp_platform->coherent = (attr == DEV_DMA_COHERENT);
	if (ccp_platform->coherent)
		sp->axcache = CACHE_WB_NO_ALLOC;
	else
		sp->axcache = CACHE_NONE;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(48));
	if (ret) {
		dev_err(dev, "dma_set_mask_and_coherent failed (%d)\n", ret);
		goto e_err;
	}

	dev_set_drvdata(dev, sp);

	ret = sp_init(sp);
	if (ret)
		goto e_err;

	dev_notice(dev, "enabled\n");

	return 0;

e_err:
	dev_notice(dev, "initialization failed\n");
	return ret;
}

static int ccp_platform_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sp_device *sp = dev_get_drvdata(dev);

	sp_destroy(sp);

	dev_notice(dev, "disabled\n");

	return 0;
}

#ifdef CONFIG_PM
static int ccp_platform_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct device *dev = &pdev->dev;
	struct sp_device *sp = dev_get_drvdata(dev);

	return sp_suspend(sp, state);
}

static int ccp_platform_resume(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sp_device *sp = dev_get_drvdata(dev);

	return sp_resume(sp);
}
#endif

static const struct sp_dev_vdata dev_vdata[] = {
	{
		.bar = 0,
#ifdef CONFIG_CRYPTO_DEV_SP_CCP
		.ccp_vdata = &ccpv3_platform,
#endif
	},
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id ccp_acpi_match[] = {
	{ "AMDI0C00", (kernel_ulong_t)&dev_vdata[0] },
	{ },
};
MODULE_DEVICE_TABLE(acpi, ccp_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id ccp_of_match[] = {
	{ .compatible = "amd,ccp-seattle-v1a",
	  .data = (const void *)&dev_vdata[0] },
	{ },
};
MODULE_DEVICE_TABLE(of, ccp_of_match);
#endif

static struct platform_driver ccp_platform_driver = {
	.driver = {
		.name = "ccp",
#ifdef CONFIG_ACPI
		.acpi_match_table = ccp_acpi_match,
#endif
#ifdef CONFIG_OF
		.of_match_table = ccp_of_match,
#endif
	},
	.probe = ccp_platform_probe,
	.remove = ccp_platform_remove,
#ifdef CONFIG_PM
	.suspend = ccp_platform_suspend,
	.resume = ccp_platform_resume,
#endif
};

int ccp_platform_init(void)
{
	return platform_driver_register(&ccp_platform_driver);
}

void ccp_platform_exit(void)
{
	platform_driver_unregister(&ccp_platform_driver);
}
