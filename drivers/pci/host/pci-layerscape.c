/*
 * PCIe host controller driver for Freescale Layerscape SoCs
 *
 * Copyright (C) 2014 - 2015 Freescale Semiconductor.
 *
 * Author: Minghuan Lian <Minghuan.Lian@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/init.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/resource.h>

#include "pcie-designware-base.h"
#include "pci-layerscape.h"

/* PEX Internal Configuration Registers */
#define PCIE_DBI_RO_WR_EN	0x8bc /* DBI Read-Only Write Enable Register */

struct ls_pcie_drvdata {
	u32 lut_offset;
	u32 ltssm_shift;
	struct pcie_host_ops *ops;
};

struct ls_pcie {
	struct dw_pcie_port	pp;
	void __iomem		*regs;
	void __iomem		*lut;
	struct regmap		*scfg;
	int			index;
	const u32 *avail_streamids;
	int streamid_index;
};

#define to_ls_pcie(x)	container_of(x, struct ls_pcie, pp)

u32 set_pcie_streamid_translation(struct pci_dev *pdev, u32 devid)
{
	u32 index, streamid;
	struct dw_pcie_port *pp = pdev->bus->sysdata;
	struct ls_pcie *pcie = to_ls_pcie(pp);

	if (!pcie->avail_streamids || !pcie->streamid_index)
		return ~(u32)0;

	index = --pcie->streamid_index;
	/* mask is set as all zeroes, want to match all bits */
	iowrite32((devid << 16), pcie->lut + PCIE_LUT_UDR(index));
	streamid = be32_to_cpup(&pcie->avail_streamids[index]);
	iowrite32(streamid | PCIE_LUT_ENABLE, pcie->lut + PCIE_LUT_LDR(index));

	return streamid;
}

static bool ls_pcie_is_bridge(struct ls_pcie *pcie)
{
	u32 header_type;

	header_type = ioread8(pcie->dbi + PCI_HEADER_TYPE);
	header_type &= 0x7f;

	return header_type == PCI_HEADER_TYPE_BRIDGE;
}

/* Clear multi-function bit */
static void ls_pcie_clear_multifunction(struct ls_pcie *pcie)
{
	iowrite8(PCI_HEADER_TYPE_BRIDGE, pcie->dbi + PCI_HEADER_TYPE);
}

/* Fix class value */
static void ls_pcie_fix_class(struct ls_pcie *pcie)
{
	iowrite16(PCI_CLASS_BRIDGE_PCI, pcie->dbi + PCI_CLASS_DEVICE);
}

/* Drop MSG TLP except for Vendor MSG */
static void ls_pcie_drop_msg_tlp(struct ls_pcie *pcie)
{
	u32 val;

	val = ioread32(pcie->dbi + PCIE_STRFMR1);
	val &= 0xDFFFFFFF;
	iowrite32(val, pcie->dbi + PCIE_STRFMR1);
}

static int ls1021_pcie_link_up(struct pcie_port *pp)
{
	u32 state;
	struct ls_pcie *pcie = to_ls_pcie(pp);

	if (!pcie->scfg)
		return 0;

	regmap_read(pcie->scfg, SCFG_PEXMSCPORTSR(pcie->index), &state);
	state = (state >> LTSSM_STATE_SHIFT) & LTSSM_STATE_MASK;

	if (state < LTSSM_PCIE_L0)
		return 0;

	return 1;
}

static void ls1021_pcie_host_init(struct pcie_port *pp)
{
	struct ls_pcie *pcie = to_ls_pcie(pp);
	u32 index[2];

	pcie->scfg = syscon_regmap_lookup_by_phandle(pp->dev->of_node,
						     "fsl,pcie-scfg");
	if (IS_ERR(pcie->scfg)) {
		dev_err(pp->dev, "No syscfg phandle specified\n");
		pcie->scfg = NULL;
		return;
	}

	if (of_property_read_u32_array(pp->dev->of_node,
				       "fsl,pcie-scfg", index, 2)) {
		pcie->scfg = NULL;
		return;
	}
	pcie->index = index[1];

	dw_pcie_setup_rc(pp);

	ls_pcie_drop_msg_tlp(pcie);
}

static int ls1_pcie_link_up(struct dw_pcie_port *pp)
{
	struct ls_pcie *pcie = to_ls_pcie(pp);
	u32 state;

	if (!pcie->scfg)
		return 0;

	state = (ioread32(pcie->lut + PCIE_LUT_DBG) >>
		 pcie->drvdata->ltssm_shift) &
		 LTSSM_STATE_MASK;

	if (state < LTSSM_PCIE_L0)
		return 0;

	return 1;
}

static int ls1_pcie_host_init(struct dw_pcie_port *pp)
{
	struct ls_pcie *pcie = to_ls_pcie(pp);
	u32 val, index[2];
	int ret;

	pcie->scfg = syscon_regmap_lookup_by_phandle(pp->dev->of_node,
						     "fsl,pcie-scfg");
	if (IS_ERR(pcie->scfg)) {
		dev_err(pp->dev, "No syscfg phandle specified\n");
		return PTR_ERR(pcie->scfg);
 	}
 
	ret = of_property_read_u32_array(pp->dev->of_node,
					 "fsl,pcie-scfg", index, 2);
	if (ret)
		return ret;

	pcie->index = index[1];

 	/*
 	 * LS1021A Workaround for internal TKT228622
 	 * to fix the INTx hang issue
 	 */
	val = dw_pcie_dbi_read(pp, PCIE_SYMBOL_TIMER_1);
 	val &= 0xffff;
	dw_pcie_dbi_write(pp, val, PCIE_SYMBOL_TIMER_1);

	/* Fix class value */
	val = dw_pcie_dbi_read(pp, PCI_CLASS_REVISION);
	val = (val & 0x0000ffff) | (PCI_CLASS_BRIDGE_PCI << 16);
	dw_pcie_dbi_write(pp, val, PCI_CLASS_REVISION);

	if (!ls1_pcie_link_up(pp))
		dev_err(pp->dev, "phy link never came up\n");

	return 0;
}

static int ls_pcie_msi_host_init(struct pcie_port *pp,
				 struct msi_controller *chip)
{
	struct device_node *msi_node;
	struct device_node *np = pp->dev->of_node;

	/*
	 * The MSI domain is set by the generic of_msi_configure().  This
	 * .msi_host_init() function keeps us from doing the default MSI
	 * domain setup in dw_pcie_host_init() and also enforces the
	 * requirement that "msi-parent" exists.
	 */
	msi_node = of_parse_phandle(np, "msi-parent", 0);
	if (!msi_node) {
		dev_err(pp->dev, "failed to find msi-parent\n");
		return -EINVAL;
	}

	return 0;
}

static struct pcie_host_ops ls1021_pcie_host_ops = {
	.link_up = ls1021_pcie_link_up,
	.host_init = ls1021_pcie_host_init,
	.msi_host_init = ls_pcie_msi_host_init,
};

static struct dw_host_ops ls1_dw_host_ops = {
	.link_up = ls1_pcie_link_up,
	.host_init = ls1_pcie_host_init,
};

static struct ls_pcie_drvdata ls1021_drvdata = {
	.ops = &ls1021_pcie_host_ops,
};

static struct ls_pcie_drvdata ls1043_drvdata = {
	.lut_offset = 0x10000,
	.ltssm_shift = 24,
	.ops = &ls_pcie_host_ops,
};

static struct ls_pcie_drvdata ls2080_drvdata = {
	.lut_offset = 0x80000,
	.ltssm_shift = 0,
	.ops = &ls_pcie_host_ops,
};

static const struct of_device_id ls_pcie_of_match[] = {
	{ .compatible = "fsl,ls1021a-pcie", .data = &ls1021_drvdata },
	{ .compatible = "fsl,ls1043a-pcie", .data = &ls1043_drvdata },
	{ .compatible = "fsl,ls2080a-pcie", .data = &ls2080_drvdata },
	{ .compatible = "fsl,ls2085a-pcie", .data = &ls2080_drvdata },
	{ },
};

static int ls2_pcie_link_up(struct dw_pcie_port *pp)
{
	struct ls_pcie *pcie = to_ls_pcie(pp);
	u32 state;

	if (!pcie->lut)
		return 0;

	state = ioread32(pcie->lut + PCIE_LUT_DBG) & LTSSM_STATE_MASK;
	if (state < LTSSM_PCIE_L0)
		return 0;

	return 1;
}

static int ls2_pcie_host_init(struct dw_pcie_port *pp)
{
	struct ls_pcie *pcie = to_ls_pcie(pp);
	u32 val;

	pcie->lut = pp->dbi + PCIE_LUT_BASE;
	/* Disable LDR zero */
	iowrite32(0, pcie->lut + PCIE_LUT_LDR(0));

	dw_pcie_dbi_write(pp, 1, PCIE_DBI_RO_WR_EN);
	/* Fix class value */
	val = dw_pcie_dbi_read(pp, PCI_CLASS_REVISION);
	val = (val & 0x0000ffff) | (PCI_CLASS_BRIDGE_PCI << 16);
	dw_pcie_dbi_write(pp, val, PCI_CLASS_REVISION);
	/* clean multi-func bit */
	val = dw_pcie_dbi_read(pp, PCI_HEADER_TYPE & ~0x3);
	val &= ~(1 << 23);
	dw_pcie_dbi_write(pp, val, PCI_HEADER_TYPE & ~0x3);
	dw_pcie_dbi_write(pp, 0, PCIE_DBI_RO_WR_EN);

	if (!ls2_pcie_link_up(pp))
		dev_err(pp->dev, "phy link never came up\n");
 
	return 0;
}
 
static struct dw_host_ops ls2_dw_host_ops = {
	.link_up = ls2_pcie_link_up,
	.host_init = ls2_pcie_host_init,
};

static int __init ls_pcie_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct ls_pcie *pcie;
	struct resource *res;
	int ret;

	match = of_match_device(ls_pcie_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	pcie = devm_kzalloc(&pdev->dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	pcie->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pcie->regs)) {
		dev_err(&pdev->dev, "missing *regs* space\n");
		return PTR_ERR(pcie->regs);
 	}

	pcie->lut = pcie->regs + PCIE_LUT_BASE;
	/* Disable LDR zero */
	iowrite32(0, pcie->lut + PCIE_LUT_LDR(0));
	pcie->pp.dev = &pdev->dev;
	pcie->pp.dbi = pcie->regs;
	pcie->pp.dw_ops = (struct dw_host_ops *)match->data;
	pcie->pp.atu_num = PCIE_ATU_NUM;

	if (of_device_is_compatible(pdev->dev.of_node, "fsl,ls2085a-pcie") ||
	of_device_is_compatible(pdev->dev.of_node, "fsl,ls2080a-pcie")) {
		int len;
		const u32 *prop;
		struct device_node *np;

		np = pdev->dev.of_node;
		prop = (u32 *)of_get_property(np, "available-stream-ids", &len);
		if (prop) {
			pcie->avail_streamids = prop;
			pcie->streamid_index = len/sizeof(u32);
		} else
			dev_err(&pdev->dev, "PCIe endpoint partitioning not possible\n");
	}

	if (!ls_pcie_is_bridge(pcie))
		return -ENODEV;

	ret = dw_pcie_port_init(&pcie->pp);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, pcie);

	return 0;
}

static struct platform_driver ls_pcie_driver = {
	.driver = {
		.name = "layerscape-pcie",
		.of_match_table = ls_pcie_of_match,
	},
};
builtin_platform_driver_probe(ls_pcie_driver, ls_pcie_probe);
