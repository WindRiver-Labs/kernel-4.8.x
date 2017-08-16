/*
 * Copyright (C) 2015, Broadcom Corporation. All Rights Reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#define USB2H_IDM_IDM_RESET_CONTROL__RESET 0
#define USB2H_IDM_IDM_IO_CONTROL_DIRECT__clk_enable 0
#define USB2H_IDM_IDM_IO_CONTROL_DIRECT_l1_sleepm_override     BIT(15)
#define USB2H_IDM_IDM_IO_CONTROL_DIRECT_l1_suspendm_override   BIT(14)
#define USB2H_IDM_IDM_IO_CONTROL_DIRECT_l2_suspendm_override   BIT(13)
#define USB2H_Ohci_Ehci_Strap__ohci_app_port_ovrcur_pol 11
#define USB2H_Ohci_Ehci_Strap__ppc_inversion 12
#define ICFG_USB2H_PHY_MISC_STATUS_PLLLOCK    0
#define USB2H_Phy_Ctrl_P0__Phy_Hard_Reset 9
#define USB2H_Phy_Ctrl_P0__Core_Reset 8
#define USB2H_Phy_Ctrl_P0__PHY_Soft_Reset 6
#define USB2H_Phy_Ctrl_P0__PHY_Test_port_Pwr_Dn 4
#define USB2H_Phy_Ctrl_P0__PHY_Test_port_UTMI_Pwr_Dn 2
#define USB2H_Phy_Ctrl_P0__PHY_PLL_Pwr_Dn 0
#define ICFG_FSM_MODE 2
#define ICFG_FSM_MODE_MASK 0x0000000C
#define ICFG_FSM_MODE_HOST 0x3

struct bcm_phy_ns2_usb2 {
	struct phy *phy;
	struct clk *clk;
	void __iomem *idm_reset_ctl;
	void __iomem *idm_io_ctl_direct;
	void __iomem *crmu_usb2_ctl;
	void __iomem *ohci_ehci_strap;
	void __iomem *phy_ctrl_p0;
	void __iomem *phy_misc_status;
	bool inv_ppc;
	uint32_t afe_corerdy_vddc;
};

static int ns2_usb2_phy_shutdown(struct phy *phy)
{
	struct bcm_phy_ns2_usb2 *iphy = phy_get_drvdata(phy);
	uint32_t reg_data;

	/* Phy bring up is done with USBH controller in reset */
	reg_data = readl(iphy->crmu_usb2_ctl);
	reg_data &= ~(1 << iphy->afe_corerdy_vddc);
	writel(reg_data, iphy->crmu_usb2_ctl);

	/* give hardware time to settle */
	udelay(100);

	/* Disable USBH controller clock */
	reg_data = readl(iphy->idm_io_ctl_direct);
	reg_data &= ~(1 << USB2H_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
	writel(reg_data, iphy->idm_io_ctl_direct);

	/* reset USBH controller */
	reg_data = readl(iphy->idm_reset_ctl);
	reg_data |= (1 << USB2H_IDM_IDM_RESET_CONTROL__RESET);
	writel(reg_data, iphy->idm_reset_ctl);

	return 0;
}

static int bcm_phy_init(struct phy *phy)
{
	uint32_t i, count = 100, reg_data;
	struct bcm_phy_ns2_usb2 *iphy = phy_get_drvdata(phy);
	bool dual_role = false;

	/* give hardware time to settle */
	udelay(100);

	/* reset USBH controller */
	reg_data = readl(iphy->idm_reset_ctl);
	reg_data |= (1 << USB2H_IDM_IDM_RESET_CONTROL__RESET);
	writel(reg_data, iphy->idm_reset_ctl);

	/* Disable USBH controller clock */
	reg_data = readl(iphy->idm_io_ctl_direct);
	reg_data &= ~(1 << USB2H_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
	writel(reg_data, iphy->idm_io_ctl_direct);

	/* Phy bring up is done with USBH controller in reset */
	reg_data = readl(iphy->crmu_usb2_ctl);
	reg_data |= (1 << iphy->afe_corerdy_vddc);
	writel(reg_data, iphy->crmu_usb2_ctl);

	i = 0;
	do {
		i++;
		reg_data = readl(iphy->phy_misc_status);
		if (i >= count) {
			dev_err(&phy->dev, "failed to get PLL lock\n");
			return -ETIMEDOUT;
		}
		udelay(100);
	} while (!(reg_data & (1 << ICFG_USB2H_PHY_MISC_STATUS_PLLLOCK)));

	/* USB Host clock enable */
	reg_data = readl(iphy->idm_io_ctl_direct);
	reg_data |= (1 << USB2H_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
	writel(reg_data, iphy->idm_io_ctl_direct);

	/* Enter reset */
	reg_data = readl(iphy->idm_reset_ctl);
	reg_data |= (1 << USB2H_IDM_IDM_RESET_CONTROL__RESET);
	writel(reg_data, iphy->idm_reset_ctl);

	/* Give hardware time to settle */
	udelay(100);

	/* Exit reset */
	reg_data &= ~(1 << USB2H_IDM_IDM_RESET_CONTROL__RESET);
	writel(reg_data, iphy->idm_reset_ctl);

	/* Give hardware time to settle */
	udelay(1000);

	/* Reverse over current polarity  */
	if (iphy->inv_ppc) {
		reg_data = readl(iphy->ohci_ehci_strap);
		reg_data |=
			(1 << USB2H_Ohci_Ehci_Strap__ohci_app_port_ovrcur_pol);
		if (!dual_role)
			reg_data |= (1 << USB2H_Ohci_Ehci_Strap__ppc_inversion);
		writel(reg_data, iphy->ohci_ehci_strap);
	}

	/* Pull these fields out of reset */
	writel(((1 << USB2H_Phy_Ctrl_P0__Phy_Hard_Reset) |
			(1 << USB2H_Phy_Ctrl_P0__Core_Reset) |
			(0x3 << USB2H_Phy_Ctrl_P0__PHY_Soft_Reset) |
			(0x3 << USB2H_Phy_Ctrl_P0__PHY_Test_port_Pwr_Dn) |
			(0x3 << USB2H_Phy_Ctrl_P0__PHY_Test_port_UTMI_Pwr_Dn) |
			(0x3 << USB2H_Phy_Ctrl_P0__PHY_PLL_Pwr_Dn)),
		iphy->phy_ctrl_p0);

       /*
        * USB PHY Sleep and Suspend override.
        * Single PHY is used for EHCI and OHCI controller.
        * If EHCI controller goes to suspend, PHY also enters to
        * Suspend. By that time OHCI controller might be still active.
        * If OHCI Controller tries to enters to suspend, kernel crash
        * is observed.
        * The following settings overwrites the Controller UTMI suspend and
        * sleep signals going to PHY.
        */
       reg_data = readl(iphy->idm_io_ctl_direct);
       reg_data |= USB2H_IDM_IDM_IO_CONTROL_DIRECT_l1_sleepm_override;
       reg_data |= USB2H_IDM_IDM_IO_CONTROL_DIRECT_l1_suspendm_override;
       reg_data |= USB2H_IDM_IDM_IO_CONTROL_DIRECT_l2_suspendm_override;
       writel(reg_data, iphy->idm_io_ctl_direct);

	return 0;
}

static struct phy_ops ops = {
	.init		= bcm_phy_init,
	.exit		= ns2_usb2_phy_shutdown,
};

static int bcm_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct bcm_phy_ns2_usb2 *iphy;
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	int ret = -ENODEV;

	iphy = devm_kzalloc(dev, sizeof(*iphy), GFP_KERNEL);
	if (!iphy) {
		ret = -ENOMEM;
		goto error1;
	}

	iphy->idm_reset_ctl = of_iomap(node, 0);
	if (!iphy->idm_reset_ctl) {
		dev_err(&pdev->dev, "can't iomap idm_reset_ctl\n");
		ret = -EIO;
		goto error2;
	}

	iphy->idm_io_ctl_direct = of_iomap(node, 1);
	if (!iphy->idm_io_ctl_direct) {
		dev_err(&pdev->dev, "can't iomap idm_io_ctl_direct\n");
		ret = -EIO;
		goto error3;
	}

	iphy->crmu_usb2_ctl = of_iomap(node, 2);
	if (!iphy->crmu_usb2_ctl) {
		dev_err(&pdev->dev, "can't iomap crmu_usb2_ctl\n");
		ret = -EIO;
		goto error4;
	}

	iphy->ohci_ehci_strap = of_iomap(node, 3);
	if (!iphy->ohci_ehci_strap) {
		dev_err(&pdev->dev, "can't iomap ohci_ehci_strap\n");
		ret = -EIO;
		goto error5;
	}

	iphy->phy_ctrl_p0 = of_iomap(node, 4);
	if (!iphy->phy_ctrl_p0) {
		dev_err(&pdev->dev, "can't iomap phy_ctrl_p0\n");
		ret = -EIO;
		goto error6;
	}

	iphy->phy_misc_status = of_iomap(node, 5);
	if (!iphy->phy_misc_status) {
		dev_err(&pdev->dev, "can't iomap phy_misc_status\n");
		ret = -EIO;
		goto error7;
	}

	ret = of_property_read_u32(node, "afe_corerdy_vddc",
				&iphy->afe_corerdy_vddc);
	if (ret != 0) {
		dev_err(&pdev->dev, "can't property_read afe_corerdy_vddc\n");
		ret = -EIO;
		goto error8;
	}

	if (of_property_read_bool(node, "reverse-ppc-polarity"))
		iphy->inv_ppc = true;

	iphy->phy = devm_phy_create(dev, dev->of_node, &ops);
	if (IS_ERR(iphy->phy)) {
		dev_err(dev, "Failed to create usb phy\n");
		ret = PTR_ERR(iphy->phy);
		goto error8;
	}
	phy_set_drvdata(iphy->phy, iphy);

	platform_set_drvdata(pdev, iphy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register as phy provider\n");
		ret = PTR_ERR(phy_provider);
		goto error8;
	}
	dev_info(dev, "Registered NS2 USB2H Phy\n");
	return 0;

error8:
	iounmap(iphy->phy_misc_status);
error7:
	iounmap(iphy->phy_ctrl_p0);
error6:
	iounmap(iphy->ohci_ehci_strap);
error5:
	iounmap(iphy->crmu_usb2_ctl);
error4:
	iounmap(iphy->idm_io_ctl_direct);
error3:
	iounmap(iphy->idm_reset_ctl);
error2:
	kfree(iphy);
error1:
	return ret;


}

static const struct of_device_id bcm_phy_dt_ids[] = {
	{ .compatible = "brcm,ns2-usb2-phy", },
	{ }
};

MODULE_DEVICE_TABLE(of, bcm_phy_dt_ids);

static struct platform_driver bcm_phy_driver = {
	.probe = bcm_phy_probe,
	.driver = {
		.name = "bcm-ns2-usb2phy",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bcm_phy_dt_ids),
	},
};

module_platform_driver(bcm_phy_driver);

MODULE_ALIAS("platform:bcm-ns2-usb2phy");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom USB2 PHY driver");
MODULE_LICENSE("GPL");

