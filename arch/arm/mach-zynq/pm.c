/*
 * Zynq power management
 *
 *  Copyright (C) 2012 - 2014 Xilinx
 *
 *  Sören Brinkmann <soren.brinkmann@xilinx.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <mach/slcr.h>
#include <asm/cacheflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/map.h>
#include <asm/suspend.h>
#include "common.h"

#define DDRC_CTRL_REG1_OFFS		0x60
#define DDRC_DRAM_PARAM_REG3_OFFS	0x20
#define SCU_CTRL			0
#define SLCR_TOPSW_CLK_CTRL		0x16c

#define DDRC_CLOCKSTOP_MASK	BIT(23)
#define DDRC_SELFREFRESH_MASK	BIT(12)
#define SCU_STBY_EN_MASK	BIT(5)
#define TOPSW_CLK_CTRL_DIS_MASK	BIT(0)

/* register offsets */
#define DDRC_CTRL_REG1_OFFS		0x60
#define DDRC_DRAM_PARAM_REG3_OFFS	0x20
#define SCU_CTRL			0
#define SLCR_TOPSW_CLK_CTRL		0x16c

/* bitfields */
#define DDRC_CLOCKSTOP_MASK	BIT(23)
#define DDRC_SELFREFRESH_MASK	BIT(12)
#define SCU_STBY_EN_MASK	BIT(5)
#define TOPSW_CLK_CTRL_DIS_MASK	BIT(0)

static struct clk *cpupll;
static void __iomem *ddrc_base;
static void __iomem *ocm_base;

static int zynq_pm_suspend(unsigned long arg)
{
	u32 reg;
	int (*zynq_suspend_ptr)(void __iomem *, void __iomem *);
	void *ocm_swap_area;
	int do_ddrpll_bypass = 1;

	/* Allocate some space for temporary OCM storage */
	ocm_swap_area = kmalloc(zynq_sys_suspend_sz, GFP_ATOMIC);
	if (!ocm_swap_area) {
		pr_warn("%s: cannot allocate memory to save portion of OCM\n",
				__func__);
		do_ddrpll_bypass = 0;
	}

	/* Enable DDR self-refresh and clock stop */
	if (ddrc_base) {
		reg = readl(ddrc_base + DDRC_CTRL_REG1_OFFS);
		reg |= DDRC_SELFREFRESH_MASK;
		writel(reg, ddrc_base + DDRC_CTRL_REG1_OFFS);

		reg = readl(ddrc_base + DDRC_DRAM_PARAM_REG3_OFFS);
		reg |= DDRC_CLOCKSTOP_MASK;
		writel(reg, ddrc_base + DDRC_DRAM_PARAM_REG3_OFFS);
	}

	/* SCU standby mode */
	if (scu_base) {
		reg = readl(scu_base + SCU_CTRL);
		reg |= SCU_STBY_EN_MASK;
		writel(reg, scu_base + SCU_CTRL);
	}

	/* Topswitch clock stop disable */
	reg = xslcr_read(SLCR_TOPSW_CLK_CTRL);
	reg |= TOPSW_CLK_CTRL_DIS_MASK;
	xslcr_write(reg, SLCR_TOPSW_CLK_CTRL);

	/* A9 clock gating */
	asm volatile ("mrc  p15, 0, r12, c15, c0, 0\n"
		      "orr  r12, r12, #1\n"
		      "mcr  p15, 0, r12, c15, c0, 0\n"
		      : /* no outputs */
		      : /* no inputs */
		      : "r12");


	/* Backup a small area of OCM used for the suspend code */
	memcpy(ocm_swap_area, (__force void *)ocm_base,
		zynq_sys_suspend_sz);

	/*
	 * Copy code to suspend system into OCM. The suspend code
	 * needs to run from OCM as DRAM may no longer be available
	 * when the PLL is stopped.
	 */
	memcpy((__force void *)ocm_base, &zynq_sys_suspend,
		zynq_sys_suspend_sz);
	flush_icache_range((unsigned long)ocm_base,
		(unsigned long)(ocm_base) + zynq_sys_suspend_sz);

	/*
	 * at this point PLLs are supposed to be bypassed:
	 *
	 * DDRPLL: Is bypassed without further sanity checking in the suspend
	 * routine which is called below and executed from OCM.
	 *
	 * IOPLL/ARMPLL: By now all clock consumers should have released their
	 * clock resulting in the PLLs to be bypassed. To account for timers and
	 * similar which run in the CPU clock domain we call a disable on the
	 * CPU clock's PLL to bypass it.
	 *
	 * A wake up device would prevent its source PLL from
	 * being bypassed, unless its the DDRPLL.
	 */
	if (!IS_ERR(cpupll))
		clk_disable(cpupll);

	/* Transfer to suspend code in OCM */
	zynq_suspend_ptr = (__force void *)ocm_base;
	flush_cache_all();
	if (ddrc_base && do_ddrpll_bypass) {
		/*
		 * Going this way will turn off DDR related clocks and the DDR
		 * PLL. I.e. We might brake sub systems relying on any of this
		 * clocks. And even worse: If there are any other masters in the
		 * system (e.g. in the PL) accessing DDR they are screwed.
		 */
		if (zynq_suspend_ptr(ddrc_base, zynq_slcr_base))
			pr_warn("DDR self refresh failed.\n");
	} else {
		wfi();
	}

	if (!IS_ERR(cpupll))
		clk_enable(cpupll);

	/* Restore original OCM contents */
	memcpy((__force void *)ocm_base, ocm_swap_area,
		zynq_sys_suspend_sz);

	kfree(ocm_swap_area);

	/* Topswitch clock stop disable */
	reg = xslcr_read(SLCR_TOPSW_CLK_CTRL);
	reg &= ~TOPSW_CLK_CTRL_DIS_MASK;
	xslcr_write(reg, SLCR_TOPSW_CLK_CTRL);

	/* SCU standby mode */
	if (scu_base) {
		reg = readl(scu_base + SCU_CTRL);
		reg &= ~SCU_STBY_EN_MASK;
		writel(reg, scu_base + SCU_CTRL);
	}

	/* A9 clock gating */
	asm volatile ("mrc  p15, 0, r12, c15, c0, 0\n"
		      "bic  r12, r12, #1\n"
		      "mcr  p15, 0, r12, c15, c0, 0\n"
		      : /* no outputs */
		      : /* no inputs */
		      : "r12");

	/* Disable DDR self-refresh and clock stop */
	if (ddrc_base) {
		reg = readl(ddrc_base + DDRC_CTRL_REG1_OFFS);
		reg &= ~DDRC_SELFREFRESH_MASK;
		writel(reg, ddrc_base + DDRC_CTRL_REG1_OFFS);

		reg = readl(ddrc_base + DDRC_DRAM_PARAM_REG3_OFFS);
		reg &= ~DDRC_CLOCKSTOP_MASK;
		writel(reg, ddrc_base + DDRC_DRAM_PARAM_REG3_OFFS);
	}

	return 0;
}

static int zynq_pm_enter(suspend_state_t suspend_state)
{
	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		outer_disable();
		cpu_suspend(0, zynq_pm_suspend);
		outer_resume();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct platform_suspend_ops zynq_pm_ops = {
	.enter		= zynq_pm_enter,
	.valid		= suspend_valid_only_mem,
};

/**
 * zynq_pm_ioremap() - Create IO mappings
 * @comp:	DT compatible string
 * Return: Pointer to the mapped memory or NULL.
 *
 * Remap the memory region for a compatible DT node.
 */
static void __iomem *zynq_pm_ioremap(const char *comp)
{
	struct device_node *np;
	void __iomem *base = NULL;

	np = of_find_compatible_node(NULL, NULL, comp);
	if (np) {
		base = of_iomap(np, 0);
		of_node_put(np);
	} else {
		pr_warn("%s: no compatible node found for '%s'\n", __func__,
				comp);
	}

	return base;
}

/**
 * zynq_pm_late_init() - Power management init
 *
 * Initialization of power management related features and infrastructure.
 */
void __init zynq_pm_late_init(void)
{
	u32 reg;

	ddrc_base = zynq_pm_ioremap("xlnx,zynq-ddrc-a05");
	if (!ddrc_base) {
		pr_warn("%s: Unable to map DDRC IO memory.\n", __func__);
	} else {
		/*
		 * Enable DDRC clock stop feature. The HW takes care of
		 * entering/exiting the correct mode depending
		 * on activity state.
		 */
		reg = readl(ddrc_base + DDRC_DRAM_PARAM_REG3_OFFS);
		reg |= DDRC_CLOCKSTOP_MASK;
		writel(reg, ddrc_base + DDRC_DRAM_PARAM_REG3_OFFS);
	}
}
