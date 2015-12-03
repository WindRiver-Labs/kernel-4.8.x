/*
 * Copyright 2013-2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/mach/arch.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/phy_fixed.h>
#include "common.h"

#ifdef CONFIG_FIXED_PHY
static int __init of_add_fixed_phys(void)
{
	int ret;
	struct device_node *np;
	u32 *fixed_link;
	struct fixed_phy_status status = {};

	for_each_node_by_name(np, "ethernet") {
		fixed_link  = (u32 *)of_get_property(np, "fixed-link", NULL);
		if (!fixed_link)
			continue;

		status.link = 1;
		status.duplex = be32_to_cpu(fixed_link[1]);
		status.speed = be32_to_cpu(fixed_link[2]);
		status.pause = be32_to_cpu(fixed_link[3]);
		status.asym_pause = be32_to_cpu(fixed_link[4]);

		ret = fixed_phy_add(PHY_POLL, be32_to_cpu(fixed_link[0]), &status, -1);
		if (ret) {
			of_node_put(np);
			return ret;
		}
	}

	return 0;
}
arch_initcall(of_add_fixed_phys);
#endif /* CONFIG_FIXED_PHY */

static const char * const ls1021a_dt_compat[] __initconst = {
	"fsl,ls1021a",
	NULL,
};

DT_MACHINE_START(LS1021A, "Freescale LS1021A")
	.smp		= smp_ops(ls1021a_smp_ops),
	.dt_compat	= ls1021a_dt_compat,
MACHINE_END
