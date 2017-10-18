/*
 * OMAP IOMMU quirks for various TI SoCs
 *
 * Copyright (C) 2015-2017 Texas Instruments Incorporated - http://www.ti.com/
 *      Suman Anna <s-anna@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/err.h>

#include "omap_hwmod.h"
#include "omap_device.h"
#include "powerdomain.h"

int omap_iommu_set_pwrdm_constraint(struct platform_device *pdev, bool request,
				    u8 *pwrst)
{
	struct powerdomain *pwrdm;
	struct omap_device *od;
	u8 next_pwrst;

	od = to_omap_device(pdev);
	if (!od)
		return -ENODEV;

	if (od->hwmods_cnt != 1)
		return -EINVAL;

	pwrdm = omap_hwmod_get_pwrdm(od->hwmods[0]);
	if (!pwrdm)
		return -EINVAL;

	if (request)
		*pwrst = pwrdm_read_next_pwrst(pwrdm);

	if (*pwrst > PWRDM_POWER_RET)
		return 0;

	next_pwrst = request ? PWRDM_POWER_ON : *pwrst;

	return pwrdm_set_next_pwrst(pwrdm, next_pwrst);
}
