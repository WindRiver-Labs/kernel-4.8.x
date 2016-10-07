/*
 * Copyright 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/arm-smccc.h>

#define REG_BIT(x)		(1 << x)
#define RTC_REG_PERIO_INTR	REG_BIT(0)
#define RTC_REG_ALARM_INTR	REG_BIT(1)
#define RTC_REG_RAW_TAMP_INTR	REG_BIT(8)

enum rtc_smc_ids {
	OEM_RTC_BBL_INIT = 0,
	OEM_RTC_CONTROL,
	OEM_RTC_TIME_REQ,
	OEM_RTC_ALARM_CONTROL,
	OEM_RTC_INTR_STAT_CONTROL,
};

enum oem_rtc_control {
	OEM_RTC_CONTROL_DISABLE = 0,
	OEM_RTC_CONTROL_ENABLE,
};

enum oem_rtc_time_req {
	OEM_RTC_TIME_REQ_READ = 0,
	OEM_RTC_TIME_REQ_SET,
};

enum oem_rtc_alarm_control {
	OEM_RTC_ALARM_CONTROL_READ = 0,
	OEM_RTC_ALARM_CONTROL_SET,
	OEM_RTC_ALARM_CONTROL_INTR_ENABLE,
	OEM_RTC_ALARM_CONTROL_INTR_DISABLE,
	OEM_RTC_ALARM_CONTROL_INTR_READ,
};

enum oem_rtc_intr_stat_control {
	OEM_RTC_INTR_STAT_CONTROL_CLR_PEROI = 0,
	OEM_RTC_INTR_STAT_CONTROL_CLR_ALARM,
	OEM_RTC_INTR_STAT_CONTROL_READ,
	OEM_RTC_INTR_STAT_CONTROL_CLR_RAW_TAMP,
};

struct smc_data {
	/*
	 * fid, arg1, arg2 will be used for passing function id and
	 * function argument to smc work function and the result
	 * of smc call will be stored in res after execution.
	 */
	u64 fid;
	u64 arg1;
	u64 arg2;
	struct arm_smccc_res res;
};

struct bcm_rtc_t {
	struct rtc_device  *rtc;
	struct device dev;
	struct rtc_class_ops *rtc_ops;
	int periodic_irq;
	int alarm_irq;
	spinlock_t lock; /* to use inside interrupt handler */
	struct mutex mut_lock;
	struct smc_data smc_fn_data;
};

static int bbl_rtc_smc_ids[5];
static struct bcm_rtc_t bcm_rtc;

/*
 * smc calls from secodary cpu cores will produce undefined exception, Do not
 * call this function directly on smp systems. Use smp_call_function_single()
 * or other similar method to execute it on cpu core 0 only.
 */
static void __bcm_smc_internal(void *info)
{
	struct smc_data *data = (struct smc_data *)info;

	arm_smccc_smc(data->fid, data->arg1, data->arg2, 0, 0, 0, 0, 0,
		      &data->res);
}

static int bcm_smc_internal(u64 fid, u64 farg1, u64 farg2, u32 *smc_retdata)
{
	int ret;
	struct smc_data data;

	if (mutex_lock_interruptible(&bcm_rtc.mut_lock))
		return -ERESTARTSYS;

	data.fid = fid;
	data.arg1 = farg1;
	data.arg2 = farg2;
	data.res.a1 = 0;
	ret = smp_call_function_single(0, __bcm_smc_internal, &data, true);
	if (ret < 0) {
		dev_err(&bcm_rtc.dev, "smp_call_function_single() failed\n");
		goto err;
	}
	ret = data.res.a0; /* value returned from smc call */
	if (smc_retdata)
		*smc_retdata = data.res.a1;

err:
	mutex_unlock(&bcm_rtc.mut_lock);
	return ret;
}

static noinline int bbl_init(void)
{
	return bcm_smc_internal(bbl_rtc_smc_ids[OEM_RTC_BBL_INIT],
				  0, 0, NULL);
}

static int bcm_rtc_enable(void)
{
	return bcm_smc_internal(bbl_rtc_smc_ids[OEM_RTC_CONTROL],
				  OEM_RTC_CONTROL_ENABLE, 0, NULL);
}

static int bcm_rtc_disable(void)
{
	return bcm_smc_internal(bbl_rtc_smc_ids[OEM_RTC_CONTROL],
				  OEM_RTC_CONTROL_DISABLE, 0, NULL);
}

static irqreturn_t bcm_rtc_interrupt(int irq, void *class_dev)
{
	unsigned long events = 0;
	u32 irq_flg;
	unsigned long flags;
	int ret;
	struct arm_smccc_res res;

	spin_lock_irqsave(&bcm_rtc.lock, flags);

	arm_smccc_smc(bbl_rtc_smc_ids[OEM_RTC_INTR_STAT_CONTROL],
		      OEM_RTC_INTR_STAT_CONTROL_READ, 0, 0, 0, 0, 0, 0, &res);
	ret = res.a0;
	if (ret < 0) {
		dev_err(&bcm_rtc.dev, "couldn't read intr stat\n");
		goto err;
	}

	irq_flg = res.a1;
	irq_flg &= RTC_REG_PERIO_INTR | RTC_REG_ALARM_INTR;
	if (!irq_flg)
		goto err;

	if (irq_flg & RTC_REG_PERIO_INTR) {
		/*Clear periodic interrupt status*/
		arm_smccc_smc(bbl_rtc_smc_ids[OEM_RTC_INTR_STAT_CONTROL],
			      OEM_RTC_INTR_STAT_CONTROL_CLR_PEROI, 0, 0, 0, 0,
			      0, 0, &res);
		ret = res.a0;
		if (ret < 0)
			dev_err(&bcm_rtc.dev, "couldn't clear periodic intr\n");

		events |= RTC_IRQF | RTC_PF;
	}

	if (irq_flg & RTC_REG_ALARM_INTR) {
		/*Clear alarm interrupt status*/
		arm_smccc_smc(bbl_rtc_smc_ids[OEM_RTC_INTR_STAT_CONTROL],
			      OEM_RTC_INTR_STAT_CONTROL_CLR_ALARM, 0, 0, 0, 0,
			      0, 0, &res);
		ret = res.a0;
		if (ret < 0)
			dev_err(&bcm_rtc.dev, "couldn't clear alarm intr\n");

		events |= RTC_IRQF | RTC_AF;
		arm_smccc_smc(bbl_rtc_smc_ids[OEM_RTC_ALARM_CONTROL],
			      OEM_RTC_ALARM_CONTROL_INTR_DISABLE, 0, 0, 0, 0,
			      0, 0, &res);
		ret = res.a0;
		if (ret < 0) {
			dev_err(&bcm_rtc.dev, "couldn't disable alarm intr\n");
			goto err;
		}
	}

err:
	if (events)
		rtc_update_irq(bcm_rtc.rtc, 1, events);

	spin_unlock_irqrestore(&bcm_rtc.lock, flags);
	return IRQ_HANDLED;
}

static int bcm_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int ret;
	unsigned int seconds;

	ret = bcm_smc_internal(bbl_rtc_smc_ids[OEM_RTC_TIME_REQ],
				 OEM_RTC_TIME_REQ_READ, 0, &seconds);
	if (ret < 0)
		goto err;

	rtc_time_to_tm(seconds, tm);
	ret = rtc_valid_tm(tm);

err:
	if (ret < 0)
		dev_err(&bcm_rtc.dev, "couldn't read time\n");
	return ret;
}

static int bcm_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int ret;
	unsigned long t;

	rtc_tm_to_time(tm, &t);
	ret = bcm_smc_internal(bbl_rtc_smc_ids[OEM_RTC_TIME_REQ],
				 OEM_RTC_TIME_REQ_SET, t, NULL);
	if (ret < 0)
		dev_err(&bcm_rtc.dev, "couldn't set time\n");

	return ret;
}

static int bcm_rtc_alarm_irq_enable(struct device *dev,
				      unsigned int enabled)
{
	int ret;

	if (enabled)
		ret = bcm_smc_internal(
				bbl_rtc_smc_ids[OEM_RTC_ALARM_CONTROL],
				OEM_RTC_ALARM_CONTROL_INTR_ENABLE,
				0, NULL);
	else
		ret = bcm_smc_internal(
				bbl_rtc_smc_ids[OEM_RTC_ALARM_CONTROL],
				OEM_RTC_ALARM_CONTROL_INTR_DISABLE,
				0, NULL);

	if (ret < 0)
		dev_err(&bcm_rtc.dev, "couldn't %s alarm intr\n",
			enabled ? "enable" : "disable");

	return ret;
}

static int bcm_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	int ret;
	unsigned int seconds, val;

	ret = bcm_smc_internal(bbl_rtc_smc_ids[OEM_RTC_ALARM_CONTROL],
				 OEM_RTC_ALARM_CONTROL_READ, 0, &seconds);
	if (ret < 0)
		goto err;

	rtc_time_to_tm(seconds, &alm->time);
	ret = bcm_smc_internal(bbl_rtc_smc_ids[OEM_RTC_ALARM_CONTROL],
				 OEM_RTC_ALARM_CONTROL_INTR_READ, 0, &val);
	if (ret < 0)
		goto err;

	val &= RTC_REG_ALARM_INTR;
	alm->pending = !val;
	alm->enabled = alm->pending && device_may_wakeup(dev);

	ret = 0;
err:
	if (ret < 0)
		dev_err(&bcm_rtc.dev, "couldn't read alarm\n");
	return ret;
}

static int bcm_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned long seconds;
	int ret;

	rtc_tm_to_time(&alm->time, &seconds);
	ret = bcm_smc_internal(bbl_rtc_smc_ids[OEM_RTC_ALARM_CONTROL],
				 OEM_RTC_ALARM_CONTROL_SET, seconds, NULL);
	if (ret < 0)
		dev_err(&bcm_rtc.dev, "couldn't set alarm\n");

	return ret;
}

static struct rtc_class_ops bcm_rtc_ops = {
	.read_time			= bcm_rtc_read_time,
	.set_time			= bcm_rtc_set_time,
	.alarm_irq_enable		= bcm_rtc_alarm_irq_enable,
	.read_alarm			= bcm_rtc_read_alarm,
	.set_alarm			= bcm_rtc_set_alarm,
};

static struct rtc_class_ops peg_rtc_ops = {
	.read_time			= bcm_rtc_read_time,
	.set_time			= bcm_rtc_set_time,
};

static const struct of_device_id bcm_rtc_of_match[] = {
	{.compatible = "brcm,ns2-secure-rtc",},
	{.compatible = "brcm,peg-secure-rtc",},
	{ }
};

static int bcm_rtc_parse_dt(struct device_node *np)
{
	int ret;
	u32 id;

	dev_info(&bcm_rtc.dev, "parsing device tree...\n");

	bcm_rtc.periodic_irq = irq_of_parse_and_map(np, 0);
	if (bcm_rtc.periodic_irq < 0)
		dev_err(&bcm_rtc.dev, "periodic_irq property not found\n");

	bcm_rtc.alarm_irq = irq_of_parse_and_map(np, 1);
	if (bcm_rtc.alarm_irq < 0)
		dev_err(&bcm_rtc.dev, "alarm_irq property not found\n");

	if (of_property_read_u32(np, "rtc_bbl_init", &id)) {
		dev_err(&bcm_rtc.dev, "prop rtc_bbl_init not found\n");
		ret = -ENODEV;
		goto err;
	}
	bbl_rtc_smc_ids[OEM_RTC_BBL_INIT] = id;

	if (of_property_read_u32(np, "rtc_control", &id)) {
		dev_err(&bcm_rtc.dev, "prop rtc_control not found\n");
		ret = -ENODEV;
		goto err;
	}
	bbl_rtc_smc_ids[OEM_RTC_CONTROL] = id;

	if (of_property_read_u32(np, "rtc_time_req", &id)) {
		dev_err(&bcm_rtc.dev, "prop rtc_time_req not found\n");
		ret = -ENODEV;
		goto err;
	}
	bbl_rtc_smc_ids[OEM_RTC_TIME_REQ] = id;

	if (of_property_read_u32(np, "rtc_alarm_control", &id))
		dev_err(&bcm_rtc.dev, "prop rtc_alarm_control not found\n");

	bbl_rtc_smc_ids[OEM_RTC_ALARM_CONTROL] = id;

	if (of_property_read_u32(np, "rtc_intr_stat_control", &id))
		dev_err(&bcm_rtc.dev, "prop rtc_intr_stat_control not found\n");

	bbl_rtc_smc_ids[OEM_RTC_INTR_STAT_CONTROL] = id;

	if (of_device_is_compatible(np, "brcm,ns2-secure-rtc"))
		bcm_rtc.rtc_ops = &bcm_rtc_ops;
	else if (of_device_is_compatible(np, "brcm,peg-secure-rtc"))
		bcm_rtc.rtc_ops = &peg_rtc_ops;

	return 0;
err:
	return ret;
}

static int bcm_rtc_probe(struct platform_device *pdev)
{
	struct device_node *dev_of = pdev->dev.of_node;
	int ret;

	memset(&bcm_rtc, 0, sizeof(struct bcm_rtc_t));
	bcm_rtc.dev = pdev->dev;

	ret = bcm_rtc_parse_dt(dev_of);
	if (ret < 0)
		goto fail;

	spin_lock_init(&bcm_rtc.lock);
	mutex_init(&bcm_rtc.mut_lock);

	ret = bbl_init();
	if (ret < 0)
		goto fail;

	platform_set_drvdata(pdev, &bcm_rtc);

	device_init_wakeup(&pdev->dev, 1);

	if (!bcm_rtc.rtc_ops)
		goto fail;

	bcm_rtc.rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
						       bcm_rtc.rtc_ops,
						       THIS_MODULE);

	if (IS_ERR(bcm_rtc.rtc)) {
		dev_err(&bcm_rtc.dev, "unable to register RTC device, err %ld\n",
			PTR_ERR(bcm_rtc.rtc));
		ret = PTR_ERR(bcm_rtc.rtc);
		goto fail;
	}

	if (bcm_rtc.periodic_irq > 0) {
		ret = devm_request_irq(&pdev->dev, bcm_rtc.periodic_irq,
				       bcm_rtc_interrupt, IRQF_SHARED,
				       "bcm_rtc_peri", &bcm_rtc);
		if (ret < 0) {
			dev_err(&bcm_rtc.dev,
				"couldn't register periodic interrupt\n");
			goto fail;
		}
	}

	if (bcm_rtc.alarm_irq > 0) {
		ret = devm_request_irq(&pdev->dev, bcm_rtc.alarm_irq,
				       bcm_rtc_interrupt, 0,
				       "bcm_rtc_alarm", &bcm_rtc);
		if (ret < 0) {
			dev_err(&bcm_rtc.dev,
				"couldn't register alarm interrupt\n");
			goto fail;
		}
	}

	ret = bcm_rtc_enable();
	if (ret < 0) {
		dev_err(&bcm_rtc.dev,  "couldn't enable rtc\n");
		goto fail;
	}

	return 0;

fail:
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int bcm_rtc_remove(struct platform_device *pdev)
{
	int ret;

	device_init_wakeup(&pdev->dev, 0);

	ret = bcm_rtc_disable();
	if (ret < 0)
		dev_err(&bcm_rtc.dev, "couldn't disable rtc\n");

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver bcm_secure_rtc_driver = {
	.probe		= bcm_rtc_probe,
	.remove		= bcm_rtc_remove,
	.driver		= {
		.name = "iproc-secure-rtc",
		.owner = THIS_MODULE,
		.of_match_table = bcm_rtc_of_match
	},
};

module_platform_driver(bcm_secure_rtc_driver);

MODULE_AUTHOR("Pratik Rathod <pratik.rathod@broadcom.com>");
MODULE_DESCRIPTION("Broadcom NS2 Secure RTC Driver");
MODULE_LICENSE("GPL v2");
