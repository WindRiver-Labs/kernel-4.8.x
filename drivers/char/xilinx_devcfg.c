/*
 * Xilinx Device Config driver
 *
 * Copyright (c) 2011 Xilinx Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA
 * 02139, USA.
 */

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysctl.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <mach/slcr.h>

#define DRIVER_NAME "xdevcfg"
#define XDEVCFG_DEVICES 1

/* An array, which is set to true when the device is registered. */
static DEFINE_MUTEX(xdevcfg_mutex);

/* Constant Definitions */
#define XDCFG_CTRL_OFFSET		0x00 /* Control Register */
#define XDCFG_LOCK_OFFSET		0x04 /* Lock Register */
#define XDCFG_INT_STS_OFFSET		0x0C /* Interrupt Status Register */
#define XDCFG_INT_MASK_OFFSET		0x10 /* Interrupt Mask Register */
#define XDCFG_STATUS_OFFSET		0x14 /* Status Register */
#define XDCFG_DMA_SRC_ADDR_OFFSET	0x18 /* DMA Source Address Register */
#define XDCFG_DMA_DEST_ADDR_OFFSET	0x1C /* DMA Destination Address Reg */
#define XDCFG_DMA_SRC_LEN_OFFSET	0x20 /* DMA Source Transfer Length */
#define XDCFG_DMA_DEST_LEN_OFFSET	0x24 /* DMA Destination Transfer */
#define XDCFG_UNLOCK_OFFSET		0x34 /* Unlock Register */

/* Control Register Bit definitions */
#define XDCFG_CTRL_PCFG_PROG_B_MASK	0x40000000 /* Program signal to
						    *  Reset FPGA */
#define XDCFG_CTRL_PCAP_PR_MASK		0x08000000 /* Enable PCAP for PR */
#define XDCFG_CTRL_PCAP_MODE_MASK	0x04000000 /* Enable PCAP */
#define XDCFG_CTRL_USER_MODE_MASK	0x00008000 /* ROM/user mode selection */
#define XDCFG_CTRL_PCFG_AES_EN_MASK	0x00000E00 /* AES Enable Mask */
#define XDCFG_CTRL_SEU_EN_MASK		0x00000100 /* SEU Enable Mask */
#define XDCFG_CTRL_SPNIDEN_MASK		0x00000040 /* Secure Non Invasive
						    *  Debug Enable */
#define XDCFG_CTRL_SPIDEN_MASK		0x00000020 /* Secure Invasive
						    *  Debug Enable */
#define XDCFG_CTRL_NIDEN_MASK		0x00000010 /* Non-Invasive Debug
						    *  Enable */
#define XDCFG_CTRL_DBGEN_MASK		0x00000008 /* Invasive Debug
						    *  Enable */
#define XDCFG_CTRL_DAP_EN_MASK		0x00000007 /* DAP Enable Mask */

/* Lock register bit definitions */

#define XDCFG_LOCK_AES_EN_MASK		0x00000008 /* Lock AES_EN update */
#define XDCFG_LOCK_SEU_MASK		0x00000004 /* Lock SEU_En update */
#define XDCFG_LOCK_DBG_MASK		0x00000001 /* This bit locks
						    *  security config
						    *  including: DAP_En,
						    *  DBGEN,NIDEN, SPNIEN */

/* Status register bit definitions */
#define XDCFG_STATUS_PCFG_INIT_MASK	0x00000010 /* FPGA init status */

/* Interrupt Status/Mask Register Bit definitions */
#define XDCFG_IXR_DMA_DONE_MASK		0x00002000 /* DMA Command Done */
#define XDCFG_IXR_PCFG_DONE_MASK	0x00000004 /* FPGA programmed */
#define XDCFG_IXR_ERROR_FLAGS_MASK	0x00F0F860
#define XDCFG_IXR_ALL_MASK		0xF8F7F87F
/* Miscellaneous constant values */
#define XDCFG_DMA_INVALID_ADDRESS	0xFFFFFFFF  /* Invalid DMA address */

/**
 * struct xdevcfg_drvdata - Device Configuration driver structure
 *
 * @dev: Pointer to the device structure
 * @cdev: Instance of the cdev structure
 * @devt: Pointer to the dev_t structure
 * @class: Pointer to device class
 * @dma_done: The dma_done status bit for the DMA command completion
 * @error_status: The error status captured during the DMA transfer
 * @irq: Interrupt number
 * @clk: Peripheral clock for devcfg
 * @is_open: The status bit to indicate whether the device is opened
 * @sem: Instance for the mutex
 * @lock: Instance of spinlock
 * @base_address: The virtual device base address of the device registers
 * @is_partial_bitstream: Status bit to indicate partial/full bitstream
 */
struct xdevcfg_drvdata {
	struct device *dev;
	struct cdev cdev;
	dev_t devt;
	struct class *class;
	int irq;
	struct clk *clk;
	volatile bool dma_done;
	volatile int error_status;
	bool is_open;
	struct mutex sem;
	spinlock_t lock;
	void __iomem *base_address;
	int ep107;
	bool is_partial_bitstream;
};

/* Register read/write access routines */
#define xdevcfg_writereg(offset, val)	__raw_writel(val, offset)
#define xdevcfg_readreg(offset)		__raw_readl(offset)

/**
 * xdevcfg_reset_pl() - Reset the programmable logic.
 * @base_address:	The base address of the device.
 *
 * Must be called with PCAP clock enabled
 */
static void xdevcfg_reset_pl(void __iomem *base_address)
{
	/*
	 * Create a rising edge on PCFG_INIT. PCFG_INIT follows PCFG_PROG_B,
	 * so we need to * poll it after setting PCFG_PROG_B to make sure that
	 * the rising edge happens.
	 */
	xdevcfg_writereg(base_address + XDCFG_CTRL_OFFSET,
			(xdevcfg_readreg(base_address + XDCFG_CTRL_OFFSET) |
			 XDCFG_CTRL_PCFG_PROG_B_MASK));
	while (!(xdevcfg_readreg(base_address + XDCFG_STATUS_OFFSET) &
				XDCFG_STATUS_PCFG_INIT_MASK))
		;

	xdevcfg_writereg(base_address + XDCFG_CTRL_OFFSET,
			(xdevcfg_readreg(base_address + XDCFG_CTRL_OFFSET) &
			 ~XDCFG_CTRL_PCFG_PROG_B_MASK));
	while (xdevcfg_readreg(base_address + XDCFG_STATUS_OFFSET) &
			XDCFG_STATUS_PCFG_INIT_MASK)
		;

	xdevcfg_writereg(base_address + XDCFG_CTRL_OFFSET,
			(xdevcfg_readreg(base_address + XDCFG_CTRL_OFFSET) |
			 XDCFG_CTRL_PCFG_PROG_B_MASK));
	while (!(xdevcfg_readreg(base_address + XDCFG_STATUS_OFFSET) &
				XDCFG_STATUS_PCFG_INIT_MASK))
		;
}

/**
 * xdevcfg_irq() - The main interrupt handler.
 * @irq:	The interrupt number.
 * @data:	Pointer to the driver data structure.
 * returns: IRQ_HANDLED after the interrupt is handled.
 **/
static irqreturn_t xdevcfg_irq(int irq, void *data)
{
	u32 intr_status;
	struct xdevcfg_drvdata *drvdata = data;

	spin_lock(&drvdata->lock);

	intr_status = xdevcfg_readreg(drvdata->base_address +
					XDCFG_INT_STS_OFFSET);

	/* Clear the interrupts */
	xdevcfg_writereg(drvdata->base_address + XDCFG_INT_STS_OFFSET,
				intr_status);

	if ((intr_status & XDCFG_IXR_DMA_DONE_MASK) == XDCFG_IXR_DMA_DONE_MASK)
		drvdata->dma_done = 1;

	if ((intr_status & XDCFG_IXR_ERROR_FLAGS_MASK) ==
			XDCFG_IXR_ERROR_FLAGS_MASK)
		drvdata->error_status = 1;

	spin_unlock(&drvdata->lock);

	return IRQ_HANDLED;
}

/**
 * xdevcfg_write() - The is the driver write function.
 *
 * @file:	Pointer to the file structure.
 * @buf:	Pointer to the bitstream location.
 * @count:	The number of bytes to be written.
 * @ppos:	Pointer to the offset value
 * returns:	Success or error status.
 **/
static ssize_t
xdevcfg_write(struct file *file, const char __user *buf, size_t count,
		loff_t *ppos)
{
	u32 *kbuf;
	int status;
	unsigned long timeout;
	u32 intr_reg;
	dma_addr_t dma_addr;
	u32 transfer_length = 0;
	struct xdevcfg_drvdata *drvdata = file->private_data;

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	status = mutex_lock_interruptible(&drvdata->sem);

	if (status)
		goto err_clk;

	kbuf = dma_alloc_coherent(drvdata->dev, count, &dma_addr, GFP_KERNEL);
	if (!kbuf) {
		status = -ENOMEM;
		goto err_unlock;
	}

	if (copy_from_user(kbuf, buf, count)) {
		status = -EFAULT;
		goto error;
	}
	/* Enable DMA and error interrupts */
	xdevcfg_writereg(drvdata->base_address + XDCFG_INT_STS_OFFSET,
				XDCFG_IXR_ALL_MASK);


	xdevcfg_writereg(drvdata->base_address + XDCFG_INT_MASK_OFFSET,
				(u32) (~(XDCFG_IXR_DMA_DONE_MASK |
				XDCFG_IXR_ERROR_FLAGS_MASK)));

	drvdata->dma_done = 0;
	drvdata->error_status = 0;

	/* Initiate DMA write command */
	if (count < 0x1000)
		xdevcfg_writereg(drvdata->base_address +
			XDCFG_DMA_SRC_ADDR_OFFSET, (u32)(dma_addr + 1));
	else
		xdevcfg_writereg(drvdata->base_address +
			XDCFG_DMA_SRC_ADDR_OFFSET, (u32) dma_addr);

	xdevcfg_writereg(drvdata->base_address + XDCFG_DMA_DEST_ADDR_OFFSET,
				(u32)XDCFG_DMA_INVALID_ADDRESS);
	/* Convert number of bytes to number of words.  */
	if (count % 4)
		transfer_length	= (count / 4 + 1);
	else
		transfer_length	= count / 4;
	xdevcfg_writereg(drvdata->base_address + XDCFG_DMA_SRC_LEN_OFFSET,
				transfer_length);
	xdevcfg_writereg(drvdata->base_address + XDCFG_DMA_DEST_LEN_OFFSET, 0);

	timeout = jiffies + msecs_to_jiffies(1000);

	while (!drvdata->dma_done) {
		if (time_after(jiffies, timeout)) {
				status = -ETIMEDOUT;
				goto error;
		}
	}

	if (drvdata->error_status)
		status = drvdata->error_status;

	/* Disable the DMA and error interrupts */
	intr_reg = xdevcfg_readreg(drvdata->base_address +
					XDCFG_INT_MASK_OFFSET);
	xdevcfg_writereg(drvdata->base_address + XDCFG_INT_MASK_OFFSET,
				intr_reg | (XDCFG_IXR_DMA_DONE_MASK |
				XDCFG_IXR_ERROR_FLAGS_MASK));

	/* If we didn't write correctly, then bail out. */
	if (status) {
		status = -EFAULT;
		goto error;
	}

	status = count;

error:
	dma_free_coherent(drvdata->dev, count, kbuf, dma_addr);
err_unlock:
	mutex_unlock(&drvdata->sem);
err_clk:
	clk_disable(drvdata->clk);
	return status;
}


/**
 * xdevcfg_read() - The is the driver read function.
 * @file:	Pointer to the file structure.
 * @buf:	Pointer to the bitstream location.
 * @count:	The number of bytes read.
 * @ppos:	Pointer to the offsetvalue
 * returns:	Success or error status.
 */
static ssize_t
xdevcfg_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	u32 *kbuf;
	int status;
	unsigned long timeout;
	dma_addr_t dma_addr;
	struct xdevcfg_drvdata *drvdata = file->private_data;
	u32 intr_reg;

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	status = mutex_lock_interruptible(&drvdata->sem);
	if (status)
		goto err_clk;

	/* Get new data from the ICAP, and return was requested. */
	kbuf = dma_alloc_coherent(drvdata->dev, count, &dma_addr, GFP_KERNEL);
	if (!kbuf) {
		status = -ENOMEM;
		goto err_unlock;
	}

	drvdata->dma_done = 0;
	drvdata->error_status = 0;

	/* Enable DMA and error interrupts */
	xdevcfg_writereg(drvdata->base_address + XDCFG_INT_STS_OFFSET,
				XDCFG_IXR_ALL_MASK);

	xdevcfg_writereg(drvdata->base_address + XDCFG_INT_MASK_OFFSET,
				(u32) (~(XDCFG_IXR_DMA_DONE_MASK |
				XDCFG_IXR_ERROR_FLAGS_MASK)));
	/* Initiate DMA read command */
	xdevcfg_writereg(drvdata->base_address + XDCFG_DMA_SRC_ADDR_OFFSET,
				(u32)XDCFG_DMA_INVALID_ADDRESS);
	xdevcfg_writereg(drvdata->base_address + XDCFG_DMA_DEST_ADDR_OFFSET,
				(u32)dma_addr);
	xdevcfg_writereg(drvdata->base_address + XDCFG_DMA_SRC_LEN_OFFSET, 0);
	xdevcfg_writereg(drvdata->base_address + XDCFG_DMA_DEST_LEN_OFFSET,
				count / 4);

	timeout = jiffies + msecs_to_jiffies(1000);

	while (!drvdata->dma_done) {
		if (time_after(jiffies, timeout)) {
			status = -ETIMEDOUT;
			goto error;
		}
	}

	if (drvdata->error_status)
		status = drvdata->error_status;

	/* Disable and clear DMA and error interrupts */
	intr_reg = xdevcfg_readreg(drvdata->base_address +
					XDCFG_INT_MASK_OFFSET);
	xdevcfg_writereg(drvdata->base_address + XDCFG_INT_MASK_OFFSET,
				intr_reg | (XDCFG_IXR_DMA_DONE_MASK |
				XDCFG_IXR_ERROR_FLAGS_MASK));


	/* If we didn't read correctly, then bail out. */
	if (status) {
		status = -EFAULT;
		goto error;
	}

	/* If we fail to return the data to the user, then bail out. */
	if (copy_to_user(buf, kbuf, count)) {
		status = -EFAULT;
		goto error;
	}

	status = count;
error:
	dma_free_coherent(drvdata->dev, count, kbuf, dma_addr);
err_unlock:
	mutex_unlock(&drvdata->sem);
err_clk:
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_open() - The is the driver open function.
 * @inode:	Pointer to the inode structure of this device.
 * @file:	Pointer to the file structure.
 * returns:	Success or error status.
 */
static int xdevcfg_open(struct inode *inode, struct file *file)
{
	struct xdevcfg_drvdata *drvdata;
	int status;

	drvdata = container_of(inode->i_cdev, struct xdevcfg_drvdata, cdev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	status = mutex_lock_interruptible(&drvdata->sem);
	if (status)
		goto err_clk;

	if (drvdata->is_open) {
		status = -EBUSY;
		goto error;
	}

	file->private_data = drvdata;
	drvdata->is_open = 1;

	/*
	 * If is_partial_bitstream is set, then PROG_B is not asserted
	 * (xdevcfg_reset_pl function) and also xslcr_init_preload_fpga and
	 * xslcr_init_postload_fpga functions are not invoked.
	 */
	if (!drvdata->is_partial_bitstream)
		xslcr_init_preload_fpga();

	/*
	 * Only do the reset of the PL for Zynq as it causes problems on the
	 * EP107 and the issue is not understood, but not worth investigating
	 * as the emulation platform is very different than silicon and not a
	 * complete implementation. Also, do not reset if it is a partial
	 * bitstream.
	 */
	if ((!drvdata->ep107) && (!drvdata->is_partial_bitstream))
		xdevcfg_reset_pl(drvdata->base_address);

	xdevcfg_writereg(drvdata->base_address + XDCFG_INT_STS_OFFSET,
			XDCFG_IXR_PCFG_DONE_MASK);

error:
	mutex_unlock(&drvdata->sem);
err_clk:
	clk_disable(drvdata->clk);
	return status;
}

/**
 * xdevcfg_release() - The is the driver release function.
 * @inode:	Pointer to the inode structure of this device.
 * @file:	Pointer to the file structure.
 * returns:	Success.
 */
static int xdevcfg_release(struct inode *inode, struct file *file)
{
	struct xdevcfg_drvdata *drvdata = file->private_data;

	if (!drvdata->is_partial_bitstream)
		xslcr_init_postload_fpga();

	drvdata->is_open = 0;

	return 0;
}

static const struct file_operations xdevcfg_fops = {
	.owner = THIS_MODULE,
	.write = xdevcfg_write,
	.read = xdevcfg_read,
	.open = xdevcfg_open,
	.release = xdevcfg_release,
};

/*
 * The following functions are the routines provided to the user to
 * set/get the status bit value in the control/lock registers.
 */

/**
 * xdevcfg_set_dap_en() - This function sets the DAP bits in the
 * control register with the given value.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	negative error if the string could not be converted
 *		or the size of the buffer.
 */
static ssize_t xdevcfg_set_dap_en(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 ctrl_reg_status;
	unsigned long flags;
	unsigned long mask_bit;
	int status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	ctrl_reg_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_CTRL_OFFSET);
	spin_lock_irqsave(&drvdata->lock, flags);

	status = strict_strtoul(buf, 10, &mask_bit);

	if (status)
		goto err_unlock;

	if (mask_bit > 7) {
		status = -EINVAL;
		goto err_unlock;
	}

	xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
			(ctrl_reg_status |
			 (((u32)mask_bit) & XDCFG_CTRL_DAP_EN_MASK)));

	spin_unlock_irqrestore(&drvdata->lock, flags);

	clk_disable(drvdata->clk);

	return size;

err_unlock:
	spin_unlock_irqrestore(&drvdata->lock, flags);
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_show_dap_en_status() - The function returns the DAP_EN bits status in
 * the control register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	Size of the buffer.
 */
static ssize_t xdevcfg_show_dap_en_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 dap_en_status;
	int status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	dap_en_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_CTRL_OFFSET) & XDCFG_CTRL_DAP_EN_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", dap_en_status);

	return status;
}

static DEVICE_ATTR(enable_dap, 0644, xdevcfg_show_dap_en_status,
				xdevcfg_set_dap_en);

/**
 * xdevcfg_set_dbgen() - This function sets the DBGEN bit in the
 * control register with the given value.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	-EINVAL if invalid parameter is sent or size
 */
static ssize_t xdevcfg_set_dbgen(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 ctrl_reg_status;
	unsigned long flags;
	unsigned long mask_bit;
	int status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	ctrl_reg_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_CTRL_OFFSET);

	status = strict_strtoul(buf, 10, &mask_bit);

	if (status)
		goto err_clk;

	if (mask_bit > 1) {
		status = -EINVAL;
		goto err_clk;
	}

	spin_lock_irqsave(&drvdata->lock, flags);

	if (mask_bit)
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status | XDCFG_CTRL_DBGEN_MASK));
	else
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status & (~XDCFG_CTRL_DBGEN_MASK)));

	spin_unlock_irqrestore(&drvdata->lock, flags);

	clk_disable(drvdata->clk);

	return size;

err_clk:
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_show_dbgen_status() - The function returns the DBGEN bit status in
 * the control register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	Size of the buffer.
 */
static ssize_t xdevcfg_show_dbgen_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 dbgen_status;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	dbgen_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_CTRL_OFFSET) & XDCFG_CTRL_DBGEN_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", (dbgen_status >> 3));

	return status;
}

static DEVICE_ATTR(enable_dbg_in, 0644, xdevcfg_show_dbgen_status,
				xdevcfg_set_dbgen);

/**
 * xdevcfg_set_niden() - This function sets the NIDEN bit in the
 * control register with the given value.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	-EINVAL if invalid parameter is sent or size
 */
static ssize_t xdevcfg_set_niden(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 ctrl_reg_status;
	unsigned long flags;
	unsigned long mask_bit;
	int status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	ctrl_reg_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_CTRL_OFFSET);

	status = strict_strtoul(buf, 10, &mask_bit);

	if (status)
		goto err_clk;

	if (mask_bit > 1) {
		status = -EINVAL;
		goto err_clk;
	}

	spin_lock_irqsave(&drvdata->lock, flags);

	if (mask_bit)
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status | XDCFG_CTRL_NIDEN_MASK));
	else
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status & (~XDCFG_CTRL_NIDEN_MASK)));

	spin_unlock_irqrestore(&drvdata->lock, flags);

	clk_disable(drvdata->clk);

	return size;

err_clk:
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_show_niden_status() - The function returns the NIDEN bit status in
 * the control register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	Size of the buffer.
 */
static ssize_t xdevcfg_show_niden_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 niden_status;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	niden_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_CTRL_OFFSET) & XDCFG_CTRL_NIDEN_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", (niden_status >> 4));

	return status;
}

static DEVICE_ATTR(enable_dbg_nonin, 0644, xdevcfg_show_niden_status,
			xdevcfg_set_niden);

/**
 * xdevcfg_set_spiden() - This function sets the SPIDEN bit in the
 * control register with the given value.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	-EINVAL if invalid parameter is sent or size
 */
static ssize_t xdevcfg_set_spiden(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 ctrl_reg_status;
	unsigned long flags;
	unsigned long mask_bit;
	int status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	ctrl_reg_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_CTRL_OFFSET);

	status = strict_strtoul(buf, 10, &mask_bit);

	if (status)
		goto err_clk;

	if (mask_bit > 1) {
		status = -EINVAL;
		goto err_clk;
	}

	spin_lock_irqsave(&drvdata->lock, flags);

	if (mask_bit)
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status | XDCFG_CTRL_SPIDEN_MASK));
	else

		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status & (~XDCFG_CTRL_SPIDEN_MASK)));

	spin_unlock_irqrestore(&drvdata->lock, flags);

	clk_disable(drvdata->clk);

	return size;

err_clk:
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_show_spiden_status() - The function returns the SPIDEN bit status in
 * the control register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	Size of the buffer.
 */
static ssize_t xdevcfg_show_spiden_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 spiden_status;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	spiden_status = xdevcfg_readreg(drvdata->base_address +
			XDCFG_CTRL_OFFSET) & XDCFG_CTRL_SPIDEN_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", (spiden_status >> 5));

	return status;
}

static DEVICE_ATTR(enable_sec_dbg_in, 0644, xdevcfg_show_spiden_status,
				xdevcfg_set_spiden);

/**
 * xdevcfg_set_spniden() - This function sets the SPNIDEN bit in the
 * control register with the given value.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	-EINVAL if invalid parameter is sent or the size of buffer
 */
static ssize_t xdevcfg_set_spniden(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 ctrl_reg_status;
	unsigned long flags;
	unsigned long mask_bit;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	ctrl_reg_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_CTRL_OFFSET);
	status = strict_strtoul(buf, 10, &mask_bit);

	if (status)
		goto err_clk;

	if (mask_bit > 1) {
		status = -EINVAL;
		goto err_clk;
	}

	spin_lock_irqsave(&drvdata->lock, flags);

	if (mask_bit)
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status | XDCFG_CTRL_SPNIDEN_MASK));
	else
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status & (~XDCFG_CTRL_SPNIDEN_MASK)));

	spin_unlock_irqrestore(&drvdata->lock, flags);

	clk_disable(drvdata->clk);

	return size;

err_clk:
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_show_spniden_status() - The function returns the SPNIDEN bit status
 * in the control register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	Size of the buffer.
 */
static ssize_t xdevcfg_show_spniden_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 spniden_status;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	spniden_status = xdevcfg_readreg(drvdata->base_address +
			XDCFG_CTRL_OFFSET) & XDCFG_CTRL_SPNIDEN_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", (spniden_status >> 6));

	return status;
}

static DEVICE_ATTR(enable_sec_dbg_nonin, 0644, xdevcfg_show_spniden_status,
					xdevcfg_set_spniden);

/**
 * xdevcfg_set_seu() - This function sets the SEU_EN bit in the
 * control register with the given value
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	-EINVAL if invalid parameter is sent or size
 */
static ssize_t xdevcfg_set_seu(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 ctrl_reg_status;
	unsigned long flags;
	unsigned long mask_bit;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	ctrl_reg_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_CTRL_OFFSET);

	status = strict_strtoul(buf, 10, &mask_bit);

	if (status)
		goto err_clk;

	if (mask_bit > 1) {
		status = -EINVAL;
		goto err_clk;
	}

	spin_lock_irqsave(&drvdata->lock, flags);

	if (mask_bit)
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status | XDCFG_CTRL_SEU_EN_MASK));
	else
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status & (~XDCFG_CTRL_SEU_EN_MASK)));

	spin_unlock_irqrestore(&drvdata->lock, flags);

	clk_disable(drvdata->clk);

	return size;

err_clk:
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_show_seu_status() - The function returns the SEU_EN bit status
 * in the control register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	size of the buffer.
 */
static ssize_t xdevcfg_show_seu_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 seu_status;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	seu_status = xdevcfg_readreg(drvdata->base_address +
			XDCFG_CTRL_OFFSET) & XDCFG_CTRL_SEU_EN_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", (seu_status > 8));

	return status;
}

static DEVICE_ATTR(enable_seu, 0644, xdevcfg_show_seu_status, xdevcfg_set_seu);

/**
 * xdevcfg_set_aes() - This function sets the AES_EN bits in the
 * control register with either all 1s or all 0s.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	-EINVAL if invalid parameter is sent or size
 *
 * The user must send only one bit in the buffer to notify whether he wants to
 * either set or reset these bits.
 */
static ssize_t xdevcfg_set_aes(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 ctrl_reg_status;
	unsigned long flags;
	unsigned long mask_bit;
	int status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	ctrl_reg_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_CTRL_OFFSET);

	status = strict_strtoul(buf, 10, &mask_bit);

	if (status < 0)
		goto err_clk;

	if (mask_bit > 1) {
		status = -EINVAL;
		goto err_clk;
	}


	spin_lock_irqsave(&drvdata->lock, flags);

	if (mask_bit)
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status |
				 XDCFG_CTRL_PCFG_AES_EN_MASK));
	else
		xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(ctrl_reg_status &
				 (~XDCFG_CTRL_PCFG_AES_EN_MASK)));

	spin_unlock_irqrestore(&drvdata->lock, flags);

	clk_disable(drvdata->clk);

	return size;

err_clk:
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_show_aes_status() - The function returns the AES_EN bit status
 * in the control register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	size of the buffer.
 */
static ssize_t xdevcfg_show_aes_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 aes_status;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	aes_status = xdevcfg_readreg(drvdata->base_address +
			XDCFG_CTRL_OFFSET) & XDCFG_CTRL_PCFG_AES_EN_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", (aes_status >> 9));

	return status;
}

static DEVICE_ATTR(enable_aes, 0644, xdevcfg_show_aes_status, xdevcfg_set_aes);

/**
 * xdevcfg_set_aes_en_lock() - This function sets the LOCK_AES_EN bit in the
 * lock register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	-EINVAL if invalid parameter is sent or size
 */
static ssize_t xdevcfg_set_aes_en_lock(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 aes_en_lock_status;
	unsigned long flags;
	unsigned long mask_bit;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	aes_en_lock_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_LOCK_OFFSET);

	status = strict_strtoul(buf, 10, &mask_bit);

	if (status)
		goto err_clk;

	if (mask_bit > 1) {
		status = -EINVAL;
		goto err_clk;
	}

	spin_lock_irqsave(&drvdata->lock, flags);

	if (mask_bit)
		xdevcfg_writereg(drvdata->base_address + XDCFG_LOCK_OFFSET,
				(aes_en_lock_status | XDCFG_LOCK_AES_EN_MASK));
	else
		xdevcfg_writereg(drvdata->base_address + XDCFG_LOCK_OFFSET,
				(aes_en_lock_status &
				 (~XDCFG_LOCK_AES_EN_MASK)));

	spin_unlock_irqrestore(&drvdata->lock, flags);

	clk_disable(drvdata->clk);

	return size;

err_clk:
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_show_aes_en_lock_status() - The function returns the LOCK_AES_EN bit
 * status in the lock register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	size of the buffer.
 */
static ssize_t xdevcfg_show_aes_en_lock_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 aes_en_lock_status;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	aes_en_lock_status = xdevcfg_readreg(drvdata->base_address +
			XDCFG_LOCK_OFFSET) & XDCFG_LOCK_AES_EN_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", (aes_en_lock_status >> 3));

	return status;
}

static DEVICE_ATTR(aes_en_lock, 0644, xdevcfg_show_aes_en_lock_status,
				xdevcfg_set_aes_en_lock);

/**
 * xdevcfg_set_seu_lock() - This function sets the LOCK_SEU bit in the
 * lock register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	-EINVAL if invalid parameter is sent or size
 */
static ssize_t xdevcfg_set_seu_lock(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 seu_lock_status;
	unsigned long flags;
	unsigned long mask_bit;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	seu_lock_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_LOCK_OFFSET);

	status = strict_strtoul(buf, 10, &mask_bit);

	if (status)
		goto err_clk;

	if (mask_bit > 1) {
		status = -EINVAL;
		goto err_clk;
	}

	spin_lock_irqsave(&drvdata->lock, flags);

	if (mask_bit)
		xdevcfg_writereg(drvdata->base_address + XDCFG_LOCK_OFFSET,
				(seu_lock_status | XDCFG_LOCK_SEU_MASK));
	else
		xdevcfg_writereg(drvdata->base_address + XDCFG_LOCK_OFFSET,
				(seu_lock_status  & (~XDCFG_LOCK_SEU_MASK)));

	spin_unlock_irqrestore(&drvdata->lock, flags);

	clk_disable(drvdata->clk);

	return size;

err_clk:
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_show_seu_lock_status() - The function returns the LOCK_SEU bit
 * status in the lock register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	size of the buffer.
 */
static ssize_t xdevcfg_show_seu_lock_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 seu_lock_status;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	seu_lock_status = xdevcfg_readreg(drvdata->base_address +
			XDCFG_LOCK_OFFSET) & XDCFG_LOCK_SEU_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", (seu_lock_status >> 2));

	return status;
}

static DEVICE_ATTR(seu_lock, 0644, xdevcfg_show_seu_lock_status,
					xdevcfg_set_seu_lock);

/**
 * xdevcfg_set_dbg_lock() - This function sets the LOCK_DBG bit in the
 * lock register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	-EINVAL if invalid parameter is sent or size
 */
static ssize_t xdevcfg_set_dbg_lock(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u32 lock_reg_status;
	unsigned long flags;
	unsigned long mask_bit;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	lock_reg_status = xdevcfg_readreg(drvdata->base_address +
				XDCFG_LOCK_OFFSET);
	status = strict_strtoul(buf, 10, &mask_bit);

	if (status)
		goto err_clk;

	if (mask_bit > 1) {
		status = -EINVAL;
		goto err_clk;
	}

	spin_lock_irqsave(&drvdata->lock, flags);

	if (mask_bit)
		xdevcfg_writereg(drvdata->base_address + XDCFG_LOCK_OFFSET,
				(lock_reg_status | XDCFG_LOCK_DBG_MASK));
	else
		xdevcfg_writereg(drvdata->base_address + XDCFG_LOCK_OFFSET,
				(lock_reg_status & (~XDCFG_LOCK_DBG_MASK)));

	spin_unlock_irqrestore(&drvdata->lock, flags);

	clk_disable(drvdata->clk);

	return size;

err_clk:
	clk_disable(drvdata->clk);

	return status;
}

/**
 * xdevcfg_show_dbg_lock_status() - The function returns the LOCK_DBG bit
 * status in the lock register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	size of the buffer.
 */
static ssize_t xdevcfg_show_dbg_lock_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 dbg_lock_status;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	dbg_lock_status = xdevcfg_readreg(drvdata->base_address +
			XDCFG_LOCK_OFFSET) & XDCFG_LOCK_DBG_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", dbg_lock_status);

	return status;
}

static DEVICE_ATTR(dbg_lock, 0644, xdevcfg_show_dbg_lock_status,
				xdevcfg_set_dbg_lock);

/**
 * xdevcfg_show_prog_done_status() - The function returns the PROG_DONE bit
 * status in the interrupt status register.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	size of the buffer.
 */
static ssize_t xdevcfg_show_prog_done_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 prog_done_status;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = clk_enable(drvdata->clk);
	if (status)
		return status;

	prog_done_status = xdevcfg_readreg(drvdata->base_address +
			XDCFG_INT_STS_OFFSET) & XDCFG_IXR_PCFG_DONE_MASK;

	clk_disable(drvdata->clk);

	status = sprintf(buf, "%d\n", (prog_done_status >> 2));

	return status;
}

static DEVICE_ATTR(prog_done, 0644, xdevcfg_show_prog_done_status,
				NULL);

/**
 * xdevcfg_set_is_partial_bitstream() - This function sets the
 * is_partial_bitstream variable. If is_partial_bitstream is set,
 * then PROG_B is not asserted (xdevcfg_reset_pl) and also
 * xslcr_init_preload_fpga and xslcr_init_postload_fpga functions
 * are not invoked.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * @size:	The number of bytes used from the buffer
 * returns:	-EINVAL if invalid parameter is sent or size
 */
static ssize_t xdevcfg_set_is_partial_bitstream(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long mask_bit;
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = strict_strtoul(buf, 10, &mask_bit);

	if (status)
		return status;

	if (mask_bit > 1)
		return -EINVAL;

	if (mask_bit)
		drvdata->is_partial_bitstream = 1;
	else
		drvdata->is_partial_bitstream = 0;

	return size;
}

/**
 * xdevcfg_show_is_partial_bitstream_status() - The function returns the
 * value of is_partial_bitstream variable.
 * @dev:	Pointer to the device structure.
 * @attr:	Pointer to the device attribute structure.
 * @buf:	Pointer to the buffer location for the configuration
 *		data.
 * returns:	size of the buffer.
 */
static ssize_t xdevcfg_show_is_partial_bitstream_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t status;
	struct xdevcfg_drvdata *drvdata = dev_get_drvdata(dev);

	status = sprintf(buf, "%d\n", drvdata->is_partial_bitstream);

	return status;
}

static DEVICE_ATTR(is_partial_bitstream, 0644,
				xdevcfg_show_is_partial_bitstream_status,
				xdevcfg_set_is_partial_bitstream);

static const struct attribute *xdevcfg_attrs[] = {
	&dev_attr_prog_done.attr, /* PCFG_DONE bit in Intr Status register */
	&dev_attr_dbg_lock.attr, /* Debug lock bit in Lock register */
	&dev_attr_seu_lock.attr, /* SEU lock bit in Lock register */
	&dev_attr_aes_en_lock.attr, /* AES EN lock bit in Lock register */
	&dev_attr_enable_aes.attr, /* AES EN bit in Control register */
	&dev_attr_enable_seu.attr, /* SEU EN bit in Control register */
	&dev_attr_enable_sec_dbg_nonin.attr, /*SPNIDEN bit in Control register*/
	&dev_attr_enable_sec_dbg_in.attr, /*SPIDEN bit in Control register */
	&dev_attr_enable_dbg_nonin.attr, /* NIDEN bit in Control register */
	&dev_attr_enable_dbg_in.attr, /* DBGEN bit in Control register */
	&dev_attr_enable_dap.attr, /* DAP_EN bits in Control register */
	&dev_attr_is_partial_bitstream.attr, /* Flag for partial bitstream */
	NULL,
};


static const struct attribute_group xdevcfg_attr_group = {
	.attrs = (struct attribute **) xdevcfg_attrs,
};

/**
 * xdevcfg_drv_probe -  Probe call for the device.
 *
 * @pdev:	handle to the platform device structure.
 * Returns 0 on success, negative error otherwise.
 *
 * It does all the memory allocation and registration for the device.
 */
static int __devinit xdevcfg_drv_probe(struct platform_device *pdev)
{
	struct resource *regs_res, *irq_res;
	struct xdevcfg_drvdata *drvdata;
	dev_t devt;
	int retval;
	u32 ctrlreg;
	struct device_node *np;
	const void *prop;
	int size;
	struct device *dev;

	regs_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs_res) {
		dev_err(&pdev->dev, "Invalid address\n");
		return -ENODEV;
	}
	irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (!irq_res) {
		dev_err(&pdev->dev, "No IRQ found\n\n");
		return -ENODEV;
	}

	retval = alloc_chrdev_region(&devt, 0, XDEVCFG_DEVICES, DRIVER_NAME);
	if (retval < 0)
		return retval;

	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata) {
		dev_err(&pdev->dev,
				"Couldn't allocate device private record\n");
		retval = -ENOMEM;
		goto failed0;
	}

	dev_set_drvdata(&pdev->dev, drvdata);

	if (!request_mem_region(regs_res->start,
				regs_res->end - regs_res->start + 1,
				DRIVER_NAME)) {
		dev_err(&pdev->dev, "Couldn't lock memory region at %Lx\n",
			(unsigned long long)regs_res->start);
		retval = -EBUSY;
		goto failed1;
	}

	drvdata->devt = devt;
	drvdata->base_address = ioremap(regs_res->start,
				(regs_res->end - regs_res->start + 1));
	if (!drvdata->base_address) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto failed2;
	}

	spin_lock_init(&drvdata->lock);

	drvdata->irq = irq_res->start;

	retval = request_irq(irq_res->start, xdevcfg_irq, IRQF_DISABLED,
					DRIVER_NAME, drvdata);
	if (retval) {
		dev_err(&pdev->dev, "No IRQ available");
		retval = -EBUSY;
		goto failed3;
	}
	mutex_init(&drvdata->sem);
	drvdata->is_open = 0;
	drvdata->is_partial_bitstream = 0;
	drvdata->dma_done = 0;
	drvdata->error_status = 0;
	dev_info(&pdev->dev, "ioremap %llx to %p with size %llx\n",
		 (unsigned long long) regs_res->start,
		 drvdata->base_address,
		 (unsigned long long) (regs_res->end - regs_res->start + 1));

	drvdata->clk = clk_get_sys("PCAP", NULL);
	if (IS_ERR(drvdata->clk)) {
		dev_err(&pdev->dev, "input clock not found\n");
		retval = PTR_ERR(drvdata->clk);
		goto failed4;
	}

	retval = clk_prepare_enable(drvdata->clk);
	if (retval) {
		dev_err(&pdev->dev, "unable to enable clock\n");
		goto failed5;
	}

	/*
	 * Figure out from the device tree if this is running on the EP107
	 * emulation platform as it doesn't match the silicon exactly and the
	 * driver needs to work accordingly.
	 */
	np = of_get_next_parent(pdev->dev.of_node);
	np = of_get_next_parent(np);
	prop = of_get_property(np, "compatible", &size);

	if (prop != NULL) {
		if ((strcmp((const char *)prop, "xlnx,zynq-ep107")) == 0)
			drvdata->ep107 = 1;
		else
			drvdata->ep107 = 0;
	}

	/* Unlock the device */
	xdevcfg_writereg(drvdata->base_address + XDCFG_UNLOCK_OFFSET,
				0x757BDF0D);

	/*
	 * Set the configuration register with the following options
	 *  - Reset FPGA
	 *  - Enable PCAP interface for Partial reconfiguration
	 *  - Enable the PCAP interface
	 *  - Set the throughput rate for maximum speed
	 *  - Se the CPU in user mode
	 */
	ctrlreg = xdevcfg_readreg(drvdata->base_address + XDCFG_CTRL_OFFSET);
	xdevcfg_writereg(drvdata->base_address + XDCFG_CTRL_OFFSET,
				(XDCFG_CTRL_PCFG_PROG_B_MASK |
				XDCFG_CTRL_PCAP_PR_MASK |
				XDCFG_CTRL_PCAP_MODE_MASK |
				XDCFG_CTRL_USER_MODE_MASK |
				ctrlreg));


	cdev_init(&drvdata->cdev, &xdevcfg_fops);
	drvdata->cdev.owner = THIS_MODULE;
	retval = cdev_add(&drvdata->cdev, devt, 1);
	if (retval) {
		dev_err(&pdev->dev, "cdev_add() failed\n");
		goto failed6;
	}

	drvdata->class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(drvdata->class)) {
		dev_err(&pdev->dev, "failed to create class\n");
		goto failed6;
	}

	dev = device_create(drvdata->class, &pdev->dev, devt, drvdata,
			DRIVER_NAME);
	if (IS_ERR(dev)) {
			dev_err(&pdev->dev, "unable to create device\n");
			goto failed7;
	}

	/* create sysfs files for the device */
	retval = sysfs_create_group(&(pdev->dev.kobj), &xdevcfg_attr_group);
	if (retval) {
		dev_err(&pdev->dev, "Failed to create sysfs attr group\n");
		cdev_del(&drvdata->cdev);
		goto failed8;
	}

	clk_disable(drvdata->clk);

	return 0;		/* Success */

failed8:
	device_destroy(drvdata->class, drvdata->devt);
failed7:
	class_destroy(drvdata->class);
failed6:
	clk_disable_unprepare(drvdata->clk);
failed5:
	clk_put(drvdata->clk);
failed4:
	free_irq(irq_res->start, drvdata);
failed3:
	iounmap(drvdata->base_address);
failed2:
	release_mem_region(regs_res->start,
				regs_res->end - regs_res->start + 1);
failed1:
	kfree(drvdata);
failed0:
	/* Unregister char driver */
	unregister_chrdev_region(devt, XDEVCFG_DEVICES);

	return retval;
}

/**
 * xdevcfg_drv_remove -  Remove call for the device.
 *
 * @pdev:	handle to the platform device structure.
 * Returns 0 or error status.
 *
 * Unregister the device after releasing the resources.
 */
static int __devexit xdevcfg_drv_remove(struct platform_device *pdev)
{
	struct xdevcfg_drvdata *drvdata;
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	drvdata = dev_get_drvdata(&pdev->dev);

	if (!drvdata)
		return -ENODEV;

	unregister_chrdev_region(drvdata->devt, XDEVCFG_DEVICES);

	sysfs_remove_group(&pdev->dev.kobj, &xdevcfg_attr_group);

	free_irq(drvdata->irq, drvdata);

	device_destroy(drvdata->class, drvdata->devt);
	class_destroy(drvdata->class);
	cdev_del(&drvdata->cdev);
	iounmap(drvdata->base_address);
	release_mem_region(res->start, res->end - res->start + 1);
	clk_unprepare(drvdata->clk);
	clk_put(drvdata->clk);
	kfree(drvdata);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;		/* Success */
}

static struct of_device_id xdevcfg_of_match[] __devinitdata = {
	{ .compatible = "xlnx,xdevcfg", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, xdevcfg_of_match);

/* Driver Structure */
static struct platform_driver xdevcfg_platform_driver = {
	.probe = xdevcfg_drv_probe,
	.remove = __devexit_p(xdevcfg_drv_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
		.of_match_table = xdevcfg_of_match,
	},
};

module_platform_driver(xdevcfg_platform_driver);

MODULE_AUTHOR("Xilinx, Inc");
MODULE_DESCRIPTION("Xilinx Device Config Driver");
MODULE_LICENSE("GPL");
