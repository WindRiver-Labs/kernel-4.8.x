/*
 *  skl-debug.c - Debugfs for skl driver
 *
 *  Copyright (C) 2015 Intel Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 */

#include <linux/pci.h>
#include <linux/debugfs.h>
#include <sound/soc.h>
#include "skl.h"
#include "skl-nhlt.h"
#include "skl-tplg-interface.h"
#include "skl-topology.h"
#include "skl-sst-dsp.h"
#include "skl-sst-ipc.h"

#define MAX_SSP 4
#define MAX_SZ 1025
#define IPC_MOD_LARGE_CONFIG_GET 3
#define IPC_MOD_LARGE_CONFIG_SET 4
#define MOD_BUF1 (3 * PAGE_SIZE)
#define MOD_BUF (2 * PAGE_SIZE)

#define DEFAULT_SZ 100
#define DEFAULT_ID 0XFF
#define ADSP_PROPERTIES_SZ	0x64
#define ADSP_RESOURCE_STATE_SZ	0x18
#define FIRMWARE_CONFIG_SZ	0x14c
#define HARDWARE_CONFIG_SZ	0x84
#define MODULES_INFO_SZ		0xa70
#define PIPELINE_LIST_INFO_SZ	0xc
#define SCHEDULERS_INFO_SZ	0x34
#define GATEWAYS_INFO_SZ	0x4e4
#define MEMORY_STATE_INFO_SZ	0x1000
#define POWER_STATE_INFO_SZ	0x1000

struct nhlt_blob {
	size_t size;
	struct nhlt_specific_cfg *cfg;
};

struct skl_debug {
	struct skl *skl;
	struct device *dev;

	struct dentry *fs;
	struct dentry *nhlt;
	struct nhlt_blob ssp_blob[2*MAX_SSP];
	struct nhlt_blob dmic_blob;
	struct dentry *modules;
	u32 ipc_data[MAX_SZ];
	struct fw_ipc_data fw_ipc_data;
};

struct nhlt_specific_cfg
*skl_nhlt_get_debugfs_blob(struct skl_debug *d, u8 link_type, u32 instance,
		u8 stream)
{
	switch (link_type) {
	case NHLT_LINK_DMIC:
		return d->dmic_blob.cfg;

	case NHLT_LINK_SSP:
		if (instance >= MAX_SSP)
			return NULL;

		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			return d->ssp_blob[instance].cfg;
		else
			return d->ssp_blob[MAX_SSP + instance].cfg;

	default:
		break;
	}

	dev_err(d->dev, "NHLT debugfs query failed\n");
	return NULL;
}

static ssize_t nhlt_read(struct file *file, char __user *user_buf,
					   size_t count, loff_t *ppos)
{
	struct nhlt_blob *blob = file->private_data;

	if (!blob->cfg)
		return -EIO;

	return simple_read_from_buffer(user_buf, count, ppos,
			blob->cfg, blob->size);
}

static ssize_t nhlt_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct nhlt_blob *blob = file->private_data;
	struct nhlt_specific_cfg *new_cfg;
	ssize_t written;
	size_t size = blob->size;

	if (!blob->cfg) {
		/* allocate mem for blob */
		blob->cfg = kzalloc(count, GFP_KERNEL);
		if (!blob->cfg)
			return -ENOMEM;
		size = count;
	} else if (blob->size < count) {
		/* size if different, so relloc */
		new_cfg = krealloc(blob->cfg, count, GFP_KERNEL);
		if (!new_cfg)
			return -ENOMEM;
		size = count;
		blob->cfg = new_cfg;
	}

	written = simple_write_to_buffer(blob->cfg, size, ppos,
						user_buf, count);
	blob->size = written;

	/* Userspace has been fiddling around behind the kernel's back */
	add_taint(TAINT_USER, LOCKDEP_NOW_UNRELIABLE);

	print_hex_dump(KERN_DEBUG, "Debugfs Blob:", DUMP_PREFIX_OFFSET, 8, 4,
			blob->cfg, blob->size, false);

	return written;
}

static const struct file_operations nhlt_fops = {
	.open = simple_open,
	.read = nhlt_read,
	.write = nhlt_write,
	.llseek = default_llseek,
};

static ssize_t mod_control_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char *state;
	char *buf1;
	int ret;
	unsigned int ofs = 0;

	if (d->ipc_data[0] == 0) {
		state = d->skl->mod_set_get_status ? "Fail\n" : "success\n";
		return simple_read_from_buffer(user_buf, count, ppos,
			state, strlen(state));
	}

	state = d->skl->mod_set_get_status ? "Fail\n" : "success\n";
	buf1 = kzalloc(MOD_BUF1, GFP_KERNEL);
	if (!buf1)
		return -ENOMEM;

	ret = snprintf(buf1, MOD_BUF1,
			"%s\nLARGE PARAM DATA\n", state);

	for (ofs = 0 ; ofs < d->ipc_data[0] ; ofs += 16) {
		ret += snprintf(buf1 + ret, MOD_BUF1 - ret, "0x%.4x : ", ofs);
		hex_dump_to_buffer(&(d->ipc_data[1]) + ofs, 16, 16, 4,
					buf1 + ret, MOD_BUF1 - ret, 0);
		ret += strlen(buf1 + ret);
		if (MOD_BUF1 - ret > 0)
			buf1[ret++] = '\n';
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf1, ret);
	kfree(buf1);
	return ret;

}

static ssize_t mod_control_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	struct mod_set_get *mod_set_get;
	char *buf;
	int retval, type;
	ssize_t written;
	u32 size, mbsz;
	u32 *large_data;
	int large_param_size;

	struct skl_sst *ctx = d->skl->skl_sst;
	struct skl_ipc_large_config_msg msg;
	struct skl_ipc_header header = {0};
	u64 *ipc_header = (u64 *)(&header);

	buf = kzalloc(MOD_BUF, GFP_KERNEL);
	written = simple_write_to_buffer(buf, MOD_BUF, ppos,
						user_buf, count);
	size = written;
	print_hex_dump(KERN_DEBUG, "buf :", DUMP_PREFIX_OFFSET, 8, 4,
			buf, size, false);

	mod_set_get = (struct mod_set_get *)buf;
	header.primary = mod_set_get->primary;
	header.extension = mod_set_get->extension;

	mbsz = mod_set_get->size - (sizeof(u32)*2);
	print_hex_dump(KERN_DEBUG, "header mailbox:", DUMP_PREFIX_OFFSET, 8, 4,
			mod_set_get->mailbx, size-12, false);
	type =  ((0x1f000000) & (mod_set_get->primary))>>24;

	switch (type) {

	case IPC_MOD_LARGE_CONFIG_GET:
		msg.module_id = (header.primary) & 0x0000ffff;
		msg.instance_id = ((header.primary) & 0x00ff0000)>>16;
		msg.large_param_id = ((header.extension) & 0x0ff00000)>>20;
		msg.param_data_size = (header.extension) & 0x000fffff;
		large_param_size = msg.param_data_size;

		large_data = kzalloc(large_param_size, GFP_KERNEL);
		if (!large_data)
			return -ENOMEM;

		if (mbsz)
			retval = skl_ipc_get_large_config(&ctx->ipc, &msg,
				large_data, &(mod_set_get->mailbx[0]), mbsz);
		else
			retval = skl_ipc_get_large_config(&ctx->ipc,
					&msg, large_data, NULL, 0);

		d->ipc_data[0] = msg.param_data_size;
		memcpy(&d->ipc_data[1], large_data, msg.param_data_size);
		kfree(large_data);
		break;

	case IPC_MOD_LARGE_CONFIG_SET:
		d->ipc_data[0] = 0;
		msg.module_id = (header.primary) & 0x0000ffff;
		msg.instance_id = ((header.primary) & 0x00ff0000)>>16;
		msg.large_param_id = ((header.extension) & 0x0ff00000)>>20;
		msg.param_data_size = (header.extension) & 0x000fffff;

		retval = skl_ipc_set_large_config(&ctx->ipc, &msg,
						(u32 *)(&mod_set_get->mailbx));
		d->ipc_data[0] = 0;
		break;

	default:
		if (mbsz)
			retval = sst_ipc_tx_message_wait(&ctx->ipc, *ipc_header,
				mod_set_get->mailbx, mbsz, NULL, 0);

		else
			retval = sst_ipc_tx_message_wait(&ctx->ipc, *ipc_header,
				NULL, 0, NULL, 0);

		d->ipc_data[0] = 0;
		break;

	}
	if (retval)
		d->skl->mod_set_get_status = 1;
	else
		d->skl->mod_set_get_status = 0;

	/* Userspace has been fiddling around behind the kernel's back */
	add_taint(TAINT_USER, LOCKDEP_NOW_UNRELIABLE);
	kfree(buf);
	return written;
}

static const struct file_operations set_get_ctrl_fops = {
	.open = simple_open,
	.read = mod_control_read,
	.write = mod_control_write,
	.llseek = default_llseek,
};

static int skl_init_mod_set_get(struct skl_debug *d)
{
	if (!debugfs_create_file("set_get_ctrl", 0644, d->modules, d,
				 &set_get_ctrl_fops)) {
		dev_err(d->dev, "module set get ctrl debugfs init failed\n");
		return -EIO;
	}
	return 0;
}

static ssize_t nhlt_control_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char *state;

	state = d->skl->nhlt_override ? "enable\n" : "disable\n";
	return simple_read_from_buffer(user_buf, count, ppos,
			state, strlen(state));
}

static ssize_t nhlt_control_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char buf[16];
	int len = min(count, (sizeof(buf) - 1));


	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;
	buf[len] = 0;

	if (!strncmp(buf, "enable\n", len))
		d->skl->nhlt_override = true;
	else if (!strncmp(buf, "disable\n", len))
		d->skl->nhlt_override = false;
	else
		return -EINVAL;

	/* Userspace has been fiddling around behind the kernel's back */
	add_taint(TAINT_USER, LOCKDEP_NOW_UNRELIABLE);

	return len;
}

static const struct file_operations ssp_cntrl_nhlt_fops = {
	.open = simple_open,
	.read = nhlt_control_read,
	.write = nhlt_control_write,
	.llseek = default_llseek,
};

static int skl_init_nhlt(struct skl_debug *d)
{
	int i;
	char name[12];

	if (!debugfs_create_file("control",
				0644, d->nhlt,
				d, &ssp_cntrl_nhlt_fops)) {
		dev_err(d->dev, "nhlt control debugfs init failed\n");
		return -EIO;
	}

	for (i = 0; i < MAX_SSP; i++) {
		snprintf(name, (sizeof(name)-1), "ssp%dp", i);
		if (!debugfs_create_file(name,
					0644, d->nhlt,
					&d->ssp_blob[i], &nhlt_fops))
			dev_err(d->dev, "%s: debugfs init failed\n", name);
		snprintf(name, (sizeof(name)-1), "ssp%dc", i);
		if (!debugfs_create_file(name,
					0644, d->nhlt,
					&d->ssp_blob[MAX_SSP + i], &nhlt_fops))
			dev_err(d->dev, "%s: debugfs init failed\n", name);
	}

	if (!debugfs_create_file("dmic", 0644,
				d->nhlt, &d->dmic_blob,
				&nhlt_fops))
		dev_err(d->dev, "%s: debugfs init failed\n", name);

	return 0;
}

static ssize_t skl_print_pins(struct skl_module_pin *m_pin, char *buf,
				int max_pin, ssize_t ret, bool direction)
{
	int i;

	for (i = 0; i < max_pin; i++)
		ret += snprintf(buf + ret, MOD_BUF - ret,
				"%s%d\n\tModule %d\n\tInstance %d\n\t%s\n\t%s\n\tIndex:%d\n",
				direction ? "Input Pin:" : "Output Pin:",
				i, m_pin[i].id.module_id,
				m_pin[i].id.instance_id,
				m_pin[i].in_use ? "Used" : "Unused",
				m_pin[i].is_dynamic ? "Dynamic" : "Static",
				i);
	return ret;
}

static ssize_t skl_print_fmt(struct skl_module_fmt *fmt, char *buf,
					ssize_t ret, bool direction)
{

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"%s\n\tCH %d\n\tFreq %d\n\tBit %d\tDepth %d\n\tCh config %x\n",
			direction ? "Input Format:" : "Output Format:",
			fmt->channels, fmt->s_freq, fmt->bit_depth,
			fmt->valid_bit_depth, fmt->ch_cfg);

	return ret;
}

static ssize_t module_read(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct skl_module_cfg *mconfig = file->private_data;
	struct skl_module *module = mconfig->module;
	struct skl_module_res *res = &module->resources[mconfig->res_idx];
	struct skl_module_intf *m_intf;
	char *buf;
	ssize_t ret;

	buf = kzalloc(MOD_BUF, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = snprintf(buf, MOD_BUF, "Module\n\tid: %d\n\tinstance id: %d\n",
			mconfig->id.module_id, mconfig->id.instance_id);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Resources\n\tMCPS %x\n\tIBS %x\n\tOBS %x\t\n",
			res->cps, res->ibs, res->obs);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Module data:\n\tCore %d\n\tIN queue %d\n\tOut queue %d\n\t%s\n",
			mconfig->core_id, mconfig->module->max_input_pins,
			mconfig->module->max_output_pins,
			mconfig->module->loadable ? "loadable" : "inbuilt");

	m_intf = &module->formats[mconfig->fmt_idx];
	ret += skl_print_fmt(&m_intf->input[0].pin_fmt, buf, ret, true);
	ret += skl_print_fmt(&m_intf->output[0].pin_fmt, buf, ret, false);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Module Gateway\n\tType %x\n\tInstance %d\n\tHW conn %x\n\tSlot %x\n",
			mconfig->dev_type, mconfig->vbus_id,
			mconfig->hw_conn_type, mconfig->time_slot);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"Pipeline ID\n\t%d\n\tPriority %d\n\tConn Type %d\n\tPages %x\n",
			mconfig->pipe->ppl_id, mconfig->pipe->pipe_priority,
			mconfig->pipe->conn_type, mconfig->pipe->memory_pages);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"\tParams:\n\t\tHost DMA %d\n\t\tLink DMA %d\n",
			mconfig->pipe->p_params->host_dma_id,
			mconfig->pipe->p_params->link_dma_id);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"\tPCM params:\n\t\tCH %d\n\t\tFreq %d\n\t\tFormat %d\n",
			mconfig->pipe->p_params->ch,
			mconfig->pipe->p_params->s_freq,
			mconfig->pipe->p_params->s_fmt);

	ret += snprintf(buf + ret, MOD_BUF - ret,
			"\tLink %x\n\tStream %x\n",
			mconfig->pipe->p_params->linktype,
			mconfig->pipe->p_params->stream);

	ret += skl_print_pins(mconfig->m_in_pin, buf,
			mconfig->module->max_input_pins, ret, true);
	ret += skl_print_pins(mconfig->m_out_pin, buf,
			mconfig->module->max_output_pins, ret, false);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);

	kfree(buf);
	return ret;
}

static const struct file_operations mcfg_fops = {
	.open = simple_open,
	.read = module_read,
	.llseek = default_llseek,
};


void skl_debug_init_module(struct skl_debug *d,
			struct snd_soc_dapm_widget *w,
			struct skl_module_cfg *mconfig)
{
	if (!debugfs_create_file(w->name, 0444,
				d->modules, mconfig,
				&mcfg_fops))
		dev_err(d->dev, "%s: module debugfs init failed\n", w->name);
}

static ssize_t adsp_control_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{

	struct skl_debug *d = file->private_data;
	char *buf1;
	ssize_t ret;
	unsigned int data, ofs = 0;
	int replysz = 0;

	mutex_lock(&d->fw_ipc_data.mutex);
	replysz = d->fw_ipc_data.replysz;
	data = d->fw_ipc_data.adsp_id;

	buf1 = kzalloc(MOD_BUF1, GFP_ATOMIC);
	if (!buf1) {
		mutex_unlock(&d->fw_ipc_data.mutex);
		return -ENOMEM;
	}

	ret = snprintf(buf1, MOD_BUF1,
			"\nADSP_PROP ID %x\n", data);
	for (ofs = 0 ; ofs < replysz ; ofs += 16) {
		ret += snprintf(buf1 + ret, MOD_BUF1 - ret,
			"0x%.4x : ", ofs);
		hex_dump_to_buffer((u8 *)(&(d->fw_ipc_data.mailbx[0])) + ofs,
					16, 16, 4,
					buf1 + ret, MOD_BUF1 - ret, 0);
		ret += strlen(buf1 + ret);
		if (MOD_BUF1 - ret > 0)
			buf1[ret++] = '\n';
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf1, ret);
	mutex_unlock(&d->fw_ipc_data.mutex);
	kfree(buf1);

	return ret;
}

static ssize_t adsp_control_write(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct skl_debug *d = file->private_data;
	char buf[8];
	int err, replysz;
	unsigned int dsp_property;
	u32 *ipc_data;
	struct skl_sst *ctx = d->skl->skl_sst;
	struct skl_ipc_large_config_msg msg;
	char id[8];
	u32 tx_data;
	int j = 0, bufsize, tx_param = 0, tx_param_id;
	int len = min(count, (sizeof(buf)-1));

	mutex_lock(&d->fw_ipc_data.mutex);
	if (copy_from_user(buf, user_buf, len)) {
		mutex_unlock(&d->fw_ipc_data.mutex);
		return -EFAULT;
	}

	buf[len] = '\0';
	bufsize = strlen(buf);

	while (buf[j] != '\0') {
		if (buf[j] == ',') {
			strncpy(id, &buf[j+1], (bufsize-j));
			buf[j] = '\0';
			tx_param = 1;
		} else
			j++;
	}

	err = kstrtouint(buf, 10, &dsp_property);

	if ((dsp_property == DMA_CONTROL) || (dsp_property == ENABLE_LOGS)) {
		dev_err(d->dev, "invalid input !! not readable\n");
		mutex_unlock(&d->fw_ipc_data.mutex);
		return -EINVAL;
	}

	if (tx_param == 1) {
		err = kstrtouint(id, 10, &tx_param_id);
		tx_data = (tx_param_id << 8) | dsp_property;
	}

	ipc_data = kzalloc(DSP_BUF, GFP_ATOMIC);
	if (!ipc_data) {
		mutex_unlock(&d->fw_ipc_data.mutex);
		return -ENOMEM;
	}

	switch (dsp_property) {

	case ADSP_PROPERTIES:
	replysz = ADSP_PROPERTIES_SZ;
	break;

	case ADSP_RESOURCE_STATE:
	replysz = ADSP_RESOURCE_STATE_SZ;
	break;

	case FIRMWARE_CONFIG:
	replysz = FIRMWARE_CONFIG_SZ;
	break;

	case HARDWARE_CONFIG:
	replysz = HARDWARE_CONFIG_SZ;
	break;

	case MODULES_INFO:
	replysz = MODULES_INFO_SZ;
	break;

	case PIPELINE_LIST_INFO:
	replysz = PIPELINE_LIST_INFO_SZ;
	break;

	case SCHEDULERS_INFO:
	replysz = SCHEDULERS_INFO_SZ;
	break;

	case GATEWAYS_INFO:
	replysz = GATEWAYS_INFO_SZ;
	break;

	case MEMORY_STATE_INFO:
	replysz = MEMORY_STATE_INFO_SZ;
	break;

	case POWER_STATE_INFO:
	replysz = POWER_STATE_INFO_SZ;
	break;

	default:
	mutex_unlock(&d->fw_ipc_data.mutex);
	kfree(ipc_data);
	return -EINVAL;
	}

	msg.module_id = 0x0;
	msg.instance_id = 0x0;
	msg.large_param_id = dsp_property;
	msg.param_data_size = replysz;

	if (tx_param == 1)
		skl_ipc_get_large_config(&ctx->ipc, &msg,
				ipc_data, &tx_data, sizeof(u32));
	else
		skl_ipc_get_large_config(&ctx->ipc, &msg,
							ipc_data, NULL, 0);

	memset(&d->fw_ipc_data.mailbx[0], 0, DSP_BUF);

	memcpy(&d->fw_ipc_data.mailbx[0], ipc_data, replysz);

	d->fw_ipc_data.adsp_id = dsp_property;

	d->fw_ipc_data.replysz = replysz;

	/* Userspace has been fiddling around behindthe kernel's back*/
	add_taint(TAINT_USER, LOCKDEP_NOW_UNRELIABLE);
	mutex_unlock(&d->fw_ipc_data.mutex);
	kfree(ipc_data);

	return len;
}

static const struct file_operations ssp_cntrl_adsp_fops = {
	.open = simple_open,
	.read = adsp_control_read,
	.write = adsp_control_write,
	.llseek = default_llseek,
};

static int skl_init_adsp(struct skl_debug *d)
{
	if (!debugfs_create_file("adsp_prop_ctrl", 0644, d->fs, d,
				 &ssp_cntrl_adsp_fops)) {
		dev_err(d->dev, "adsp control debugfs init failed\n");
		return -EIO;
	}

	memset(&d->fw_ipc_data.mailbx[0], 0, DSP_BUF);
	d->fw_ipc_data.replysz = DEFAULT_SZ;
	d->fw_ipc_data.adsp_id = DEFAULT_ID;

	return 0;
}

struct skl_debug *skl_debugfs_init(struct skl *skl)
{
	struct skl_debug *d;

	d = devm_kzalloc(&skl->pci->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return NULL;

	mutex_init(&d->fw_ipc_data.mutex);

	/* create the root dir first */
	d->fs = debugfs_create_dir("snd_soc_skl", NULL);
	if (IS_ERR(d->fs) || !d->fs) {
		dev_err(&skl->pci->dev, "debugfs root creation failed\n");
		return NULL;
	}

	d->skl = skl;
	d->dev = &skl->pci->dev;

	/* now create the NHLT dir */
	d->nhlt =  debugfs_create_dir("nhlt", d->fs);
	if (IS_ERR(d->nhlt) || !d->nhlt) {
		dev_err(&skl->pci->dev, "nhlt debugfs create failed\n");
		goto err;
	}

	/* now create the module dir */
	d->modules =  debugfs_create_dir("modules", d->fs);
	if (IS_ERR(d->modules) || !d->modules) {
		dev_err(&skl->pci->dev, "modules debugfs create failed\n");
		goto err;
	}

	skl_init_nhlt(d);
	skl_init_adsp(d);
	skl_init_mod_set_get(d);

	return d;

err:
	debugfs_remove_recursive(d->fs);
	return NULL;
}

void skl_debugfs_exit(struct skl_debug *d)
{
	int i;

	debugfs_remove_recursive(d->fs);

	/* free blob memory, if allocated */
	for (i = 0; i < MAX_SSP; i++) {
		kfree(d->ssp_blob[i].cfg);
		kfree(d->ssp_blob[MAX_SSP + i].cfg);
	}
	kfree(d->dmic_blob.cfg);
	mutex_destroy(&d->fw_ipc_data.mutex);

	kfree(d);

}
