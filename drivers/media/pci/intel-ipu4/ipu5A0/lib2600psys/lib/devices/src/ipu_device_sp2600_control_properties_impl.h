/**
* Support for Intel Camera Imaging ISP subsystem.
 * Copyright (c) 2010 - 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
*/

#ifndef _IPU_DEVICE_SP2600_CONTROL_PROPERTIES_IMPL_H_
#define _IPU_DEVICE_SP2600_CONTROL_PROPERTIES_IMPL_H_

/* sp2600_control definition */

#include "ipu_device_cell_properties_struct.h"

enum ipu_device_sp2600_control_registers {
	/* control registers */
	IPU_DEVICE_SP2600_CONTROL_STAT_CTRL      = 0x0,
	IPU_DEVICE_SP2600_CONTROL_START_PC       = 0x4,

	/* master port registers */
	IPU_DEVICE_SP2600_CONTROL_ICACHE_BASE    = 0x10,
	IPU_DEVICE_SP2600_CONTROL_ICACHE_INFO    = 0x14,
	IPU_DEVICE_SP2600_CONTROL_ICACHE_INFO_OVERRIDE    = 0x18,

	IPU_DEVICE_SP2600_CONTROL_QMEM_BASE      = 0x1C,

	IPU_DEVICE_SP2600_CONTROL_CMEM_BASE      = 0x28,
	IPU_DEVICE_SP2600_CONTROL_CMEM_INFO      = 0x2C,
	IPU_DEVICE_SP2600_CONTROL_CMEM_INFO_OVERRIDE      = 0x30,

	IPU_DEVICE_SP2600_CONTROL_XMEM_BASE      = 0x58,
	IPU_DEVICE_SP2600_CONTROL_XMEM_INFO      = 0x5C,
	IPU_DEVICE_SP2600_CONTROL_XMEM_INFO_OVERRIDE      = 0x60,

	/* debug registers */
	IPU_DEVICE_SP2600_CONTROL_DEBUG_PC       = 0x9C,
	IPU_DEVICE_SP2600_CONTROL_STALL          = 0xA0
};

enum ipu_device_sp2600_control_mems {
	IPU_DEVICE_SP2600_CONTROL_REGS,
	IPU_DEVICE_SP2600_CONTROL_PMEM,
	IPU_DEVICE_SP2600_CONTROL_DMEM,
	IPU_DEVICE_SP2600_CONTROL_NUM_MEMORIES
};

static const unsigned int
ipu_device_sp2600_control_mem_size[IPU_DEVICE_SP2600_CONTROL_NUM_MEMORIES] = {
	0x000AC,
	0x00000,
	0x10000
};

enum ipu_device_sp2600_control_masters {
	IPU_DEVICE_SP2600_CONTROL_ICACHE,
	IPU_DEVICE_SP2600_CONTROL_QMEM,
	IPU_DEVICE_SP2600_CONTROL_CMEM,
	IPU_DEVICE_SP2600_CONTROL_XMEM,
	IPU_DEVICE_SP2600_CONTROL_NUM_MASTERS
};

static const struct ipu_device_cell_master_properties_s
ipu_device_sp2600_control_masters[IPU_DEVICE_SP2600_CONTROL_NUM_MASTERS] = {
	{
		0,
		0xC,
		IPU_DEVICE_SP2600_CONTROL_ICACHE_BASE,
		IPU_DEVICE_SP2600_CONTROL_ICACHE_INFO,
		IPU_DEVICE_SP2600_CONTROL_ICACHE_INFO_OVERRIDE
	},
	{
		0,
		0xC,
		IPU_DEVICE_SP2600_CONTROL_QMEM_BASE,
		0xFFFFFFFF,
		0xFFFFFFFF
	},
	{
		2,
		0xC,
		IPU_DEVICE_SP2600_CONTROL_CMEM_BASE,
		IPU_DEVICE_SP2600_CONTROL_CMEM_INFO,
		IPU_DEVICE_SP2600_CONTROL_CMEM_INFO_OVERRIDE
	},
	{
		2,
		0xC,
		IPU_DEVICE_SP2600_CONTROL_XMEM_BASE,
		IPU_DEVICE_SP2600_CONTROL_XMEM_INFO,
		IPU_DEVICE_SP2600_CONTROL_XMEM_INFO_OVERRIDE
	}
};

enum ipu_device_sp2600_control_stall_bits {
	IPU_DEVICE_SP2600_CONTROL_STALL_ICACHE,
	IPU_DEVICE_SP2600_CONTROL_STALL_DMEM,
	IPU_DEVICE_SP2600_CONTROL_STALL_QMEM,
	IPU_DEVICE_SP2600_CONTROL_STALL_CMEM,
	IPU_DEVICE_SP2600_CONTROL_STALL_XMEM,
	IPU_DEVICE_SP2600_CONTROL_NUM_STALL_BITS
};

/* 32 bits per instruction */
#define IPU_DEVICE_SP2600_CONTROL_ICACHE_WORD_SIZE 4
/* 32 instructions per burst */
#define IPU_DEVICE_SP2600_CONTROL_ICACHE_BURST_SIZE 32

static const struct ipu_device_cell_count_s ipu_device_sp2600_control_count = {
	IPU_DEVICE_SP2600_CONTROL_NUM_MEMORIES,
	IPU_DEVICE_SP2600_CONTROL_NUM_MASTERS,
	IPU_DEVICE_SP2600_CONTROL_NUM_STALL_BITS,
	IPU_DEVICE_SP2600_CONTROL_ICACHE_WORD_SIZE *
	IPU_DEVICE_SP2600_CONTROL_ICACHE_BURST_SIZE
};

static const unsigned int
ipu_device_sp2600_control_reg_offset[/* CELL_NUM_REGS */] = {
	0x0, 0x4, 0x10, 0x9C, 0xA0
};

static const struct ipu_device_cell_type_properties_s
ipu_device_sp2600_control_properties = {
	&ipu_device_sp2600_control_count,
	ipu_device_sp2600_control_masters,
	ipu_device_sp2600_control_reg_offset,
	ipu_device_sp2600_control_mem_size
};

#endif /* _IPU_DEVICE_SP2600_CONTROL_PROPERTIES_IMPL_H_ */
