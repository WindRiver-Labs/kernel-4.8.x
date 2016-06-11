/*
 * Texas Instruments Keystone SerDes driver
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * This is the Rapidio SerDes driver for Keystone devices. This is
 * required to support RapidIO functionality on K2HK devices.
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef KEYSTONE_RIO_SERDES_H
#define KEYSTONE_RIO_SERDES_H

#define KEYSTONE_SERDES_TYPE_K1      0
#define KEYSTONE_SERDES_TYPE_K2      1

#define KEYSTONE_SERDES_LANE(lane)   (BIT(lane))
#define KEYSTONE_SERDES_MAX_LANES    4
#define KEYSTONE_SERDES_LANE_MASK    0xf

#define KEYSTONE_SERDES_TIMEOUT      100

#define KEYSTONE_SERDES_BAUD_1_250   0
#define KEYSTONE_SERDES_BAUD_2_500   1
#define KEYSTONE_SERDES_BAUD_3_125   2
#define KEYSTONE_SERDES_BAUD_5_000   3
#define KEYSTONE_SERDES_BAUD_6_250   4

#define KEYSTONE_SERDES_QUARTER_RATE 0
#define KEYSTONE_SERDES_HALF_RATE    1
#define KEYSTONE_SERDES_FULL_RATE    2

struct keystone_serdes_lane_tx_config {
	u32 pre_1lsb;
	u32 c1_coeff;
	u32 c2_coeff;
	u32 cm_coeff;
	u32 att;
	u32 vreg;
	u32 vdreg;
};

struct keystone_serdes_lane_rx_config {
	u32 att;
	u32 boost;
	u32 mean_att;
	u32 mean_boost;
	u32 start_att;
	u32 start_boost;
};

struct keystone_serdes_config {
	u16 prescalar_srv_clk;

	struct keystone_serdes_lane_tx_config tx[KEYSTONE_SERDES_MAX_LANES];
	struct keystone_serdes_lane_rx_config rx[KEYSTONE_SERDES_MAX_LANES];

	u32 cal_timeout;
	int do_phy_init_cfg;
	u32 rate;
};

struct keystone_serdes_data;

struct keystone_serdes_ops {
	int (*config_lanes)(u32 lanes, u32 baud,
			    struct keystone_serdes_data *serdes);
	int (*start_tx_lanes)(u32 lanes, struct keystone_serdes_data *serdes);
	int (*wait_lanes_ok)(u32 lanes, struct keystone_serdes_data *serdes);
	int (*disable_lanes)(u32 lanes, struct keystone_serdes_data *serdes);
	int (*shutdown_lanes)(u32 lanes, struct keystone_serdes_data *serdes);
	void (*recover_lanes)(u32 lanes, struct keystone_serdes_data *serdes);
	int (*calibrate_lanes)(u32 lanes, struct keystone_serdes_data *serdes);
};

struct keystone_serdes_data {
	const struct keystone_serdes_ops	*ops;
	struct device				*dev;
	void __iomem				*regs;
	void __iomem				*sts_reg;
	struct keystone_serdes_config		*config;
	struct kobject				*serdes_kobj;
	struct kobject				serdes_rx_kobj;
	struct kobject				serdes_tx_kobj;
};

int keystone_rio_serdes_register(u16 serdes_type,
				 void __iomem *regs,
				 void __iomem *sts_reg,
				 struct device *dev,
				 struct keystone_serdes_data *serdes,
				 struct keystone_serdes_config *serdes_config);

void keystone_rio_serdes_unregister(struct device *dev,
				    struct keystone_serdes_data *serdes);

#endif
