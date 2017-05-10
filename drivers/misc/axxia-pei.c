/*
 *  Copyright (C) 2016 Intel Corporation
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 */

/*
  ==============================================================================
  ==============================================================================
  Private
  ==============================================================================
  ==============================================================================
*/

#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/lsi-ncr.h>

static int is_5500;
static int is_5600;
static int is_6700;

static void __iomem *pcie_gpreg0;
static void __iomem *pcie_rc;

enum SataMode {
	SATA0,
	SATA1
};
enum SataSpeed {
	SATA_SPEED_1_5G,
	SATA_SPEED_3G,
	SATA_SPEED_6G
};
enum SrioMode {
	SRIO0,
	SRIO1
};
enum SrioSpeed {
	SRIO_SPEED_1_25G,
	SRIO_SPEED_2_5G,
	SRIO_SPEED_3_125G,
	SRIO_SPEED_5G,
	SRIO_SPEED_6_25G
};
enum PLLMode {
	PLLA,
	PLLB
};
enum PowerState {
	P0,
	P1,
	P2
};

enum Dir {
	TX,
	RX
};

/******************************************************************************
 * PCIe/SRIO/SATA parameters
 * Supported configs:
 *	PEI0x8
 *	PEI0x4_PEI1x4
 *	PEI0x4_PEI1x2_SATA0x1_SATA1x1
 *	PEI0x2_PEI2x2_PEI1x2_SATA0x1_SATA1x1
 *	PEI0x2_SRIO0x2_PEI1x4
 *	PEI0x2_SRIO0x2_PEI1x2_SATA0x1_SATA1x1
 *	PEI0x2_SRIO0x2_PEI1x2_PEI2x2
 *	SRIO1x2_SRIO0x2_PEI1x4
 *	SRIO1x2_SRIO0x2_PEI1x2_SATA0x1_SATA1x1
 *	SRIO1x2_SRIO0x2_PEI1x2_PEI2x2
 *
 * Bits 25:22 :Supported configs:
 *	0x0 -	PEI0x8 (HSS10-ch0,1;HSS11-ch0,1;HSS12-ch0,1;HSS13-ch0,1)
 *	0x1 -	PEI0x4 (HSS10-ch0,1; HSS11-ch0,1)
 *		PEI1x4 (HSS12-ch0,1; HSS12-ch0,1)
 *	0x2 -	PEI0x4 (HSS10-ch0,1; HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		SATA0x1 (HSS13-ch0)
 *		SATA1x1 (HSS13-ch1)
 *	0x3 -	PEI0x2 (HSS10-ch0,1)
 *		PEI2x2 (HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		SATA0x1 (HSS13-ch0)
 *		SATA1x1 (HSS13-ch1)
 *	0x4 -	PEI0x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x4 (HSS12-ch0,1; HSS13-ch0,1)
 *	0x5 -	PEI0x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		SATA0x1 (HSS13-ch0)
 *		SATA1x1 (HSS13-ch1)
 *	0x6 -	PEI0x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		PEI2x2 (HSS13-ch0,1)
 *	0x7 -	SRIO1x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x4 (HSS12-ch0,1; HSS13-ch0,1)
 *	0x8 -	SRIO1x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		SATA0x1 (HSS13-ch0)
 *		SATA1x1 (HSS13-ch1)
 *	0x9 -	SRIO1x2 (HSS10-ch0,1)
 *		SRIO0x2 (HSS11-ch0,1)
 *		PEI1x2 (HSS12-ch0,1)
 *		PEI2x2 (HSS13-ch0,1)
 * Bits 21:20: SATA1 speed selection
 *	0x0 - indicates 1.5 Gbps
 *	0x1 - indicates 3 Gbps
 *	0x2 - indicates 6 Gbps
 * Bits 19:18: SATA0 speed selection
 *	0x0 - indicates 1.5 Gbps
 *	0x1 - indicates 3 Gbps
 *	0x2 - indicates 6 Gbps
 * Bits 17:15 : SRIO1 speed selection
 *	0x0 -  indicates 1.25 Gbps
 *	0x1 -  indicates 2.5 Gbps
 *	0x2 -  indicates 3.125 Gbps
 *	0x3 -  indicates 5 Gbps
 *	0x4 -  indicates 6.25 Gbps
 * Bits 14:12 : SRIO0 speed selection
 *	0x0 -  indicates 1.25 Gbps
 *	0x1 -  indicates 2.5 Gbps
 *	0x2 -  indicates 3.125 Gbps
 *	0x3 -  indicates 5 Gbps
 *	0x4 -  indicates 6.25 Gbps
 * Bits 11:10 : If SRIO1 is configured,
 *	0x0 - indicates Host with ID 0,
 *	0x1 - indicates Host with ID 1,
 *	0x2 - indicates not a host (agent)
 *	0x3 - RESERVED
 * Bits 9:8 : If SRIO0 is configured,
 *	0x0 - indicates Host with ID 0,
 *	0x1 - indicates Host with ID 1,
 *	0x2 - indicates not a host (agent)
 *	0x3 - RESERVED
 * Bit 7: If PEI0 is configured,
 *	0x0 - indicates END_POINT
 *	0x1 - indicates ROOT_COMPLEX
 * Bit 6: SATA1 enabled
 * Bit 5: SATA0 enabled
 * Bit 4: SRIO1 enabled
 * Bit 3: SRIO0 enabled
 * Bit 2: PEI2 enabled
 * Bit 1: PEI1 enabled
 * Bit 0: PEI0 enabled
 *******************************************************************************/

void setup_sata_mode(enum SataMode mode, enum SataSpeed speed)
{
	u32 regVal, speedVal = 0, laneid;

	if (mode == SATA0)
		laneid = 6;
	else
		laneid = 7;

	/* setup Tx word mode */
	/* 20 bit mode */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c)), &regVal);
	regVal &= 0xffffffcf;
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c)), regVal);
	/* setup Rx word mode */
	/* 20 bit mode */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0xc),
		   &regVal);
	regVal &= 0xfffffcff;
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0xc),
		    regVal);
	/* set gearbox word clk sel lane 1 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x14),
		   &regVal);
	regVal |= (0x1) << 20;
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x14),
		    regVal);
	/* config tx lane 6 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c)), &regVal);
	/* Disable TX swing boost */
	regVal &= 0xffff07ff;
	/* TX swing level */
	regVal |= (15 << 12);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c)), regVal);
	/* Program SATA speed */
	if (speed == SATA_SPEED_1_5G)
		speedVal = 20;
	else if (speed == SATA_SPEED_3G)
		speedVal = 23;
	else if (speed == SATA_SPEED_6G)
		speedVal = 29;

	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x8),
		    speedVal);

	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x4),
		   &regVal);
	regVal &= 0xffec8fcf;
	/* Set power state of the transmitter
	 * select baud rate
	 * MPLLB select disable
	 * Enable TX MPLL
	 * 20 bit width of TX data
	 */
	regVal |= (3 << 4) | (1 << 12) | (3 << 16) | (0 << 20) | (1 << 22);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x4),
		    regVal);

	/* config rx lane 6 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0xc),
		   &regVal);
	/* Disable LFPS filter
	 * Disable word alignment
	 * Disable divide by 16.5 recovered clock
	 */
	regVal &= 0xff05ff87;
	/* LOS threshold set to 120mVpp
	 * Enable CDR tracking of RX data
	 * Enable CDR tracking gains and duration
	 * Set Receiver VCO lower frequency band mode
	 * Grounded RX termination
	 */
	regVal |= ((0x2 << 4) | (1 << 17) | (1 << 20) | (1 << 22) | (1 << 23));
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0xc),
		    regVal);

	/* RX VCO calibration load and reference load value */
	regVal = (12 << 0) | (1440 << 8);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x10),
		    regVal);

	/* Set AFE attenuation level to -2dB
	 * Set the AFE first and second stage varialble Gain Amplifier gain
	 * Set the continous time linear equalizer CTLE boost pole location
	 * Set the CTLE boost level
	 * Set the value of DFE data tap 1
	 */
	regVal = ((0 << 0) | (7 << 4) | (7 << 8) | (0 << 12) | (6 << 16) |
		  (0 << 24));
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x18),
		    regVal);
	/* Program SATA speed */
	if (speed == SATA_SPEED_1_5G)
		speedVal = 3;
	else if (speed == SATA_SPEED_3G)
		speedVal = 2;
	else if (speed == SATA_SPEED_6G)
		speedVal = 1;
	ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x14),
		   &regVal);
	/* Disable AFE/DFE */
	regVal &= 0xff2ccfcf;
	regVal |= (3 << 4) | (speedVal << 12) | (3 << 16);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (laneid * 0x1c) + 0x14),
		    regVal);

	/* PLLA/SATA */
	/*
	 *	ref_clk_en                  = 1'b1;
	 *	ref_use_pad                 = 1'b1;
	 *	ref_clk_div2_en             = 1'b0;
	 *	ref_range                   = 3'h3;
	 *	ref_clk_mplla_div2_en       = 1'b1;
	 *	ref_clk_mpllb_div2_en       = 1'b1;
	 *	ref_repeat_clk_en           = 1'b0;
	 *	tx_vboost_lvl               = 3'h5;
	 *	vref_ctrl                   = 5'h11;
	 */

	/* Enable ref clk, ref use_pad, ref_clk_div2_en */
	regVal = ((0x1) | ((0x1) << 1) | (0x1 << 2));
	/* Select ref range 78.1-104 MHz */
	regVal |= (0x3 << 4);
	/* Enable ref_clk_mplla_div2_en, ref_clk_mpllb_div2_en
	 * and ref_repeat_clk_en */
	regVal |= ((0x1 << 8) | (0x1 << 9) | (0x1 << 10));
	/* Set tx_vboost_lvl and vref_ctrl */
	regVal |= ((0x3 << 16) | (17 << 20));
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18)),
		    regVal);

	/* MPLLA setting
	 * mplla_ssc_en                = 1'b1;
	 * mplla_div_clk_en            = 1'b0;
	 * mplla_ssc_range             = 3'h0;
	 * mplla_ssc_clk_sel   = 2'h2;
	 * mplla_div_multiplier        = 7'h0;
	 */
	/* MPLLA spread-spectrum clock (SSC) and
	 * div_clk enable */
	regVal = (0x1 | (0x1 << 2));
	/*  SSC range  and SSC clk select */
	regVal |= (2 << 20);
	/* mplla_div_multiplier */
	regVal |= (7 << 24);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x8),
		    regVal);

	/* MPLLA settings
	 * mplla_force_en              = 1'b0;
	 * mplla_div8_clk_en   = 1'b0;
	 * mplla_div10_clk_en  = 1'b1;
	 * mplla_div16p5_clk_en        = 1'b0;
	 * mplla_multiplier            = 7'h3c;
	 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x4),
		   &regVal);
	/* Enable MPLLA force, mplla_div8_clk_en, mplla_div10_clk_en
	 * and mplla_div16p5_clk_en */
	/* MPLLA multiplier is 0 */
	regVal |= ((0x1) | (0x1 << 1) | (0x1 << 2) |
		   (0x1 << 3));
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x4),
		    regVal);

	/*
	 * mplla_fracn_ctrl            = 9'h0;
	 * mplla_bandwidth             = 7'h2a;
	 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x10),
		   &regVal);
	regVal &= 0xffff0000;
	regVal |= (0x42 << 9);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (3 * 0x18) + 0x10),
		    regVal);
}

void setup_srio_mode(enum SrioMode mode, enum SrioSpeed speed)
{
	u32 regVal;
	u32 phy, lane, rate, width, pll;
	int i = 0;
	if (mode == SRIO0) {
		phy = 1;
		lane = 2;
	} else {
		phy = 0;
		lane = 0;
	}

	/* set wordmode to bypass for lane 0 and lane 1*/
	for (i = lane; i <= (lane + 1); i++) {
		/* set Tx word mode */
		ncr_read32(NCP_REGION_ID(0x115, 0), (0x18 + (i * 0x1c)),
			   &regVal);
		regVal &= (~(3 << 4));
		regVal |= (0 << 4);
		/* TX iboost level */
		regVal |= (15 << 12);
		ncr_write32(NCP_REGION_ID(0x115, 0), (0x18 + (i * 0x1c)),
			    regVal);

		/* control for setting the transmitter driver
		 * output pre-emphasis/post-emphasis and amplitude
		 */
		regVal = ((0 << 8) | (0 << 16) | (40 << 0) | (1 << 28));
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0x18 + (i * 0x1c) + 0x8),
			    regVal);
		/* Tx width */
		ncr_read32(NCP_REGION_ID(0x115, 0),
			   (0x18 + (i * 0x1c) + 0x4), &regVal);
		regVal &= 0xffac8fcf;
		/* SEt the power state of the transmitter
		 * TX rate/width
		 */
		rate = (speed == SRIO_SPEED_1_25G) ? 3 :
			((speed == SRIO_SPEED_2_5G) ||
			 (speed == SRIO_SPEED_3_125G)) ? 2 : 1;
		width = 1;
		pll = 0;
		pll = ((speed == SRIO_SPEED_3_125G) ||
		       (speed == SRIO_SPEED_6_25G)) ? 1 : 0;
		regVal |= ((0x3 << 4) | (rate << 12) | (width << 16)
			   | (pll << 20) | (1 << 22));
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0x18 + (i * 0x1c) + 0x4), regVal);

		if ((speed == SRIO_SPEED_3_125G) || (speed == SRIO_SPEED_6_25G))
			regVal = (0x550 << 8) | 0x22;
		else
			regVal = (0x540 << 8) | 0x2a;
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0x18 + (i * 0x1c) + 0x10), regVal);
		ncr_read32(NCP_REGION_ID(0x115, 0),
			   (0x18 + (i * 0x1c) + 0x14), &regVal);
		regVal &= 0xff3c8fcf;
		regVal |= ((0x3 << 4) | (rate << 12) | (width << 16));
		if ((speed == SRIO_SPEED_5G) || (speed == SRIO_SPEED_6_25G))
			regVal |= ((1 << 22) | (1 << 23));
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0x18 + (i * 0x1c) + 0x14), regVal);

	}
	/* set gearbox word clk sel for phy 1 lane 0/lane 1 */
	ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x14),
		   &regVal);
	regVal &= (~(0x7 << 20));
	if (speed == SRIO_SPEED_1_25G)
		regVal |= (0x2) << 20;
	else if ((speed == SRIO_SPEED_2_5G) || (speed == SRIO_SPEED_3_125G))
		regVal |= (0x1) << 20;
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x14),
		    regVal);
	ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x14),
		   &regVal);
	regVal &= (~(0x7 << 24));
	if (speed == SRIO_SPEED_1_25G)
		regVal |= (0x2) << 24;
	else if ((speed == SRIO_SPEED_2_5G) || (speed == SRIO_SPEED_3_125G))
		regVal |= (0x1) << 24;
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x14),
		    regVal);
	/* PLLA Settings */
	/* Enable ref clk, ref use_pad, ref_clk_div2_en */
	regVal = ((0x1) | ((0x1) << 1) | (0 << 2));
	/* Select ref range 78.1-104 MHz */
	regVal |= (0x6 << 4);
	/* Enable ref_clk_mplla_div2_en, ref_clk_mpllb_div2_en
	 * and ref_repeat_clk_en */
	regVal |= ((0x1 << 8) | (0x1 << 9) | (0 << 10));
	/* Set tx_vboost_lvl and vref_ctrl */
	regVal |= ((0x5 << 16) | (17 << 20));
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18)),
		    regVal);

	/* MPLLA setting
	 * mplla_ssc_en                = 1'b0;
	 * mplla_div_clk_en            = 1'b0;
	 * mplla_ssc_range             = 3'h0;
	 * mplla_ssc_clk_sel   = 2'h2;
	 * mplla_div_multiplier        = 7'h0;
	 */
	/* MPLLA spread-spectrum clock (SSC) and
	 * div_clk enable */
	regVal = (0 | (0 << 2));
	/*  SSC range  and SSC clk select */
	regVal |= (0 << 20);
	/* mplla_div_multiplier */
	regVal |= (0 << 24);
	ncr_write32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x8),
		    regVal);

	switch (speed) {
	case SRIO_SPEED_1_25G:
	case SRIO_SPEED_2_5G:
	case SRIO_SPEED_5G:
		/* 500 MHz PLLA */
		/* MPLLA settings
		 * mplla_force_en              = 1'b0;
		 * mplla_div8_clk_en   = 1'b0;
		 * mplla_div10_clk_en  = 1'b1;
		 * mplla_div16p5_clk_en        = 1'b0;
		 * mplla_multiplier            = 7'h20;
		 */
		ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x4),
			   &regVal);
		/* Disable MPLLA force, mplla_div8_clk_en
		 * Enable mplla_div10_clk_en
		 * and Disable mplla_div16p5_clk_en */
		/* MPLLA multiplier is 32 */
		regVal |= ((0x1 << 2) | (0x20 << 4));
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0xf8 + (phy * 0x18) + 0x4),
			    regVal);
		/*
		 * mplla_fracn_ctrl            = 9'h0;
		 * mplla_bandwidth             = 7'h3a;
		 */
		ncr_read32(NCP_REGION_ID(0x115, 0),
			   (0xf8 + (phy * 0x18) + 0x10),
			   &regVal);
		regVal &= 0xffff0000;
		regVal |= (0x3a << 9);
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0xf8 + (phy * 0x18) + 0x10),
			    regVal);
		break;
	case SRIO_SPEED_3_125G:
	case SRIO_SPEED_6_25G:
		/* 625 MHz PLLB */
		/* MPLLB settings
		 * mpllb_force_en              = 1'b0;
		 * mpllb_div8_clk_en   = 1'b0;
		 * mpllb_div10_clk_en  = 1'b1;
		 * mpllb_div16p5_clk_en        = 1'b0;
		 * mpllb_multiplier            = 7'h28;
		 */
		ncr_read32(NCP_REGION_ID(0x115, 0), (0xf8 + (phy * 0x18) + 0x4),
			   &regVal);
		/* Disable MPLLB force, mpllb_div8_clk_en
		 * Enable mpllb_div10_clk_en
		 * and Disable mpllb_div16p5_clk_en */
		/* MPLLB multiplier is 40 */
		regVal |= (((0x1 << 2) | (0x28 << 4))<<16);
		regVal |= ((0x1 << 2) | (0x20 << 4)); /* Is this needed*/
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0xf8 + (phy * 0x18) + 0x4), regVal);
		/*
		 * mplla_fracn_ctrl            = 9'h0;
		 * mplla_bandwidth             = 7'h5b;
		 */
		ncr_read32(NCP_REGION_ID(0x115, 0),
			   (0xf8 + (phy * 0x18) + 0x10),
			   &regVal);
		regVal &= 0xffff;
		regVal |= ((0x5b << 9)<<16);
		regVal |= (0x3a << 9); /* Is this needed*/
		ncr_write32(NCP_REGION_ID(0x115, 0),
			    (0xf8 + (phy * 0x18) + 0x10),
			    regVal);
		if (speed == SRIO_SPEED_6_25G) {
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x16, 0x0010);
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x24, 0x001E);
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x26, 0x0000);
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x18, 0x0010);
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x44, 0x001E);
			ncr_write32(NCP_REGION_ID(0x115, (phy + 1)),
				    0x46, 0x0000);
		}
		break;
	}
}

void enable_reset(u32 phy)
{
	u32 regVal;

	if (phy == 0) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0,
			   &regVal);
		regVal |= (1 << 5);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0,
			    regVal);
	} else if (phy == 1) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0,
			   &regVal);
		regVal |= (1 << 14);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0,
			    regVal);
	} else if (phy == 2) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4,
			   &regVal);
		regVal |= (1 << 19);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4,
			    regVal);
	} else if (phy == 3) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4,
			   &regVal);
		regVal |= (1 << 29);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4,
			    regVal);
	}
}

void release_reset(u32 phy)
{
	u32 regVal;

	if (phy == 0) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0,
			   &regVal);
		regVal &= (~(1 << 5));
		ncr_write32(NCP_REGION_ID(0x115, 0), 0,
			    regVal);
	} else if (phy == 1) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0,
			   &regVal);
		regVal &= (~(1 << 14));
		ncr_write32(NCP_REGION_ID(0x115, 0), 0,
			    regVal);
	} else if (phy == 2) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4,
			   &regVal);
		regVal &= (~(1 << 19));
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4,
			    regVal);
	} else if (phy == 3) {
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4,
			   &regVal);
		regVal &= (~(1 << 29));
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4,
			    regVal);
	}
}

int check_pll_lock(enum PLLMode mode, u32 phy)
{
	u32 regVal;
	int count = 0;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8), &regVal);
	regVal &= (1 << (20 + mode));

	do {
		/* 100 ms delay */
		mdelay(100);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8),
			   &regVal);
		regVal &= (1 << (20 + mode));
	} while ((!regVal) && (count++ < 5));

	return regVal;
}

int check_ack(u32 phy, u32 lane, enum Dir dir)
{
	u32 regVal;
	int count = 0;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8), &regVal);
	if (dir == TX)
		regVal &= (1 << (0 + lane));
	else
		regVal &= (1 << (16 + lane));

	do {
		/* 100 ms delay */
		mdelay(100);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8),
			   &regVal);
		if (dir == TX)
			regVal &= (1 << (0 + lane));
		else
			regVal &= (1 << (16 + lane));
	} while ((!regVal) && (count++ < 5));

	return regVal;
}

void set_tx_clk_ready(void)
{
	u32 regVal;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x8, &regVal);
	regVal |= 0xff;
	ncr_write32(NCP_REGION_ID(0x115, 0), 0x8, regVal);
}

/* RX clock and data recovery */
int check_rx_valid(u32 phy, u32 lane)
{
	u32 regVal;
	int count = 0;

	ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8), &regVal);
	regVal &= (1 << (24 + (4 * lane)));

	do {
		/* 100 ms delay */
		mdelay(100);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x184 + (phy * 8), &regVal);
		regVal &= (1 << (24 + (4 * lane)));
	} while ((!regVal) && (count++ < 5));

	return regVal;
}

/*
  ------------------------------------------------------------------------------
  powerup_lane
*/

static int
powerup_lane(u32 phy, u32 lane, enum PowerState state, enum Dir dir)
{
	u32 regVal, powerVal;
	u32 offset;
	int ret;

	if (dir == TX)
		offset = 0x18 + (((phy * 2) + lane) * 0x1c) + 0x4;
	else
		offset = 0x18 + (((phy * 2) + lane) * 0x1c) + 0x14;

	ncr_read32(NCP_REGION_ID(0x115, 0), offset, &regVal);
	regVal &= 0xffffffce;
	/* New Transmitter setting request */
	powerVal = (state == P2) ? 3 : ((state == P1) ? 2 : 0);
	regVal |= (powerVal << 4) | (1 << 0);
	ncr_write32(NCP_REGION_ID(0x115, 0), offset, regVal);

	/* Check if ack is set */
	ret = check_ack(phy, lane, dir);
	ncr_read32(NCP_REGION_ID(0x115, 0), offset, &regVal);
	regVal &= ~(1 << 0);
	ncr_write32(NCP_REGION_ID(0x115, 0), offset, regVal);

	return ret;
}

/*
  ------------------------------------------------------------------------------
  enable_lane
*/

static void
enable_lane(u32 phy, u32 lane, enum Dir dir)
{
	u32 regVal;
	u32 offset;

	if (TX == dir)
		offset = 0x18 + (((phy * 2) + lane) * 0x1c);
	else
		offset = 0x18 + (((phy * 2) + lane) * 0x1c) + 0x14;

	ncr_read32(NCP_REGION_ID(0x115, 0), offset, &regVal);

	if (TX == dir)
		regVal |= (1 << 25);
	else
		regVal |= (1 << 21);

	ncr_write32(NCP_REGION_ID(0x115, 0), offset, regVal);

	return;
}

/*
  ==============================================================================
  ==============================================================================
  Public
  ==============================================================================
  ==============================================================================
*/

/*
  ------------------------------------------------------------------------------
  pei_setup
*/

int
pei_setup(unsigned int control)
{
	unsigned int pci_srio_sata_mode;
	unsigned int val;
	unsigned int rc_mode;
	unsigned int phyVal0;
	unsigned int phyVal1;
	unsigned int srio0_mode;
	unsigned int srio1_mode;
	unsigned int srio0_speed;
	unsigned int srio1_speed;
	unsigned int sata0_speed;
	unsigned int sata1_speed;
	unsigned int sata0_mode;
	unsigned int sata1_mode;
	unsigned int srio0_ctrl;
	unsigned int srio1_ctrl;
	unsigned int reg_val = 0;
	enum PLLMode pll;
	int phy, lane;

	pci_srio_sata_mode = (control & 0x03c00000) >> 22;
	sata0_mode = (control & 0x20) >> 5;
	sata1_mode = (control & 0x40) >> 6;

	sata0_speed = (control & 0xc0000) >> 18;
	sata1_speed = (control & 0x30000) >> 20;

	srio0_mode = (control & 0x8) >> 3;
	srio1_mode = (control & 0x10) >> 4;

	srio0_speed = (control & 0x7000) >> 12;
	srio1_speed = (control & 0x38000) >> 15;

	/* LTSSM Disable for PEI0 */
	val = readl(pcie_gpreg0 + 0x8038);
	val &= (~(0x1));
	writel(val, pcie_gpreg0 + 0x8038);
	msleep(100);

	for (phy = 0; phy < 4; phy++)
		enable_reset(phy);

	ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
	phyVal0 &= ~1;
	ncr_write32(NCP_REGION_ID(0x115, 0), 0, phyVal0);
	mdelay(100);		/* TODO: Why is this needed? */

	switch (pci_srio_sata_mode) {
	case 0:
		/* PEI0x8 */
		rc_mode = (control & 0x80)<<15;
		/* Enable PEI0, PEI0 RC mode */
		phyVal0 = (control & 0x1) | rc_mode;
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, phyVal0);
		/* PIPE port select -- Enable PIPE0 interface */
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, (0x1<<24));
		break;
	case 1:
		/*
		  PEI0x4, PEI1x4
		*/
		phyVal1 = 0x0e7001ac;
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, phyVal1);

		phyVal0 = 0x84400040;
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, phyVal0);
		break;
	case 2:
		/* PEI0x4_PEI1x2_SATA0x1_SATA1x1 */
		rc_mode = (control & 0x80)<<15;
		/* Enable PEI0/PEI1, PEI0 RC mode */
		phyVal0 = (control & 0x3) | rc_mode;
		/* PEI0x4 */
		phyVal0 |= (0x1 << 26);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, phyVal0);
		/* PEI1x2_SATA0_SATA1 */
		phyVal1 = (0x1 << 23);
		/* PIPE port select -- Enable PIPE0/PIPE1 interface */
		phyVal1 |= (0x2 << 24);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, phyVal1);
		if (sata0_mode)
			setup_sata_mode(SATA0, sata0_speed);
		if (sata1_mode)
			setup_sata_mode(SATA1, sata1_speed);
		if (!check_pll_lock(PLLA, 1)) {
			printk("PLLA didn't lock\n");
			return 1;
		}
		for (lane = 0; lane < 2; lane++) {
			if (!check_ack(3, lane, TX)) {
				printk("TX ACK not set for PHY3 LANE%d\n", lane);
				return 1;
			}
			if (!check_ack(3, lane, RX)) {
				printk("RX ACK not set for PHY3 LANE%d\n", lane);
				return 1;
			}
		}

		/* Set TX clock ready */
		set_tx_clk_ready();

		/* Enable SATA0/SATA1 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &phyVal0);
		phyVal0 |= (control & 0x20) << 24;
		phyVal0 |= (control & 0x40) << 24;
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, phyVal0);

		if (!check_rx_valid(3, 0)) {
			printk("RX clock/data recovery not asserted for PHY3 LANE0\n");
			return 1;
		}
		if (!check_rx_valid(3, 1)) {
			printk("RX clock/data recovery not asserted for PHY3 LANE1\n");
			return 1;
		}
		break;
	case 3:
		/* PEI0x2_PEI2x2_PEI1x2_SATA0x1_SATA1x1 */
		rc_mode = (control & 0x80)<<15;
		/* Enable PEI0/PEI1/PEI2, PEI0 RC mode */
		phyVal0 = (control & 0x7) | rc_mode;
		/* PEI0x2_PEI2x2 */
		phyVal0 |= (0x3 << 26);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, phyVal0);
		/* PEI1x2_SATA0_SATA1 */
		phyVal1 = (0x1 << 23);
		/* PIPE port select -- Enable PIPE0, PIPE1
		 * and PIPE2 interface */
		phyVal1 |= (0x3 << 24);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, phyVal1);
		if (sata0_mode)
			setup_sata_mode(SATA0, sata0_speed);
		if (sata1_mode)
			setup_sata_mode(SATA1, sata1_speed);

		if (!check_pll_lock(PLLA, 1)) {
			printk("PLLA didn't lock\n");
			return 1;
		}
		for (lane = 0; lane < 2; lane++) {
			if (!check_ack(3, lane, TX)) {
				printk("TX ACK not set for PHY3 LANE%d\n", lane);
				return 1;
			}
			if (!check_ack(3, lane, RX)) {
				printk("RX ACK not set for PHY3 LANE%d\n", lane);
				return 1;
			}
		}
		/* Set TX clock ready */
		set_tx_clk_ready();

		/* Enable SATA0/SATA1 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
		phyVal0 |= (control & 0x20) << 24;
		phyVal0 |= (control & 0x40) << 24;
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, phyVal0);

		if (!check_rx_valid(3, 0)) {
			printk("RX clock/data recovery not asserted for PHY3 LANE0\n");
			return 1;
		}
		if (!check_rx_valid(3, 1)) {
			printk("RX clock/data recovery not asserted for PHY3 LANE1\n");
			return 1;
		}
		break;
	case 4:
		/* PEI0x2_SRIO0x2_PEI1x4 */
		rc_mode = (control & 0x80)<<15;
		/* Enable PEI0/PEI1/SRIO0, PEI0 RC mode */
		phyVal0 = (control & 0x3) | rc_mode;
		/* PEI0x2_SRI0x2 */
		phyVal0 |= (0x7 << 26);
		/* SRIO0 mode */
		srio0_ctrl = ((control & 0x300) << 12);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val &= ~(0xf | (0x1 << 22) | (0x7<<26) | (0x3 << 20));
		reg_val |= phyVal0 | srio0_ctrl;
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
		/* PEI1x4 */
		phyVal1 = (0x1 << 22);
		/* PIPE port select -- Enable PIPE0/PIPE1 interface */
		phyVal1 |= (0x2 << 24);
		/* SRIO0 speed */
		srio0_speed = (control & 0x7000);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &reg_val);
		reg_val &= ~((0x3<<24) | (0x3 << 22) | (0x7 << 12));
		reg_val |= (phyVal1 | srio0_speed);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, reg_val);
		srio0_speed = srio0_speed >> 12;
		if (srio0_mode)
			setup_srio_mode(SRIO0, srio0_speed);
		printk("Set up sRIO0 -- %d\n", srio0_speed);
		for (phy = 0; phy < 4; phy++)
			release_reset(phy);

		pll = PLLA;
		if ((srio0_speed == SRIO_SPEED_3_125G)
		    || (srio0_speed == SRIO_SPEED_6_25G))
			pll = PLLB;
		if (!check_pll_lock(pll, 1)) {
			printk("%s didn't lock\n",
			       pll == PLLA ? "PLLA" : "PLLB");
			return 1;
		}
		printk("Enabling sRIO .");
		for (lane = 0; lane < 2; lane++) {
			if (!check_ack(3, lane, RX)) {
				printk("RX ACK not set for PHY1 LANE%d\n", lane);
				return 1;
			}
		}
		/* Power up TX/RX lanes */
		for (lane = 0; lane < 2; lane++) {
			if (!powerup_lane(1, lane, P1, TX)) {
				printk("TX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
			if (!powerup_lane(1, lane, P1, RX)) {
				printk("RX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
		}

		/* Set TX clock ready */
		set_tx_clk_ready();

		/* Power up TX/RX lanes */
		for (lane = 0; lane < 2; lane++) {
			if (!powerup_lane(1, lane, P0, TX)) {
				printk("TX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
			if (!powerup_lane(1, lane, P0, RX)) {
				printk("RX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
		}

		for (lane = 0; lane < 2; lane++) {
			enable_lane(1, lane, TX);
			enable_lane(1, lane, RX);
		}
		/* Enable SRIO0 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0,
			    phyVal0 | 0x8);
		printk("Done\n");
		break;
	case 5:
		/* PEI0x2_SRIO0x2_PEI1x2_SATA0x1_SATA1x1 */
		rc_mode = (control & 0x80)<<15;
		/* Enable PEI0/PEI1/SRIO0, PEI0 RC mode */
		phyVal0 = (control & 0x3) | rc_mode;
		/* PEI0x2_SRI0x2 */
		phyVal0 |= (0x7 << 26);
		/* SRIO0 mode */
		srio0_ctrl = ((control & 0x300) << 12);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val &= ~(0xf | (0x1 << 22) | (0x7<<26) | (0x3 << 20));
		reg_val |= phyVal0 | srio0_ctrl;
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
		/* PEI1x2_SATA0x1_SATA1x1 */
		phyVal1 = (0x1 << 23);
		/* PIPE port select -- Enable PIPE0/PIPE1 interface */
		phyVal1 |= (0x2 << 24);
		/* SRIO0 speed */
		srio0_speed = (control & 0x7000);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &reg_val);
		reg_val &= ~((0x3<<24) | (0x3 << 22) | (0x7 << 12));
		reg_val |= (phyVal1 | srio0_speed);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, reg_val);
		srio0_speed = srio0_speed >> 12;
		if (sata0_mode)
			setup_sata_mode(SATA0, sata0_speed);
		if (sata1_mode)
			setup_sata_mode(SATA1, sata1_speed);
		if (srio0_mode)
			setup_srio_mode(SRIO0, srio0_speed);
		printk("Set up sRIO0 -- %d\n", srio0_speed);
		for (phy = 0; phy < 4; phy++)
			release_reset(phy);
		pll = PLLA;
		if ((srio0_speed == SRIO_SPEED_3_125G)
		    || (srio0_speed == SRIO_SPEED_6_25G))
			pll = PLLB;
		if (!check_pll_lock(pll, 1)) {
			printk("%s didn't lock\n",
			       pll == PLLA ? "PLLA" : "PLLB");
			return 1;
		}
		for (lane = 0; lane < 2; lane++) {
			if (!check_ack(3, lane, TX)) {
				printk("TX ACK not set for PHY3 LANE%d\n", lane);
				return 1;
			}
			if (!check_ack(3, lane, RX)) {
				printk("RX ACK not set for PHY3 LANE%d\n", lane);
				return 1;
			}
		}
		printk("Enabling sRIO/SATA .");
		/* Power up TX/RX lanes */
		for (lane = 0; lane < 2; lane++) {
			if (!powerup_lane(1, lane, P1, TX)) {
				printk("TX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
			if (!powerup_lane(1, lane, P1, RX)) {
				printk("RX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
		}

		/* Set TX clock ready */
		set_tx_clk_ready();

		/* Power up TX/RX lanes */
		for (lane = 0; lane < 2; lane++) {
			if (!powerup_lane(1, lane, P0, TX)) {
				printk("TX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
			if (!powerup_lane(1, lane, P0, RX)) {
				printk("RX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
		}
		printk(".");
		for (lane = 0; lane < 2; lane++) {
			enable_lane(1, lane, TX);
			enable_lane(1, lane, RX);
		}

		/* Enable SATA0/SATA1 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
		phyVal0 |= (control & 0x20) << 24;
		phyVal0 |= (control & 0x40) << 24;
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, phyVal0);

		if (!check_rx_valid(3, 0)) {
			printk("RX clock/data recovery not asserted for PHY3 LANE0\n");
			return 1;
		}
		if (!check_rx_valid(3, 1)) {
			printk("RX clock/data recovery not asserted for PHY3 LANE1\n");
			return 1;
		}
		/* Enable SRIO0 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0,
			    phyVal0 | 0x8);
		printk("Done\n");
		break;
	case 6:
		/* PEI0x2_SRIO0x2_PEI1x2_PEI2x2 */
		rc_mode = (control & 0x80)<<15;
		/* Enable PEI0/PEI1/PEI2/SRIO0, PEI0 RC mode */
		phyVal0 = (control & 0x7) | rc_mode;
		/* PEI0x2_SRIO0x2 */
		phyVal0 |= (0x7 << 26);
		/* SRIO0 mode */
		srio0_ctrl = ((control & 0x300) << 12);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val &= ~(0xf | (0x1 << 22) | (0x7<<26) | (0x3 << 20));
		reg_val |= phyVal0 | srio0_ctrl;
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
		/* PEI1x2_PEI2x2 */
		phyVal1 = (0x3 << 22);
		/* PIPE port select -- Enable PIPE0, PIPE1
		 * and PIPE2 interface */
		phyVal1 |= (0x3 << 24);
		/* SRIO0 speed */
		srio0_speed = (control & 0x7000);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &reg_val);
		reg_val &= ~((0x3<<24) | (0x3 << 22) | (0x7 << 12));
		reg_val |= (phyVal1 | srio0_speed);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, reg_val);
		srio0_speed = srio0_speed >> 12;
		if (srio0_mode)
			setup_srio_mode(SRIO0, srio0_speed);
		printk("Set up sRIO0 -- %d\n", srio0_speed);
		for (phy = 0; phy < 4; phy++)
			release_reset(phy);

		pll = PLLA;
		if ((srio0_speed == SRIO_SPEED_3_125G)
		    || (srio0_speed == SRIO_SPEED_6_25G))
			pll = PLLB;
		if (!check_pll_lock(pll, 1)) {
			printk("%s didn't lock\n",
			       pll == PLLA ? "PLLA" : "PLLB");
			return 1;
		}

		printk("Enabling sRIO .");
		/* Power up TX/RX lanes */
		for (lane = 0; lane < 2; lane++) {
			if (!powerup_lane(1, lane, P1, TX)) {
				printk("TX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
			if (!powerup_lane(1, lane, P1, RX)) {
				printk("RX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
		}

		/* Set TX clock ready */
		set_tx_clk_ready();

		/* Power up TX/RX lanes */
		for (lane = 0; lane < 2; lane++) {
			if (!powerup_lane(1, lane, P0, TX)) {
				printk("TX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
			if (!powerup_lane(1, lane, P0, RX)) {
				printk("RX powerup failed for PHY1 LANE%d\n", lane);
				return 1;
			}
		}

		for (lane = 0; lane < 2; lane++) {
			enable_lane(1, lane, TX);
			enable_lane(1, lane, RX);
		}
		/* Enable SRIO0 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0,
			    phyVal0 | 0x8);
		printk("Done\n");
		break;
	case 7:
		/* SRIO1x2_SRIO0x2_PEI1x4 */
		/* Enable PEI1/SRIO0/SRIO1 */
		phyVal0 = (control & 0x2);
		/* SRIO1x2_SRIO0x2 */
		phyVal0 |= (0x4 << 26);
		/* SRIO0 mode */
		srio0_ctrl = ((control & 0x300) << 12);
		/* SRIO1 mode */
		srio1_ctrl = ((control & 0xC00) << 14);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val &= ~(0xf | (0x7<<26) | (0x3 << 20) | (0x3 << 24));
		reg_val |= (phyVal0 | srio0_ctrl | srio1_ctrl);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
		/* PEI1x4 */
		phyVal1 = (0x1 << 22);
		/* PIPE port select -- Enable PIPE0/PIPE1 interface */
		phyVal1 |= (0x2 << 24);
		/* SRIO0 speed */
		srio0_speed = (control & 0x7000);
		/* SRIO1 speed */
		srio1_speed = (control & 0x38000) << 1;
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &reg_val);
		reg_val &= ~((0x3 << 24) | (0x3 << 22) | (0x7 << 12) |
			     (0x7 << 16));
		reg_val |= (phyVal1 | srio0_speed | srio1_speed);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, reg_val);
		srio0_speed = srio0_speed >> 12;
		srio1_speed = srio1_speed >> 16;
		if (srio0_mode)
			setup_srio_mode(SRIO0, srio0_speed);
		printk("Set up sRIO0 -- %d\n", srio0_speed);
		if (srio1_mode)
			setup_srio_mode(SRIO1, srio1_speed);
		printk("Set up sRIO1 -- %d\n", srio1_speed);
		for (phy = 0; phy < 4; phy++)
			release_reset(phy);

		for (phy = 0; phy < 2; phy++) {
			pll = PLLA;
			if ((phy == 0) &&
			    ((srio1_speed == SRIO_SPEED_3_125G) ||
			     (srio1_speed == SRIO_SPEED_6_25G)))
				pll = PLLB;
			else if ((phy == 1) &&
				 ((srio0_speed == SRIO_SPEED_3_125G) ||
				  (srio0_speed == SRIO_SPEED_6_25G)))
				pll = PLLB;
			if (!check_pll_lock(pll, phy)) {
				printk("%s didn't lock\n",
				       pll == PLLA ? "PLLA" : "PLLB");
				return 1;
			}
		}
		printk("Enabling sRIO .");
		/* Power up TX/RX lanes */
		for (phy = 0; phy < 2; phy++) {
			for (lane = 0; lane < 2; lane++) {
				if (!powerup_lane(phy, lane, P1, TX)) {
					printk("TX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
				if (!powerup_lane(phy, lane, P1, RX)) {
					printk("RX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
			}
		}
		printk(".");
		/* Set TX clock ready */
		set_tx_clk_ready();
		/* Power up TX/RX lanes */
		for (phy = 0; phy < 2; phy++) {
			for (lane = 0; lane < 2; lane++) {
				if (!powerup_lane(phy, lane, P0, TX)) {
					printk("TX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
				if (!powerup_lane(phy, lane, P0, RX)) {
					printk("RX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
			}
		}
		printk(".");
		for (phy = 0; phy < 2; phy++) {
			for (lane = 0; lane < 2; lane++) {
				enable_lane(phy, lane, TX);
				enable_lane(phy, lane, RX);
			}
		}
		printk(".");
		/* Enable SRIO0/SRIO1 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0,
			    phyVal0 | 0x408);
		printk("Done\n");
		break;
	case 8:
		/* SRIO1x2_SRIO0x2_PEI1x2_SATA0x1_SATA1x1 */
		/* Enable PEI1 */
		phyVal0 = (control & 0x2);
		/* SRIO1x2_SRIO0x2 */
		phyVal0 |= (0x4 << 26);
		/* SRIO0 mode */
		srio0_ctrl = ((control & 0x300) << 12);
		/* SRIO1 mode */
		srio1_ctrl = ((control & 0xC00) << 14);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val &= ~(0xf | (0x7 << 26) | (0x3 << 20) | (0x3 << 24));
		reg_val |= (phyVal0 | srio0_ctrl | srio1_ctrl);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
		/* PEI1x2_SATA0x1_SATA1x1 */
		phyVal1 = (0x1 << 23);
		/* PIPE port select -- Enable PIPE0/PIPE1 interface */
		phyVal1 |= (0x2 << 24);
		/* SRIO0 speed */
		srio0_speed = (control & 0x7000);
		/* SRIO1 speed */
		srio1_speed = (control & 0x38000) << 1;
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &reg_val);
		reg_val &= ~((0x3 << 24) | (0x3 << 22) | (0x7 << 12) |
			     (0x7 << 16));
		reg_val |= (phyVal1 | srio0_speed | srio1_speed);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, reg_val);
		srio0_speed = srio0_speed >> 12;
		srio1_speed = srio1_speed >> 16;
		if (sata0_mode)
			setup_sata_mode(SATA0, sata0_speed);
		if (sata1_mode)
			setup_sata_mode(SATA1, sata1_speed);
		if (srio0_mode)
			setup_srio_mode(SRIO0, srio0_speed);
		printk("Set up sRIO0 -- %d\n", srio0_speed);
		if (srio1_mode)
			setup_srio_mode(SRIO1, srio1_speed);
		printk("Set up sRIO1 -- %d\n", srio1_speed);
		for (phy = 0; phy < 4; phy++)
			release_reset(phy);

		for (phy = 0; phy < 2; phy++) {
			pll = PLLA;
			if ((phy == 0) && ((srio1_speed == SRIO_SPEED_3_125G)
					   || (srio1_speed == SRIO_SPEED_6_25G)))
				pll = PLLB;
			else if ((phy == 1) &&
				 ((srio0_speed == SRIO_SPEED_3_125G)
				  || (srio0_speed == SRIO_SPEED_6_25G)))
				pll = PLLB;
			if (!check_pll_lock(pll, phy)) {
				printk("%s didn't lock\n",
				       pll == PLLA ? "PLLA" : "PLLB");
				return 1;
			}
		}
		printk("Enabling sRIO/SATA .");
		/* Power up TX/RX lanes */
		for (phy = 0; phy < 2; phy++) {
			for (lane = 0; lane < 2; lane++) {
				if (!powerup_lane(phy, lane, P1, TX)) {
					printk("TX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
				if (!powerup_lane(phy, lane, P1, RX)) {
					printk("RX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
			}
		}

		printk(".");
		/* Set TX clock ready */
		set_tx_clk_ready();

		/* Power up TX/RX lanes */
		for (phy = 0; phy < 2; phy++) {
			for (lane = 0; lane < 2; lane++) {
				if (!powerup_lane(phy, lane, P0, TX)) {
					printk("TX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
				if (!powerup_lane(phy, lane, P0, RX)) {
					printk("RX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
			}
		}
		printk(".");

		for (phy = 0; phy < 2; phy++) {
			for (lane = 0; lane < 2; lane++) {
				enable_lane(phy, lane, TX);
				enable_lane(phy, lane, RX);
			}
		}
		printk(".");

		/* Enable SATA0/SATA1 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
		phyVal0 |= (control & 0x20) << 24;
		phyVal0 |= (control & 0x40) << 24;
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, phyVal0);

		if (!check_rx_valid(3, 0)) {
			printk("RX clock/data recovery not asserted for PHY3 LANE0\n");
			return 1;
		}
		if (!check_rx_valid(3, 1)) {
			printk("RX clock/data recovery not asserted for PHY3 LANE1\n");
			return 1;
		}
		/* Enable SRIO0/SRIO1 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0,
			    phyVal0 | 0x408);
		printk("Done\n");
		break;
	case 9:
		/* SRIO1x2_SRIO0x2_PEI1x2_PEI2x2 */
		/* Enable PEI2/PEI1 */
		phyVal0 = (control & 0x6);
		/* SRIO1x2_SRIO0x2 */
		phyVal0 |= (0x4 << 26);
		/* SRIO0 mode */
		srio0_ctrl = ((control & 0x300) << 12);
		/* SRIO1 mode */
		srio1_ctrl = ((control & 0xC00) << 14);
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &reg_val);
		reg_val &= ~(0xf | (0x7 << 26) | (0x3 << 20) | (0x3 << 24));
		reg_val |= (phyVal0 | srio0_ctrl | srio1_ctrl);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0, reg_val);
		/* PEI1x2_PEI2x2 */
		phyVal1 = (0x3 << 22);
		/* PIPE port select -- Enable PIPE0, PIPE1
		 * and PIPE2 interface */
		phyVal1 |= (0x3 << 24);
		/* SRIO0 speed */
		srio0_speed = (control & 0x7000);
		/* SRIO1 speed */
		srio1_speed = (control & 0x38000) << 1;
		ncr_read32(NCP_REGION_ID(0x115, 0), 0x4, &reg_val);
		reg_val &= ~((0x3 << 24) | (0x3 << 22) | (0x7 << 12) |
			     (0x7 << 16));
		reg_val |= (phyVal1 | srio0_speed | srio1_speed);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0x4, reg_val);
		srio0_speed = srio0_speed >> 12;
		srio1_speed = srio1_speed >> 16;
		if (srio0_mode)
			setup_srio_mode(SRIO0, srio0_speed);
		printk("Set up sRIO0 -- %d\n", srio0_speed);
		if (srio1_mode)
			setup_srio_mode(SRIO1, srio1_speed);
		printk("Set up sRIO1 -- %d\n", srio1_speed);
		for (phy = 0; phy < 4; phy++)
			release_reset(phy);

		for (phy = 0; phy < 2; phy++) {
			pll = PLLA;
			if ((phy == 0) &&
			    ((srio1_speed == SRIO_SPEED_3_125G) ||
			     (srio1_speed == SRIO_SPEED_6_25G)))
				pll = PLLB;
			else if ((phy == 1) &&
				 ((srio0_speed == SRIO_SPEED_3_125G) ||
				  (srio0_speed == SRIO_SPEED_6_25G)))
				pll = PLLB;
			if (!check_pll_lock(pll, phy)) {
				printk("%s didn't lock\n",
				       pll == PLLA ? "PLLA" : "PLLB");
				return 1;
			}
		}
		printk("Enabling sRIO .");
		/* Power up TX/RX lanes */
		for (phy = 0; phy < 2; phy++) {
			for (lane = 0; lane < 2; lane++) {
				if (!powerup_lane(phy, lane, P1, TX)) {
					printk("TX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
				if (!powerup_lane(phy, lane, P1, RX)) {
					printk("RX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
			}
		}

		printk(".");
		/* Set TX clock ready */
		set_tx_clk_ready();

		/* Power up TX/RX lanes */
		for (phy = 0; phy < 2; phy++) {
			for (lane = 0; lane < 2; lane++) {
				if (!powerup_lane(phy, lane, P0, TX)) {
					printk("TX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
				if (!powerup_lane(phy, lane, P0, RX)) {
					printk("RX powerup failed for PHY%d LANE%d\n", phy, lane);
					return 1;
				}
			}
		}
		printk(".");

		for (phy = 0; phy < 2; phy++) {
			for (lane = 0; lane < 2; lane++) {
				enable_lane(phy, lane, TX);
				enable_lane(phy, lane, RX);
			}
		}
		printk(".");
		/* Enable SRIO0/SRIO1 */
		ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
		ncr_write32(NCP_REGION_ID(0x115, 0), 0,
			    phyVal0 | 0x408);
		printk("Done\n");
		break;
	}

	ncr_read32(NCP_REGION_ID(0x115, 0), 0, &phyVal0);
	phyVal0 |= 1;
	ncr_write32(NCP_REGION_ID(0x115, 0), 0, phyVal0);
	msleep(100);

	switch (pci_srio_sata_mode) {
	case 1:
		writel(0x70120, pcie_rc + 0x710);
		writel(0x104be, pcie_rc + 0x80c);
		writel(0x1017201, pcie_rc + 0x8a8);
		break;
	default:
		break;
	}

	return 0;
}

EXPORT_SYMBOL(pei_setup);

/*
  ------------------------------------------------------------------------------
  pei_init
*/

static int
pei_init(void)
{
	/* Use the device tree to determine the Axxia type. */
	if (of_find_compatible_node(NULL, NULL, "lsi,axm5500") ||
	    of_find_compatible_node(NULL, NULL, "lsi,axm5516")) {
		is_5500 = 1;
	} else if (of_find_compatible_node(NULL, NULL, "lsi,axm5616")) {
		is_5600 = 1;
		pcie_gpreg0 = ioremap(0xa003000000, 0x10000);
		pcie_rc = ioremap(0xa002000000, 0x1000);
	} else if (of_find_compatible_node(NULL, NULL, "lsi,axc6732")) {
		is_6700 = 1;
	} else {
		pr_err("No Valid Compatible String Found for PEI!\n");

		return -1;
	}

	return 0;
}

arch_initcall(pei_init);

MODULE_AUTHOR("John Jacques <john.jacques@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PEI Setup for Axxia");
