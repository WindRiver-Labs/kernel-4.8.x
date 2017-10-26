/*
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

/*
 * TODO: IPU5_SDK Verify that this file is correct when
 * IPU5 SDK has been released.
 */

#ifndef _IPU_DEVICE_ACB_DEVICES_H_
#define _IPU_DEVICE_ACB_DEVICES_H_

enum ipu_device_acb_id {
	/* PSA accelerators */
	IPU_DEVICE_ACB_WBA_ID      = 0,
	IPU_DEVICE_ACB_RYNR_ID,
	IPU_DEVICE_ACB_DEMOSAIC_ID,
	IPU_DEVICE_ACB_GLTM_ID,
	IPU_DEVICE_ACB_XNR_ID,
	IPU_DEVICE_ACB_ACM_ID, /* ACM is called VCA in HW */
	IPU_DEVICE_ACB_GAMMASTAR_ID,
	IPU_DEVICE_ACB_GTC_ID,
	IPU_DEVICE_ACB_YUV1_ID,
	IPU_DEVICE_ACB_DVS_ID,
	/* ISA accelerators */
	IPU_DEVICE_ACB_ICA_ID,
	IPU_DEVICE_ACB_LSC_ID,
	IPU_DEVICE_ACB_DPC_ID,
	IPU_DEVICE_ACB_IDS_ID,
	IPU_DEVICE_ACB_AWB_ID,
	IPU_DEVICE_ACB_AF_ID,
	IPU_DEVICE_ACB_AE_ID,
	IPU_DEVICE_ACB_NUM_ACB
};

#define IPU_DEVICE_ACB_NUM_PSA_ACB (IPU_DEVICE_ACB_DVS_ID + 1)
#define IPU_DEVICE_ACB_NUM_ISA_ACB \
	(IPU_DEVICE_ACB_NUM_ACB - IPU_DEVICE_ACB_NUM_PSA_ACB)

#endif /*  _IPU_DEVICE_ACB_DEVICES_H_ */
