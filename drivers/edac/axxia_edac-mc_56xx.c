/*
 * drivers/edac/axxia_edac-mc.c
 *
 * EDAC Driver for Intel's Axxia 5600 System Memory Controller
 *
 * Copyright (C) 2016 Intel Inc.
 *
 * This file may be distributed under the terms of the
 * GNU General Public License.
 */

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/lsi-ncr.h>
#include <linux/edac.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include "edac_core.h"
#include "edac_module.h"

#define FMT "%s: syscon lookup failed hence using hardcoded register address\n"

#define MPR_FMT9 "\n%3d %#010x %#010x %#010x %#010x"\
		" %#010x %#010x %#010x %#010x %#010x"

#define MPR_FMT16 " %#010x %#010x %#010x %#010x"\
		" %#010x %#010x %#010x %#010x %#010x"

#define MPR_HDR9	"Lp.    dram0      dram1      dram2      dram3"\
		"      dram4      dram5      dram6      dram7      dram8"

#define MPR_HDR18	"      dram9     dram10     dram11     dram12"\
		"     dram13     dram14     dram15     dram16     dram17"

#define INTEL_EDAC_MOD_STR	"axxia56xx_edac"

#define AXI2_SER3_PHY_ADDR	0x008002c00000ULL
#define AXI2_SER3_PHY_SIZE	PAGE_SIZE

#define SM_MPR_PAGE		0x1

#define SM_56XX_DENALI_CTL_00	0x0
#define SM_56XX_DENALI_CTL_57	0xe4
#define SM_56XX_DENALI_CTL_117	0x1d4
#define SM_56XX_DENALI_CTL_123	0x1ec

/* INT STATUS */
#define SM_56XX_DENALI_CTL_366	0x5b8
#define SM_56XX_DENALI_CTL_367	0x5bc

/* INT ACK */
#define SM_56XX_DENALI_CTL_368 0x5c0
#define SM_56XX_DENALI_CTL_369 0x5c4

/* INT MASK */
#define SM_56XX_DENALI_CTL_370 0x5c8
#define SM_56XX_DENALI_CTL_371 0x5cc

/* MPR PAGE */
#define SM_56XX_DENALI_CTL_58 0xe8
#define SM_56XX_DENALI_CTL_59 0xec
#define SM_56XX_DENALI_CTL_60 0xf0
#define SM_56XX_DENALI_CTL_61 0xf4
#define SM_56XX_DENALI_CTL_62 0xf8

#define SYSCON_PERSIST_SCRATCH	 0xdc
#define SMEM_PERSIST_SCRATCH_BIT (0x1 << 3)

#define IRQ_NAME_LEN 16
#define MEMORY_CONTROLLERS 4
#define MAX_DQ 18
#define MAX_CS 4
#define MPR_CIRCULAR_BUF_LEN 16
#define MPR_PAGE_BYTES 4
#define MPR_ERRORS 2 /* CRC, CA Parity error */

#define SM_INT_MASK_LOW (0xfbbfef01)
#define SM_INT_MASK_ALL_LOW (0xffffffff)
#define SM_INT_MASK_HIGH (0x1)
#define SM_INT_MASK_ALL_HIGH (0x7)
#define ALIVE_NOTIFICATION_PERIOD (90*1000)

static int log = 1;
module_param(log, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(log, "Log each error to kernel log.");

static int force_restart = 1;
module_param(force_restart, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(force_restart, "Machine restart on fatal error.");

static atomic64_t mc_counter = ATOMIC_INIT(0);
/*
 Bit [34] = Logical OR of all lower bits.
 Bit [33] = A CRC error occurred on the write data bus.
 Bit [32] = The software-initiated control word write has completed.
 Bit [31] = The user-initiated DLL resync has completed.
 Bit [30] = A state change has been detected on the
	dfi_init_complete signal after initialization.
 Bit [29] = The assertion of the INHIBIT_DRAM_CMD parameter has
	successfully inhibited the command queue.
 Bit [28] = The register interface-initiated mode register write has
	completed and another mode register write may be issued.
 Bit [27] = A Low Power Interface (LPI) timeout error has occurred.
 Bit [26] = MPR read command, initiated with a software MPR_READ request,
	 is complete.
 Bit [25] = Error received from the PHY on the DFI bus.
 Bit [24] = RESERVED
 Bit [23] = RESERVED
 Bit [22] = A parity error has been detected on the address/control bus
	on a registered DIMM.
 Bit [21] = The leveling operation has completed.
 Bit [20] = A read leveling gate training operation has been requested.
 Bit [19] = A read leveling operation has been requested.
 Bit [18] = A write leveling operation has been requested.
 Bit [17] = A DFI update error has occurred. Error information can be
	found in the UPDATE_ERROR_STATUS parameter.
 Bit [16] = A write leveling error has occurred. Error information can
	be found in the WRLVL_ERROR_STATUS parameter.
 Bit [15] = A read leveling gate training error has occurred. Error
	information can be found in the RDLVL_ERROR_STATUS parameter.
 Bit [14] = A read leveling error has occurred. Error information can be
	found in the RDLVL_ERROR_STATUS parameter.
 Bit [13] = The user has programmed an invalid setting associated with
	user words per burst.
	Examples: Setting param_reduc when burst length = 2. A 1:2
	MC:PHY clock ratio with burst length = 2.
 Bit [12] = A wrap cycle crossing a DRAM page has been detected. This
	is unsupported & may result in memory data corruption.
 Bit [11] = A write was attempted to a writeprotected region.
 Bit [10] = The BIST operation has been completed.
 Bit [9] = The low power operation has been completed.
 Bit [8] = The MC initialization has been completed.
 Bit [7] = An error occurred on the port command channel.
 Bit [6] = Multiple uncorrectable ECC events have been detected.
 Bit [5] = An uncorrectable ECC event has been detected.
 Bit [4] = Multiple correctable ECC events have been detected.
 Bit [3] = A correctable ECC event has been detected.
 Bit [2] = Multiple accesses outside the defined PHYSICAL memory space
	have occurred.
 Bit [1] = A memory access outside the defined PHYSICAL memory space
	has occurred.
 Bit [0] = The memory reset is valid on the DFI bus.

 Of these 1, 2, 3, 4, 5, 6, 7, 12, 22 and 26 are of interest.
*/

/*
 *   MPR dump processing - overview.
 *
 * As ALERT_N does not have information about failing cs
 * one need to collect dumps for all available cs. Below given example
 * for two cs0/cs1.
 *
 *   SMEM MC           smmon_isr           smmon_wq
 *     |                   |                   |
 *     |                   |                   |
 *     |ALERT_N - int_status bit [33]          |
 *     |------------------>|                   |
 *     |                   |schedule smmon_wq  |
 *     |                   |------------------>|
 *     |                   |                   |if(dump_in_progress==0)
 *     |                   |                   |  dump_in_progress=1
 *     |                   |                   |
 *     |CTL_57 cs0 page1 (trigger dump)        |
 *     |<--------------------------------------|
 *     |                   |                   |wait
 *     |int_status bit [26]|                   |
 *     |------------------>|                   |
 *     |                   |wake up smmon_wq   |
 *     |                   |------------------>|
 *     |read MPR CTL_58-78 |                   |
 *     |<--------------------------------------|collect cs0 MPR page1
 *     |                   |                   |
 *     |                   |                   |
 *     |CTL_57 cs1 page1 (trigger dump)        |
 *     |<--------------------------------------|
 *     |                   |                   |wait
 *     |int_status bit [26]|                   |
 *     |------------------>|                   |
 *     |                   |wake up smmon_wq   |
 *     |                   |------------------>|
 *     |read MPR CTL_58-78 |                   |
 *     |<--------------------------------------|collect cs1 MPR page1
 *     |                   |                   |
 *     |                   |                   |process dumps
 *     |                   |                   |dump_in_progress=0
 *     |                   |                   |
 */

/* INT_STATUS */
struct __packed sm_56xx_denali_ctl_366
{
	unsigned int int_status;
};

struct __packed sm_56xx_denali_ctl_367
{
#ifdef CPU_BIG_ENDIAN
	unsigned	 obsolete1				: 24;
	unsigned	 reserved0				: 5;
	unsigned	 int_status				: 3;
#else	/* Little Endian */
	unsigned	 int_status				: 3;
	unsigned	 reserved0				: 5;
	unsigned	 obsolete1				: 24;
#endif
};

/* ACK */
struct __packed sm_56xx_denali_ctl_368
{
	unsigned int	 int_ack;
};

struct __packed sm_56xx_denali_ctl_369
{
#ifdef CPU_BIG_ENDIAN
	unsigned	 obsolete1				: 24;
	unsigned	 reserved0				: 6;
	unsigned	 int_ack				: 2;
#else	/* Little Endian */
	unsigned	 int_ack				: 2;
	unsigned	 reserved0				: 6;
	unsigned	 obsolete1				: 24;
#endif
};
/* MASK */

struct __packed sm_56xx_denali_ctl_370
{
	unsigned int int_mask;
};

struct __packed sm_56xx_denali_ctl_371
{
#ifdef CPU_BIG_ENDIAN
	unsigned	 reserved0				: 4;
	unsigned	 odt_rd_map_cs3				: 4;
	unsigned	 reserved1				: 4;
	unsigned	 odt_wr_map_cs2				: 4;
	unsigned	 reserved2				: 4;
	unsigned	 odt_rd_map_cs2				: 4;
	unsigned	 reserved3				: 5;
	unsigned	 int_mask				: 3;
#else	/* Little Endian */
	unsigned	 int_mask				: 3;
	unsigned	 reserved3				: 5;
	unsigned	 odt_rd_map_cs2				: 4;
	unsigned	 reserved2				: 4;
	unsigned	 odt_wr_map_cs2				: 4;
	unsigned	 reserved1				: 4;
	unsigned	 odt_rd_map_cs3				: 4;
	unsigned	 reserved0				: 4;
#endif
};

/* DRAM CLASS */
struct __packed sm_56xx_denali_ctl_00
{
#ifdef CPU_BIG_ENDIAN
	unsigned	 version				: 16;
	unsigned	 reserved0				: 4;
	unsigned	 dram_class				: 4;
	unsigned	 reserved1				: 7;
	unsigned	 start					: 1;
#else	/* Little Endian */
	unsigned	 start					: 1;
	unsigned	 reserved1				: 7;
	unsigned	 dram_class				: 4;
	unsigned	 reserved0				: 4;
	unsigned	 version				: 16;
#endif
};

/* Trigger MPR */
struct __packed sm_56xx_denali_ctl_57
{
#ifdef CPU_BIG_ENDIAN
	unsigned	 obsolete3				: 8;
	unsigned	 reserved2				: 3;
	unsigned	 read_mpr				: 5;
	unsigned	 reserved3				: 7;
	unsigned	 reserved1				: 1;
	unsigned	 mrw_status				: 8;
#else	/* Little Endian */
	unsigned	 mrw_status				: 8;
	unsigned	 reserved1				: 1;
	unsigned	 reserved3				: 7;
	unsigned	 read_mpr				: 5;
	unsigned	 reserved2				: 3;
	unsigned	 obsolete3				: 8;
#endif
};

struct __packed sm_56xx_denali_ctl_117
{
#ifdef CPU_BIG_ENDIAN
	unsigned	reserved0				: 5;
	unsigned	row_diff				: 3;
	unsigned	reserved1				: 6;
	unsigned	bank_diff				: 2;
	unsigned	reserved2				: 7;
	unsigned	zqcs_rotate				: 1;
	unsigned	reserved3				: 7;
	unsigned	zq_in_progress				: 1;
#else	/* Little Endian */
	unsigned	zq_in_progress				: 1;
	unsigned	reserved3				: 7;
	unsigned	zqcs_rotate				: 1;
	unsigned	reserved2				: 7;
	unsigned	bank_diff				: 2;
	unsigned	reserved1				: 6;
	unsigned	row_diff				: 3;
	unsigned	reserved0				: 5;
#endif
};

struct __packed sm_56xx_denali_ctl_123
{
#ifdef CPU_BIG_ENDIAN
	unsigned	 reserved0				: 5;
	unsigned	 memdata_ratio_0			: 3;
	unsigned	 reserved1				: 7;
	unsigned	 reduc					: 1;
	unsigned	 reserved2				: 4;
	unsigned	 burst_on_fly_bit			: 4;
	unsigned	 reserved3				: 4;
	unsigned	 cs_map					: 4;
#else	/* Little Endian */
	unsigned	 cs_map					: 4;
	unsigned	 reserved3				: 4;
	unsigned	 burst_on_fly_bit			: 4;
	unsigned	 reserved2				: 4;
	unsigned	 reduc					: 1;
	unsigned	 reserved1				: 7;
	unsigned	 memdata_ratio_0			: 3;
	unsigned	 reserved0				: 5;
#endif
};

struct __packed mpr_dump {
/*! @brief This specifies 8 bits of page information from 4 registers per
 * DRAM as per JEDEC
 */
	u8 dram_0_page[MPR_PAGE_BYTES]; /* X9-SMEM/X9-CMEM, XLF-SMEM/XLF-CMEM */
	u8 dram_1_page[MPR_PAGE_BYTES]; /* X9-SMEM/X9-CMEM, XLF-SMEM/XLF-CMEM */
	u8 dram_2_page[MPR_PAGE_BYTES]; /* X9-SMEM, XLF-SMEM */
	u8 dram_3_page[MPR_PAGE_BYTES]; /* X9-SMEM, XLF-SMEM */
	u8 dram_4_page[MPR_PAGE_BYTES]; /* X9-SMEM, XLF-SMEM */
	u8 dram_5_page[MPR_PAGE_BYTES]; /* X9-SMEM */
	u8 dram_6_page[MPR_PAGE_BYTES]; /* X9-SMEM */
	u8 dram_7_page[MPR_PAGE_BYTES]; /* X9-SMEM */
	u8 dram_8_page[MPR_PAGE_BYTES]; /* X9-SMEM */
	u8 dram_9_page[MPR_PAGE_BYTES];
	u8 dram_10_page[MPR_PAGE_BYTES];
	u8 dram_11_page[MPR_PAGE_BYTES];
	u8 dram_12_page[MPR_PAGE_BYTES];
	u8 dram_13_page[MPR_PAGE_BYTES];
	u8 dram_14_page[MPR_PAGE_BYTES];
	u8 dram_15_page[MPR_PAGE_BYTES];
	u8 dram_16_page[MPR_PAGE_BYTES];
	u8 dram_17_page[MPR_PAGE_BYTES];

/*! @brief specifies which MPR Page(n) to read per JEDEC. 2-bit width */
	u8	mpr_page_id;
	u8	cs;
};

enum events {
	EV_ILLEGAL = 0,
	EV_MULT_ILLEGAL,
	EV_CORR_ECC,
	EV_MULT_CORR_ECC,
	EV_UNCORR_ECC,
	EV_MULT_UNCORR_ECC,
	EV_PORT_ERROR,
	EV_WRAP_ERROR,
	EV_PARITY_ERROR,
	NR_EVENTS
};


static char *block_name[] = {
	"illegal",
	"mult_illegal",
	"ecc",
	"mult_ecc",
	"uncorr_ecc",
	"mult_uncorr_ecc",
	"port_error",
	"wrap_error",
	"parity_error",
	"alert_n_cs0_dram0_ca_par_error",
	"alert_n_cs0_dram0_crc_error",
	"alert_n_cs0_dram1_ca_par_error",
	"alert_n_cs0_dram1_crc_error",
	"alert_n_cs0_dram2_ca_par_error",
	"alert_n_cs0_dram2_crc_error",
	"alert_n_cs0_dram3_ca_par_error",
	"alert_n_cs0_dram3_crc_error",
	"alert_n_cs0_dram4_ca_par_error",
	"alert_n_cs0_dram4_crc_error",
	"alert_n_cs0_dram5_ca_par_error",
	"alert_n_cs0_dram5_crc_error",
	"alert_n_cs0_dram6_ca_par_error",
	"alert_n_cs0_dram6_crc_error",
	"alert_n_cs0_dram7_ca_par_error",
	"alert_n_cs0_dram7_crc_error",
	"alert_n_cs0_dram8_ca_par_error",
	"alert_n_cs0_dram8_crc_error",
	"alert_n_cs0_dram9_ca_par_error",
	"alert_n_cs0_dram9_crc_error",
	"alert_n_cs0_dram10_ca_par_error",
	"alert_n_cs0_dram10_crc_error",
	"alert_n_cs0_dram11_ca_par_error",
	"alert_n_cs0_dram11_crc_error",
	"alert_n_cs0_dram12_ca_par_error",
	"alert_n_cs0_dram12_crc_error",
	"alert_n_cs0_dram13_ca_par_error",
	"alert_n_cs0_dram13_crc_error",
	"alert_n_cs0_dram14_ca_par_error",
	"alert_n_cs0_dram14_crc_error",
	"alert_n_cs0_dram15_ca_par_error",
	"alert_n_cs0_dram15_crc_error",
	"alert_n_cs0_dram16_ca_par_error",
	"alert_n_cs0_dram16_crc_error",
	"alert_n_cs0_dram17_ca_par_error",
	"alert_n_cs0_dram17_crc_error",
	"alert_n_cs1_dram0_ca_par_error",
	"alert_n_cs1_dram0_crc_error",
	"alert_n_cs1_dram1_ca_par_error",
	"alert_n_cs1_dram1_crc_error",
	"alert_n_cs1_dram2_ca_par_error",
	"alert_n_cs1_dram2_crc_error",
	"alert_n_cs1_dram3_ca_par_error",
	"alert_n_cs1_dram3_crc_error",
	"alert_n_cs1_dram4_ca_par_error",
	"alert_n_cs1_dram4_crc_error",
	"alert_n_cs1_dram5_ca_par_error",
	"alert_n_cs1_dram5_crc_error",
	"alert_n_cs1_dram6_ca_par_error",
	"alert_n_cs1_dram6_crc_error",
	"alert_n_cs1_dram7_ca_par_error",
	"alert_n_cs1_dram7_crc_error",
	"alert_n_cs1_dram8_ca_par_error",
	"alert_n_cs1_dram8_crc_error",
	"alert_n_cs1_dram9_ca_par_error",
	"alert_n_cs1_dram9_crc_error",
	"alert_n_cs1_dram10_ca_par_error",
	"alert_n_cs1_dram10_crc_error",
	"alert_n_cs1_dram11_ca_par_error",
	"alert_n_cs1_dram11_crc_error",
	"alert_n_cs1_dram12_ca_par_error",
	"alert_n_cs1_dram12_crc_error",
	"alert_n_cs1_dram13_ca_par_error",
	"alert_n_cs1_dram13_crc_error",
	"alert_n_cs1_dram14_ca_par_error",
	"alert_n_cs1_dram14_crc_error",
	"alert_n_cs1_dram15_ca_par_error",
	"alert_n_cs1_dram15_crc_error",
	"alert_n_cs1_dram16_ca_par_error",
	"alert_n_cs1_dram16_crc_error",
	"alert_n_cs1_dram17_ca_par_error",
	"alert_n_cs1_dram17_crc_error",
	"alert_n_cs2_dram0_ca_par_error",
	"alert_n_cs2_dram0_crc_error",
	"alert_n_cs2_dram1_ca_par_error",
	"alert_n_cs2_dram1_crc_error",
	"alert_n_cs2_dram2_ca_par_error",
	"alert_n_cs2_dram2_crc_error",
	"alert_n_cs2_dram3_ca_par_error",
	"alert_n_cs2_dram3_crc_error",
	"alert_n_cs2_dram4_ca_par_error",
	"alert_n_cs2_dram4_crc_error",
	"alert_n_cs2_dram5_ca_par_error",
	"alert_n_cs2_dram5_crc_error",
	"alert_n_cs2_dram6_ca_par_error",
	"alert_n_cs2_dram6_crc_error",
	"alert_n_cs2_dram7_ca_par_error",
	"alert_n_cs2_dram7_crc_error",
	"alert_n_cs2_dram8_ca_par_error",
	"alert_n_cs2_dram8_crc_error",
	"alert_n_cs2_dram9_ca_par_error",
	"alert_n_cs2_dram9_crc_error",
	"alert_n_cs2_dram10_ca_par_error",
	"alert_n_cs2_dram10_crc_error",
	"alert_n_cs2_dram11_ca_par_error",
	"alert_n_cs2_dram11_crc_error",
	"alert_n_cs2_dram12_ca_par_error",
	"alert_n_cs2_dram12_crc_error",
	"alert_n_cs2_dram13_ca_par_error",
	"alert_n_cs2_dram13_crc_error",
	"alert_n_cs2_dram14_ca_par_error",
	"alert_n_cs2_dram14_crc_error",
	"alert_n_cs2_dram15_ca_par_error",
	"alert_n_cs2_dram15_crc_error",
	"alert_n_cs2_dram16_ca_par_error",
	"alert_n_cs2_dram16_crc_error",
	"alert_n_cs2_dram17_ca_par_error",
	"alert_n_cs2_dram17_crc_error",
	"alert_n_cs3_dram0_ca_par_error",
	"alert_n_cs3_dram0_crc_error",
	"alert_n_cs3_dram1_ca_par_error",
	"alert_n_cs3_dram1_crc_error",
	"alert_n_cs3_dram2_ca_par_error",
	"alert_n_cs3_dram2_crc_error",
	"alert_n_cs3_dram3_ca_par_error",
	"alert_n_cs3_dram3_crc_error",
	"alert_n_cs3_dram4_ca_par_error",
	"alert_n_cs3_dram4_crc_error",
	"alert_n_cs3_dram5_ca_par_error",
	"alert_n_cs3_dram5_crc_error",
	"alert_n_cs3_dram6_ca_par_error",
	"alert_n_cs3_dram6_crc_error",
	"alert_n_cs3_dram7_ca_par_error",
	"alert_n_cs3_dram7_crc_error",
	"alert_n_cs3_dram8_ca_par_error",
	"alert_n_cs3_dram8_crc_error",
	"alert_n_cs3_dram9_ca_par_error",
	"alert_n_cs3_dram9_crc_error",
	"alert_n_cs3_dram10_ca_par_error",
	"alert_n_cs3_dram10_crc_error",
	"alert_n_cs3_dram11_ca_par_error",
	"alert_n_cs3_dram11_crc_error",
	"alert_n_cs3_dram12_ca_par_error",
	"alert_n_cs3_dram12_crc_error",
	"alert_n_cs3_dram13_ca_par_error",
	"alert_n_cs3_dram13_crc_error",
	"alert_n_cs3_dram14_ca_par_error",
	"alert_n_cs3_dram14_crc_error",
	"alert_n_cs3_dram15_ca_par_error",
	"alert_n_cs3_dram15_crc_error",
	"alert_n_cs3_dram16_ca_par_error",
	"alert_n_cs3_dram16_crc_error",
	"alert_n_cs3_dram17_ca_par_error",
	"alert_n_cs3_dram17_crc_error"
};
/*
 * index = (sizeof(event_mask)/sizoef(event_mask[1])
 *		+ #cs*36 + (par?1:0)*18 + #dram
 */


static const u32 event_mask[NR_EVENTS] = {
	[EV_ILLEGAL]			= 0x00000002,
	[EV_MULT_ILLEGAL]		= 0x00000004,
	[EV_CORR_ECC]			= 0x00000008,
	[EV_MULT_CORR_ECC]		= 0x00000010,
	[EV_UNCORR_ECC]			= 0x00000020,
	[EV_MULT_UNCORR_ECC]		= 0x00000040,
	[EV_PORT_ERROR]			= 0x00000080,
	[EV_WRAP_ERROR]			= 0x00001000,
	[EV_PARITY_ERROR]		= 0x00400000,
};

static const struct event_logging {
	int		 fatal;
	const char *level;
	const char *name;
} event_logging[NR_EVENTS] = {
	[EV_ILLEGAL]		= {0, KERN_ERR, "Illegal access"},
	[EV_MULT_ILLEGAL]	= {0, KERN_ERR, "Illegal access"},
	[EV_CORR_ECC]		= {0, KERN_NOTICE, "Correctable ECC error"},
	[EV_MULT_CORR_ECC]	= {0, KERN_NOTICE, "Correctable ECC error"},
	[EV_UNCORR_ECC]		= {1, KERN_CRIT, "Uncorrectable ECC error"},
	[EV_MULT_UNCORR_ECC]	= {1, KERN_CRIT, "Uncorrectable ECC error"},
	[EV_PORT_ERROR]		= {0, KERN_CRIT, "Port error"},
	[EV_WRAP_ERROR]		= {0, KERN_CRIT, "Wrap error"},
	[EV_PARITY_ERROR]	= {0, KERN_CRIT, "Parity error"},
};

/* Private structure for common edac device */
struct event_counter {
	atomic_t counter;
	int edac_block_idx;
};

struct mc_edac_data {
	struct event_counter events[NR_EVENTS];
	struct event_counter alerts[MAX_CS][MAX_DQ][MPR_ERRORS];
	u8 mpr_page1[MPR_CIRCULAR_BUF_LEN][MAX_DQ][MPR_PAGE_BYTES];
	struct mpr_dump mpr;
	char irq_name[IRQ_NAME_LEN];
	int cs_count;
	int dram_count;
	int irq;
	int latest_mpr_page1_idx;
	raw_spinlock_t mpr_data_lock;
	struct mutex edac_sysfs_data_lock;
	wait_queue_head_t dump_wq;
	wait_queue_head_t event_wq;
	atomic_t dump_ready;
	atomic_t event_ready;
	atomic_t dump_in_progress;
};

struct intel_edac_dev_info {
	struct platform_device *pdev;
	struct mc_edac_data *data;
	char *ctl_name;
	char *blk_name;
	struct work_struct offload_alerts;
	struct work_struct offload_events;
	int is_ddr4;
	int edac_idx;
	u32 sm_region;
	struct regmap *syscon;
	void __iomem *axi2ser3_region;
	struct edac_device_ctl_info *edac_dev;
	void (*check)(struct edac_device_ctl_info *edac_dev);
};


static ssize_t mpr1_dump_show(struct edac_device_ctl_info
				 *edac_dev, char *data)
{
	u8 (*mpr_page1)[MAX_DQ][MPR_PAGE_BYTES];
	unsigned long flags;
	char *buf = NULL;
	ssize_t	 count = 0;
	ssize_t	 total_count = 0;
	int	 i = 0, j = 0;
	int latest_mpr_page1_idx;
	struct intel_edac_dev_info *dev_info = edac_dev->pvt_info;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf == NULL)
		goto no_mem_buffer;

	mpr_page1 = kmalloc_array(MPR_CIRCULAR_BUF_LEN, sizeof(*mpr_page1),
		      GFP_KERNEL);
	if (mpr_page1 == NULL)
		goto no_mem_dump;

	/*
	 * If this cause a performance issue use rcu list, where each node
	 * is 2-dimentional array instead.
	 */
	raw_spin_lock_irqsave(&dev_info->data->mpr_data_lock, flags);
	latest_mpr_page1_idx = dev_info->data->latest_mpr_page1_idx;
	memcpy(mpr_page1, dev_info->data->mpr_page1,
			MPR_CIRCULAR_BUF_LEN * MAX_DQ * MPR_PAGE_BYTES);
	raw_spin_unlock_irqrestore(&dev_info->data->mpr_data_lock, flags);

	/* Now process on copied data ... */
	count = scnprintf(buf+total_count, PAGE_SIZE-total_count,
			"%s", MPR_HDR9);

	total_count += (count > 0 ? count : 0);

	if (dev_info->data->dram_count == MAX_DQ) {
		count = scnprintf(buf+total_count, PAGE_SIZE-total_count,
				"%s", MPR_HDR18);
		total_count += (count > 0 ? count : 0);
	}


	for (i = 0; i < MPR_CIRCULAR_BUF_LEN; ++i) {
		j = (latest_mpr_page1_idx
			+ MPR_CIRCULAR_BUF_LEN - i) % MPR_CIRCULAR_BUF_LEN;

		/* x8 base */
		count = scnprintf(buf+total_count,
				PAGE_SIZE-total_count,
				MPR_FMT9, i+1,
				*((u32 *) &mpr_page1[j][0]),
				*((u32 *) &mpr_page1[j][1]),
				*((u32 *) &mpr_page1[j][2]),
				*((u32 *) &mpr_page1[j][3]),
				*((u32 *) &mpr_page1[j][4]),
				*((u32 *) &mpr_page1[j][5]),
				*((u32 *) &mpr_page1[j][6]),
				*((u32 *) &mpr_page1[j][7]),
				*((u32 *) &mpr_page1[j][8]));

		total_count += (count > 0 ? count : 0);

		/* x16 addition */
		if (dev_info->data->dram_count == MAX_DQ) {
			count = scnprintf(buf+total_count,
					PAGE_SIZE-total_count,
					MPR_FMT16,
					*((u32 *) &mpr_page1[j][9]),
					*((u32 *) &mpr_page1[j][10]),
					*((u32 *) &mpr_page1[j][11]),
					*((u32 *) &mpr_page1[j][12]),
					*((u32 *) &mpr_page1[j][13]),
					*((u32 *) &mpr_page1[j][14]),
					*((u32 *) &mpr_page1[j][15]),
					*((u32 *) &mpr_page1[j][16]),
					*((u32 *) &mpr_page1[j][17]));

			total_count += (count > 0 ? count : 0);
		}
	}

	total_count = scnprintf(data, PAGE_SIZE, "%s\n", buf);

	/* free resourses */
	kfree(mpr_page1);
	kfree(buf);
	return total_count;

no_mem_dump:
	pr_info("Could not allocate memory for smem MPR dump.\n");
	kfree(buf);
	return 0;
no_mem_buffer:
	pr_err("Could not allocate memory for smem MPR buffer.\n");
	return 0;
}

static struct edac_dev_sysfs_attribute device_block_attr[] = {
	{
		.attr = {
			.name = "mpr_page1",
			.mode = (S_IRUGO | S_IWUSR)
		},
		.show = mpr1_dump_show,
		.store = NULL},
	/* End of list */
	{
		.attr = {.name = NULL}
	}
};

static void axxia_mc_sysfs_attributes(struct edac_device_ctl_info *edac_dev)
{
		edac_dev->sysfs_attributes = &device_block_attr[0];
}

static inline void __attribute__((always_inline))
handle_events(struct intel_edac_dev_info *edac_dev,
		struct sm_56xx_denali_ctl_366 *denali_ctl_366)
{
	unsigned long set_val;
	int i;

	for (i = 0; i < NR_EVENTS; ++i) {
		if ((denali_ctl_366->int_status & event_mask[i]) != 0) {
			if (force_restart && event_logging[i].fatal) {
				if (IS_ERR(edac_dev->syscon)) {
					set_val = readl(
						edac_dev->axi2ser3_region
						+ SYSCON_PERSIST_SCRATCH);
					/* set bit 3 in pscratch reg */
					set_val = set_val
						| SMEM_PERSIST_SCRATCH_BIT;
					writel(set_val,
						edac_dev->axi2ser3_region +
						SYSCON_PERSIST_SCRATCH);
				} else {
					regmap_update_bits(edac_dev->syscon,
						SYSCON_PERSIST_SCRATCH,
						SMEM_PERSIST_SCRATCH_BIT,
						SMEM_PERSIST_SCRATCH_BIT);
				}
				pr_emerg("%s uncorrectable error\n",
						edac_dev->ctl_name);
				machine_restart(NULL);
			}
			atomic_inc(&edac_dev->data->events[i].counter);
		}
	}
}

static inline void __attribute__((always_inline))
inc_alert_counter(struct event_counter (*alert)[MAX_DQ][MPR_ERRORS],
			int cs, int dram, u8 val)
{
	/* PARITY */
	if (val & 0x40)
		atomic_inc(&alert[cs][dram][0].counter);

	/* CRC */
	if (val & 0x80)
		atomic_inc(&alert[cs][dram][1].counter);
}

static inline void __attribute__((always_inline))
store_mpr_dump(struct intel_edac_dev_info *edac_dev, int cs)
{
	struct mpr_dump *mpr = &edac_dev->data->mpr;
	int idx;

	edac_dev->data->latest_mpr_page1_idx++;
	/* used smart bitwise op instead of %= MPR_CIRCULAR_BUF_LEN */
	/* MPR_CIRCULAR_BUF_LEN is power of 2 */
	idx = (edac_dev->data->latest_mpr_page1_idx &=
			(MPR_CIRCULAR_BUF_LEN - 1));

	memcpy((char *) &edac_dev->data->mpr_page1[idx],
		(char *) &mpr->dram_0_page[0],
		MAX_DQ * MPR_PAGE_BYTES);
}

static inline void __attribute__((always_inline))
update_alert_counters(struct intel_edac_dev_info *edac_dev, int cs)
{
	/* Casting magic
	 *
	 * mpr.dram_0_page[0] -> dram[0][0]
	 * mpr.dram_0_page[1] -> dram[0][1]
	 * mpr.dram_0_page[2] -> dram[0][2]
	 * mpr.dram_0_page[3] -> dram[0][3]
	 * ...
	 * mpr.dram_1_page[0] -> dram[1][0]
	 * ...
	 * mpr.dram_8_page[0] -> dram[8][0]
	 */
	u8 (*dram)[MPR_PAGE_BYTES] =
		(u8 (*)[MPR_PAGE_BYTES]) (&edac_dev->data->mpr.dram_0_page[0]);
	int i;

	for (i = 0; i < MAX_DQ; ++i)
		inc_alert_counter(edac_dev->data->alerts, cs, i, dram[i][3]);

}

static inline int __attribute__((always_inline))
collect_mpr_dump(struct intel_edac_dev_info *edac_dev, u8 page, int cs)
{
	struct mpr_dump *mpr = &edac_dev->data->mpr;
	unsigned long flags;
	u32 regval;
	int i;

	mpr->mpr_page_id = page;

	for (i = 0; i < MPR_PAGE_BYTES; i++) {
		if (ncr_read(edac_dev->sm_region,
				(SM_56XX_DENALI_CTL_58 + (0x14 * i)),
				4, &regval))
			goto error_read;

		mpr->dram_0_page[i] = regval & 0xff;
		mpr->dram_1_page[i] = ((regval & 0xff00) >> 8);
		mpr->dram_2_page[i] = ((regval & 0xff0000) >> 16);
		mpr->dram_3_page[i] = ((regval & 0xff000000) >> 24);

		if (ncr_read(edac_dev->sm_region,
				(SM_56XX_DENALI_CTL_59 + (0x14 * i)),
				4, &regval))
			goto error_read;

		mpr->dram_4_page[i] = regval & 0xff;
		mpr->dram_5_page[i] = ((regval & 0xff00) >> 8);
		mpr->dram_6_page[i] = ((regval & 0xff0000) >> 16);
		mpr->dram_7_page[i] = ((regval & 0xff000000) >> 24);

		if (ncr_read(edac_dev->sm_region,
				(SM_56XX_DENALI_CTL_60 + (0x14 * i)),
				4, &regval))
			goto error_read;

		mpr->dram_8_page[i] = regval & 0xff;

		if (edac_dev->data->dram_count == MAX_DQ) {
			mpr->dram_9_page[i] = ((regval & 0xff00) >> 8);
			mpr->dram_10_page[i] = ((regval & 0xff0000) >> 16);
			mpr->dram_11_page[i] = ((regval & 0xff000000) >> 24);

			if (ncr_read(edac_dev->sm_region,
						(SM_56XX_DENALI_CTL_60 +
						 (0x14 * i)), 4, &regval))
				goto error_read;

			mpr->dram_12_page[i] = regval & 0xff;
			mpr->dram_13_page[i] = ((regval & 0xff00) >> 8);
			mpr->dram_14_page[i] = ((regval & 0xff0000) >> 16);
			mpr->dram_15_page[i] = ((regval & 0xff000000) >> 24);

			if (ncr_read(edac_dev->sm_region,
						(SM_56XX_DENALI_CTL_61 +
						 (0x14 * i)), 4, &regval))
				goto error_read;

			mpr->dram_16_page[i] = regval & 0xff;
			mpr->dram_17_page[i] = ((regval & 0xff00) >> 8);
		}
	}
	raw_spin_lock_irqsave(&edac_dev->data->mpr_data_lock, flags);
	store_mpr_dump(edac_dev, cs);
	raw_spin_unlock_irqrestore(&edac_dev->data->mpr_data_lock, flags);

	update_alert_counters(edac_dev, cs);
	return 0;

error_read:
	printk_ratelimited("%s: Memory error reading MC mpr page\n",
		dev_name(&edac_dev->pdev->dev));
	return 1;
}

static irqreturn_t
smmon_isr(int interrupt, void *device)
{
	struct intel_edac_dev_info *dev_info = device;
	struct sm_56xx_denali_ctl_366 denali_ctl_366;
	struct sm_56xx_denali_ctl_367 denali_ctl_367;
	struct sm_56xx_denali_ctl_368 denali_ctl_368 = {0};
	struct sm_56xx_denali_ctl_369 denali_ctl_369 = {0, 0, 0};

	/*
	 * NOTE:
	 * ISR function is only reading int_status, and write into int_act
	 * registers.
	 *
	 * - first handle critical events, which might require restart
	 *  (handle_events) and then to the job outside isr
	 * - second collect MPR dump if any exists and then trigger new if
	 *   needed - all outside isr,
	 * - third wake up job outside isr to trigger mpr dump procedure when
	 *   ALERT_N reported (bit [33] is on)
	 */

	if (ncr_read(dev_info->sm_region, SM_56XX_DENALI_CTL_367,
				4, (u32 *) &denali_ctl_367))
		goto error_read;

	if (denali_ctl_367.int_status & 0x4) {

		if (ncr_read(dev_info->sm_region, SM_56XX_DENALI_CTL_366,
			4, (u32 *) &denali_ctl_366))
			goto error_read;

		handle_events(dev_info, &denali_ctl_366);
		atomic_set(&dev_info->data->event_ready, 1);
		wake_up(&dev_info->data->event_wq);

		denali_ctl_368.int_ack =
			(denali_ctl_366.int_status & 0xf8ffffff);

		if (dev_info->is_ddr4) {
			if (denali_ctl_366.int_status & 0x4000000) {
				atomic_set(&dev_info->data->dump_ready, 1);
				wake_up(&dev_info->data->dump_wq);
				denali_ctl_368.int_ack |= 0x4000000;
			}
		}
		if (ncr_write(dev_info->sm_region, SM_56XX_DENALI_CTL_368,
			4, (u32 *) &denali_ctl_368))
			goto error_write;
	}

	if (denali_ctl_367.int_status & 0x2) {
		if (dev_info->is_ddr4) {
			atomic_inc(&dev_info->data->dump_in_progress);
			wake_up(&dev_info->data->dump_wq);
		}
		denali_ctl_369.int_ack = 0x2;
		if (ncr_write(dev_info->sm_region, SM_56XX_DENALI_CTL_369,
			4, (u32 *) &denali_ctl_369))
			goto error_write;
	}

	return IRQ_HANDLED;

error_write:
	printk_ratelimited("%s: Error writing interrupt status\n",
		       dev_name(&dev_info->pdev->dev));
	return IRQ_HANDLED;
error_read:
	printk_ratelimited("%s: Error reading interrupt status\n",
		       dev_name(&dev_info->pdev->dev));
	return IRQ_HANDLED;
}


static void intel_sm_alerts_error_check(struct edac_device_ctl_info *edac_dev)
{
	struct intel_edac_dev_info *dev_info =
			(struct intel_edac_dev_info *) edac_dev->pvt_info;
	struct event_counter (*alerts)[MAX_DQ][MPR_ERRORS] =
			dev_info->data->alerts;
	struct sm_56xx_denali_ctl_57 denali_ctl_57;
	int i, j, k, l;
	u32 counter;

start:
	/* keep hung up monitor happy 90 sec's */
	if (0 == wait_event_timeout(dev_info->data->dump_wq,
		atomic_read(&dev_info->data->dump_in_progress),
		msecs_to_jiffies(ALIVE_NOTIFICATION_PERIOD)))
		goto start;

		/* the only one running workqueue */
	for (i = 0; i < dev_info->data->cs_count; ++i) {

		/* trigger dump */
		if (ncr_read(dev_info->sm_region,
			SM_56XX_DENALI_CTL_57,
			4, (u32 *) &denali_ctl_57))
			goto error_read;

		/* bits [3:2] cs number */
		denali_ctl_57.read_mpr = 4*i + SM_MPR_PAGE;
		if (ncr_write(dev_info->sm_region,
			SM_56XX_DENALI_CTL_57,
			4, (u32 *) &denali_ctl_57))
			goto error_write;

		/* bit [4] trigger dump */
		denali_ctl_57.read_mpr += 16;
		if (ncr_write(dev_info->sm_region,
			SM_56XX_DENALI_CTL_57,
			4, (u32 *) &denali_ctl_57))
			goto error_write;
		/* wait */
		wait_event(dev_info->data->dump_wq,
			   atomic_read(&dev_info->data->dump_ready));

		atomic_set(&dev_info->data->dump_ready, 0);
		/* collect data */
		collect_mpr_dump(dev_info, SM_MPR_PAGE, i);
	}

	/* process all dumps - update counters */

	mutex_lock(&dev_info->data->edac_sysfs_data_lock);
	for (i = 0; i < dev_info->data->cs_count; ++i) {
		for (j = 0;  j < dev_info->data->dram_count; ++j) {
			for (k = 0;  k < MPR_ERRORS; ++k) {
				/*
				 * TODO - How can one determine event type?
				 *	recoverable/unrecoverable
				 */
				counter = atomic_xchg(&alerts[i][j][k].counter,
							0);
				for (l = 0; l < counter; ++l)
					edac_device_handle_ce(edac_dev, 0,
						alerts[i][j][k].edac_block_idx,
						edac_dev->ctl_name);
			}
		}
	}
	mutex_unlock(&dev_info->data->edac_sysfs_data_lock);
	atomic_set(&dev_info->data->dump_in_progress, 0);
	goto start;

error_read:
error_write:
	printk_ratelimited("Could not collect MPR dump.\n");
	atomic_set(&dev_info->data->dump_in_progress, 0);
	goto start;
}

static void intel_sm_events_error_check(struct edac_device_ctl_info *edac_dev)
{
	struct intel_edac_dev_info *dev_info =
			(struct intel_edac_dev_info *) edac_dev->pvt_info;
	struct event_counter *events = dev_info->data->events;
	int i, j;
	u32 counter;

	while (1) {
		if (0 == wait_event_timeout(dev_info->data->event_wq,
			atomic_read(&dev_info->data->event_ready),
			msecs_to_jiffies(ALIVE_NOTIFICATION_PERIOD)))
			continue;

		atomic_set(&dev_info->data->event_ready, 0);

		mutex_lock(&dev_info->data->edac_sysfs_data_lock);
		for (i = 0; i < NR_EVENTS; ++i) {
			counter = atomic_xchg(&events[i].counter, 0);
			for (j = 0; j < counter; ++j) {
				switch (i) {
				/*
				 * TODO - How can one determine event type?
				 *	recoverable/unrecoverable
				 */
				case EV_ILLEGAL:
				case EV_MULT_ILLEGAL:
				case EV_UNCORR_ECC:
				case EV_MULT_UNCORR_ECC:
					edac_device_handle_ue(edac_dev, 0, i,
							edac_dev->ctl_name);
					break;
				case EV_CORR_ECC:
				case EV_MULT_CORR_ECC:
				case EV_PORT_ERROR:
				case EV_WRAP_ERROR:
				case EV_PARITY_ERROR:
					edac_device_handle_ce(edac_dev, 0, i,
							edac_dev->ctl_name);
					break;
				default:
					printk_ratelimited(
						"ERROR EVENT MISSING.\n");
				}
			}
		}
		mutex_unlock(&dev_info->data->edac_sysfs_data_lock);
	}
}

static void axxia_alerts_work(struct work_struct *work)
{
	struct intel_edac_dev_info *dev_info =
		container_of(work, struct intel_edac_dev_info, offload_alerts);

	intel_sm_alerts_error_check(dev_info->edac_dev);
}

static void axxia_events_work(struct work_struct *work)
{
	struct intel_edac_dev_info *dev_info =
		container_of(work, struct intel_edac_dev_info, offload_events);

	intel_sm_events_error_check(dev_info->edac_dev);
}

static int get_active_cs(struct intel_edac_dev_info *dev_info)
{
	struct sm_56xx_denali_ctl_123 denali_ctl_123 = {0};
	int i;
	int cs = 0;

	if (ncr_read(dev_info->sm_region, SM_56XX_DENALI_CTL_123,
		4, (u32 *) &denali_ctl_123)) {
		pr_err("Could not read active CS number.\n");
		return 0;
	}

	for (i = 0; i < MAX_CS; i++)
		if (denali_ctl_123.cs_map & (0x1 << i))
			++cs;

	return cs;
}

static int get_active_dram(struct intel_edac_dev_info *dev_info)
{
	struct sm_56xx_denali_ctl_117 denali_ctl_117 = {0};
	int dram = 0;

	if (ncr_read(dev_info->sm_region, SM_56XX_DENALI_CTL_117,
		4, (u32 *) &denali_ctl_117)) {
		pr_err("Could not read number of lanes.\n");
	}

	if (denali_ctl_117.bank_diff == 0)
		dram = MAX_DQ/2;

	if (denali_ctl_117.bank_diff == 1)
		dram = MAX_DQ;

	return dram;
}

static int intel_edac_mc_probe(struct platform_device *pdev)
{
	struct edac_device_instance *instance;
	struct edac_device_block *block;
	int i, j, k, l;
	int count;
	struct intel_edac_dev_info *dev_info = NULL;
	struct resource *io;
	struct device_node *np = pdev->dev.of_node;
	int irq = -1, rc = 0;
	struct sm_56xx_denali_ctl_00 denali_ctl_00;
	struct sm_56xx_denali_ctl_370 denali_ctl_370;
	struct sm_56xx_denali_ctl_371 denali_ctl_371;
	int cs_count = MAX_CS;
	int dram_count = MAX_DQ;

	count = atomic64_inc_return(&mc_counter);
	if ((count - 1) == MEMORY_CONTROLLERS)
		goto err_nodev;

	dev_info = devm_kzalloc(&pdev->dev, sizeof(*dev_info), GFP_KERNEL);
	if (!dev_info)
		goto err_nomem;

	dev_info->data =
		devm_kzalloc(&pdev->dev, sizeof(*dev_info->data), GFP_KERNEL);
	if (!dev_info->data)
		goto err_noctlinfo;

	init_waitqueue_head(&dev_info->data->dump_wq);
	init_waitqueue_head(&dev_info->data->event_wq);

	raw_spin_lock_init(&dev_info->data->mpr_data_lock);
	mutex_init(&dev_info->data->edac_sysfs_data_lock);

	dev_info->ctl_name = kstrdup(np->name, GFP_KERNEL);
	dev_info->blk_name = "ECC";
	edac_op_state = EDAC_OPSTATE_POLL;

	dev_info->pdev = pdev;
	dev_info->edac_idx = edac_device_alloc_index();
	dev_info->data->irq = 0;

	/* setup all counters */
	for (i = 0; i < NR_EVENTS; ++i)
		atomic_set(&dev_info->data->events[i].counter, 0);

	for (j = 0; j < MAX_CS; ++j) {
		for (l = 0; l < MAX_DQ; ++l) {
			for (k = 0; k < MPR_ERRORS; ++k, ++i) {
				atomic_set(&dev_info->data->
						alerts[j][l][k].counter, 0);
			}
		}
	}

	/* set up dump in progress flag */
	atomic_set(&dev_info->data->dump_in_progress, 0);

	io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!io) {
		dev_err(&pdev->dev, "Unable to get mem resource\n");
		goto err_noctlinfo;
	}
	dev_info->sm_region = io->start;
	dev_info->syscon =
		syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(dev_info->syscon)) {
		pr_info(FMT, np->name);
		dev_info->axi2ser3_region = ioremap(AXI2_SER3_PHY_ADDR,
			AXI2_SER3_PHY_SIZE);
		if (!dev_info->axi2ser3_region) {
			pr_err("ioremap of axi2ser3 region failed\n");
			goto err_noctlinfo;
		}
	}

	cs_count = get_active_cs(dev_info);
	if (cs_count == 0)
		goto err_noctlinfo;

	dram_count = get_active_dram(dev_info);
	if (dram_count == 0)
		goto err_noctlinfo;


	if (ncr_read(dev_info->sm_region, SM_56XX_DENALI_CTL_00,
		4, (u32 *) &denali_ctl_00)) {
		pr_err("Could not read ddr version.\n");
		goto err_noctlinfo;
	}

	dev_info->is_ddr4 = (denali_ctl_00.dram_class == 0xa ? 1 : 0);

	if (dev_info->is_ddr4)
		pr_info("%s supports mpr dump (DDR4).\n", dev_info->ctl_name);
	else {
		pr_info("%s doesn't support mpr dump.\n", dev_info->ctl_name);
		cs_count = 0;
		dram_count = 0;
	}

	dev_info->data->cs_count = cs_count;
	dev_info->data->dram_count = dram_count;

	dev_info->edac_dev =
		edac_device_alloc_ctl_info(0, dev_info->ctl_name,
					 1, dev_info->blk_name,
					 NR_EVENTS +
					 cs_count * dram_count * MPR_ERRORS,
					 0, NULL, 0, dev_info->edac_idx);

	if (!dev_info->edac_dev) {
		pr_info("No memory for edac device\n");
		goto err_noctlinfo;
	}

	instance = &dev_info->edac_dev->instances[0];

	/* It just gives more descriptive name. */
	for (i = 0; i < NR_EVENTS; ++i) {
		block = &instance->blocks[i];
		snprintf(block->name,
			sizeof(block->name),
			 "%s", block_name[i]);
		dev_info->data->events[i].edac_block_idx = i;
	}
	/*
	 * NOTE, please notice that 'i' index is
	 * further used in following loops. This is done
	 * intentionally. Edac is using index for all instances,
	 * each instance shall be however named based on correct
	 * cs, dram, ca/crc. Those might differ between HW versions.
	 * CS 1-4
	 * DRAM 4-16,
	 * CRC/CA Parity - always 2 events.
	 */
	if (dev_info->is_ddr4) {
		for (j = 0; j < cs_count; ++j) {
			for (l = 0; l < dram_count; ++l) {
				for (k = 0; k < MPR_ERRORS; ++k, ++i) {
					int idx =  NR_EVENTS +
						MAX_DQ * MPR_ERRORS * j +
						MPR_ERRORS * l + k;

					dev_info->data->alerts[j][l][k].
						edac_block_idx = i;
					block = &instance->blocks[i];
					snprintf(block->name,
						sizeof(block->name),
						 "%s", block_name[idx]);
				}
			}
		}
	}

	dev_info->edac_dev->pvt_info = dev_info;
	dev_info->edac_dev->dev = &dev_info->pdev->dev;
	dev_info->edac_dev->ctl_name = dev_info->ctl_name;
	dev_info->edac_dev->mod_name = INTEL_EDAC_MOD_STR;
	dev_info->edac_dev->dev_name = dev_name(&dev_info->pdev->dev);
	dev_info->edac_dev->edac_check = NULL;

	if (dev_info->is_ddr4)
		axxia_mc_sysfs_attributes(dev_info->edac_dev);

	if (edac_device_add_device(dev_info->edac_dev) != 0) {
		pr_info("Unable to add edac device for %s\n",
			dev_info->ctl_name);
		goto err_nosysfs;
	}

	snprintf(&dev_info->data->irq_name[0], IRQ_NAME_LEN,
			"%s-mon", dev_info->ctl_name);

	if (dev_info->is_ddr4)
		INIT_WORK(&dev_info->offload_alerts, axxia_alerts_work);

	INIT_WORK(&dev_info->offload_events, axxia_events_work);

	if (dev_info->is_ddr4)
		schedule_work(&dev_info->offload_alerts);
	schedule_work(&dev_info->offload_events);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		pr_err("Could not get irq number.\n");
		goto err_noirq;
	}

	/*
	 * Enable memory controller interrupts.
	 */
	if (dev_info->is_ddr4)
		denali_ctl_370.int_mask = SM_INT_MASK_LOW;
	else
		denali_ctl_370.int_mask = SM_INT_MASK_LOW |
			0x04000000;

	if (ncr_write(dev_info->sm_region, SM_56XX_DENALI_CTL_370,
		4, (u32 *) &denali_ctl_370)) {
		pr_err("Could not write interrupt mask reg (%s - ctl_370).\n",
			dev_info->ctl_name);
		goto err_noirq;
	}

	if (ncr_read(dev_info->sm_region, SM_56XX_DENALI_CTL_371,
		4, (u32 *) &denali_ctl_371)) {
		pr_err("Could not read interrupt mask reg (%s - ctl_371).\n",
			dev_info->ctl_name);
		goto err_noirq;
	}

	denali_ctl_371.int_mask = SM_INT_MASK_HIGH;
	if (ncr_write(dev_info->sm_region, SM_56XX_DENALI_CTL_371,
		4, (u32 *) &denali_ctl_371)) {
		pr_err("Could not write interrupt mask reg (%s - ctl_371).\n",
			dev_info->ctl_name);
		goto err_noirq;
	}

	dev_info->data->irq = irq;
	rc = devm_request_irq(&pdev->dev, irq,
			smmon_isr, IRQF_ONESHOT,
			&dev_info->data->irq_name[0], dev_info);

	if (rc) {
		pr_err("Could not register interrupt handler (%s).\n",
			dev_info->ctl_name);

		dev_info->data->irq = 0;

		/* Mask all interrupts in controller */
		denali_ctl_370.int_mask = SM_INT_MASK_ALL_LOW;
		if (ncr_write(dev_info->sm_region, SM_56XX_DENALI_CTL_370,
					4, (u32 *) &denali_ctl_370)) {
			pr_err("Could not mask interrupts (%s - ctl_370).\n",
					dev_info->ctl_name);
		}

		denali_ctl_371.int_mask = SM_INT_MASK_ALL_HIGH;
		if (ncr_write(dev_info->sm_region, SM_56XX_DENALI_CTL_371,
					4, (u32 *) &denali_ctl_371)) {
			pr_err("Could not mask interrupts (%s - ctl_371).\n",
					dev_info->ctl_name);
		}
		goto err_noirq;
	}
	return 0;

err_noirq:
	if (dev_info->is_ddr4)
		cancel_work_sync(&dev_info->offload_alerts);
	cancel_work_sync(&dev_info->offload_events);

	edac_device_del_device(&dev_info->pdev->dev);

err_nosysfs:
	edac_device_free_ctl_info(dev_info->edac_dev);
err_noctlinfo:
	mutex_destroy(&dev_info->data->edac_sysfs_data_lock);
	atomic64_dec(&mc_counter);
	return 1;
err_nomem:
	atomic64_dec(&mc_counter);
	return -ENOMEM;
err_nodev:
	atomic64_dec(&mc_counter);
	return -ENODEV;
}

static int intel_edac_mc_remove(struct platform_device *pdev)
{
	struct intel_edac_dev_info *dev_info =
		(struct intel_edac_dev_info *) &pdev->dev;

	if (dev_info) {
		if (dev_info->data->irq > 0) {
			disable_irq(dev_info->data->irq);
			devm_free_irq(&pdev->dev,
					dev_info->data->irq, dev_info);

			dev_info->data->irq = 0;

			if (dev_info->is_ddr4)
				cancel_work_sync(&dev_info->offload_alerts);
			cancel_work_sync(&dev_info->offload_events);
		}

		if (dev_info->edac_dev != NULL) {
			edac_device_del_device(&dev_info->pdev->dev);
			edac_device_free_ctl_info(dev_info->edac_dev);
		}

		mutex_destroy(&dev_info->data->edac_sysfs_data_lock);
		atomic64_dec(&mc_counter);
	}
	platform_device_unregister(pdev);
	return 0;
}

static const struct of_device_id intel_edac_smmon_match[] = {
	{ .compatible = "intel,smmon" },
	{ }
};
MODULE_DEVICE_TABLE(platform, intel_edac_smmon_match);

static struct platform_driver intel_edac_mc_driver = {
	.probe = intel_edac_mc_probe,
	.remove = intel_edac_mc_remove,
	.driver = {
		.name = "intel_edac_smmon",
		.of_match_table = intel_edac_smmon_match,
	}
};
module_platform_driver(intel_edac_mc_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marek Majtyka <marekx.majtyka@intel.com>");
MODULE_AUTHOR("Arun Joshi <arun.joshi@intel.com>");
