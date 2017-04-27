/*
 * Texas Instruments Ethernet Switch Driver
 *
 * Copyright (C) 2014 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * Userspace API for Switch Configuration
 */

#ifndef __NET_CONFIG_SWITCH_H__
#define __NET_CONFIG_SWITCH_H__

enum {
	CONFIG_SWITCH_INVALID,
	CONFIG_SWITCH_ADD_MULTICAST,
	CONFIG_SWITCH_DEL_MULTICAST,
	CONFIG_SWITCH_ADD_VLAN,
	CONFIG_SWITCH_DEL_VLAN,
	CONFIG_SWITCH_SET_PORT_CONFIG,
	CONFIG_SWITCH_GET_PORT_CONFIG,
};

/*
 * Pass all unused parameters as zero is recomented.
 */
struct net_switch_config {
	unsigned int cmd;	/* API to be invoked by the kernel driver */

	unsigned int	port;
	unsigned int	vid;		/* VLAN identifier */
	unsigned char	unreg_multi;	/* unreg multicast Egress Ports */
	unsigned char	reg_multi;	/* register multicast Egress ports */
	unsigned char	untag_port;	/* Untag ports */
	unsigned char	addr[6];
	unsigned int	super;
	struct ethtool_cmd ecmd;

	unsigned int ret_type;   /* Return  Success/Failure */
};

#endif /* __NET_CONFIG_SWITCH_H__*/
