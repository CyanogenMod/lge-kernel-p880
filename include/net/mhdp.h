/*
 * File: mhdp.h
 *
 * Modem-Host Interface (MHI) - MHDP kernel interface
 *
 * Copyright (C) 2011 Renesas Mobile Corporation. All rights reserved.
 *
 */

#ifndef __NET_MHDP_H
#define __NET_MHDP_H

struct mhdp_tunnel_parm {
	char name[IFNAMSIZ];
	int  pdn_id;
	char master_interface[IFNAMSIZ];
};

struct mhdp_tunnel {
	struct mhdp_tunnel	*next;
	struct net_device	*dev;
	struct mhdp_tunnel_parm	parms;
	struct net_device *master_dev;
	struct sk_buff *skb;
};

#define SIOCADDPDNID     (SIOCDEVPRIVATE + 1)
#define SIOCDELPDNID     (SIOCDEVPRIVATE + 2)
#define SIOCRESETMHDP    (SIOCDEVPRIVATE + 3)
 
#endif /* __NET_MHDP_H */
