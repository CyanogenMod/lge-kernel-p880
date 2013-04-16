/*
 * File: mhdp.h
 *
 * Modem-Host Interface (MHI) - MHDP kernel interface
 *
 * Copyright (C) 2011 Renesas Mobile Corporation. All rights reserved.
 *
 */

#ifndef __NET_MHI_MHDP_H
#define __NET_MHI_MHDP_H

struct mhdp_tunnel_parm {
	char name[IFNAMSIZ];
	char master[IFNAMSIZ];
	int  pdn_id;
};

#define SIOCADDPDNID     (SIOCDEVPRIVATE + 1)
#define SIOCDELPDNID     (SIOCDEVPRIVATE + 2)
#define SIOCRESETMHDP    (SIOCDEVPRIVATE + 3)
extern int sysctl_mhdp_concat_nb_pkt;

#endif /* __NET_MHI_MHDP_H */
