/*
 * File: mhi/dgram.h
 *
 * MHI DGRAM socket definitions
 *
 * Copyright (C) 2011 Renesas Mobile Corporation. All rights reserved.
 *
 */

#ifndef MHI_DGRAM_H
#define MHI_DGRAM_H

#include <linux/types.h>
#include <linux/socket.h>

#include <net/sock.h>


extern int mhi_dgram_sock_create(
	struct net *net,
	struct socket *sock,
	int proto,
	int kern );

extern int  mhi_dgram_proto_init(void);
extern void mhi_dgram_proto_exit(void);


#endif /* MHI_DGRAM_H */
