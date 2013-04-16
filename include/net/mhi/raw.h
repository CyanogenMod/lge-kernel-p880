/*
 * File: mhi/raw.h
 *
 * MHI RAW socket definitions
 *
 * Copyright (C) 2011 Renesas Mobile Corporation. All rights reserved.
 *
 */

#ifndef MHI_RAW_H
#define MHI_RAW_H

#include <linux/types.h>
#include <linux/socket.h>

#include <net/sock.h>


extern int mhi_raw_sock_create(
	struct net *net,
	struct socket *sock,
	int proto,
	int kern );

extern int  mhi_raw_proto_init(void);
extern void mhi_raw_proto_exit(void);


#endif /* MHI_RAW_H */
