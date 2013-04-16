/*
 * File: net/af_mhi.h
 *
 * MHI Protocol Family kernel definitions
 *
 * Copyright (C) 2011 Renesas Mobile Corporation. All rights reserved.
 *
 */

#ifndef __LINUX_NET_AFMHI_H
#define __LINUX_NET_AFMHI_H

#include <linux/types.h>
#include <linux/socket.h>

#include <net/sock.h>


extern int mhi_register_protocol(int protocol);
extern int mhi_unregister_protocol(int protocol);
extern int mhi_protocol_registered(int protocol);

extern int mhi_skb_send(struct sk_buff *skb, struct net_device *dev, u8 proto );


#endif /* __LINUX_NET_AFMHI_H */ 
