/*
 * File: mhi/sock.h
 *
 * MHI socket definitions
 *
 * Copyright (C) 2011 Renesas Mobile Corporation. All rights reserved.
 *
 */

#ifndef MHI_SOCK_H
#define MHI_SOCK_H

#include <linux/types.h>
#include <linux/socket.h>

#include <net/sock.h>


extern struct proto_ops mhi_socket_ops;

extern int  mhi_sock_rcv_unicast(struct sk_buff *skb, u8 l3prot, u32 l3len);
extern int  mhi_sock_rcv_multicast(struct sk_buff *skb, u8 l3prot, u32 l3len);

extern void mhi_sock_hash(struct sock *sk);
extern void mhi_sock_unhash(struct sock *sk);

extern int  mhi_sock_init(void);
extern void mhi_sock_exit(void);


#endif /* MHI_SOCK_H */
