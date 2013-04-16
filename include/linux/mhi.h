/*
 * file mhi.h
 *
 * Modem-Host Interface (MHI) kernel interface
 *
 * Copyright (C) 2011 Renesas Mobile Corporation. All rights reserved.
 *
 */

#ifndef LINUX_MHI_H
#define LINUX_MHI_H

#include <linux/types.h>
#include <linux/socket.h>
#include <net/sock.h>
#include <asm/byteorder.h>


struct mhi_sock {
	struct sock	sk;
	int		sk_l3proto;
	int		sk_ifindex;
}; 

struct sockaddr_mhi {
	sa_family_t	sa_family;
	int		sa_ifindex;
	__u8		sa_zero[sizeof(struct sockaddr) - sizeof(sa_family_t) - sizeof(int)];
};


static inline struct mhi_sock *mhi_sk(struct sock *sk)
{
	return (struct mhi_sock *)sk;
}

static inline struct sockaddr_mhi *sa_mhi(struct sockaddr *sa)
{
	return (struct sockaddr_mhi *)sa;
}

#endif
