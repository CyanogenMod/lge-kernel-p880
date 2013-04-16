/*
 * File: mhi/sock.h
 *
 * MHI socket definitions
 *
 * Copyright (C) 2011 Renesas Mobile Corporation. All rights reserved.
 *
 */

#ifndef MHI_SCHED_H
#define MHI_SCHED_H

#define MHI_NOTIFY_QUEUE_LOW     19
#define MHI_NOTIFY_QUEUE_HIGH    20

extern int
mhi_register_queue_notifier(struct Qdisc *sch, struct notifier_block *nb, unsigned long cl);

extern int
mhi_unregister_queue_notifier(struct Qdisc *sch, struct notifier_block *nb, unsigned long cl);


#endif /* MHI_SCHED_H */
