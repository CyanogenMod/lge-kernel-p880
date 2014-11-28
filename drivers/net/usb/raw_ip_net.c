/*
 * raw_ip_net.c
 *
 * USB network driver for RAW-IP modems.
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/etherdevice.h>
#include <linux/usb.h>

#define BASEBAND_USB_NET_DEV_NAME		"rmnet%d"

/* ethernet packet ethertype for IP packets */
#define NET_IP_ETHERTYPE		0x08, 0x00

#define	TX_TIMEOUT		10

#ifndef USB_NET_BUFSIZ
#define USB_NET_BUFSIZ				8192
#endif  /* USB_NET_BUFSIZ */

/* maximum interface number supported */
#define MAX_INTFS	5
#define RAWIP_RX_BUFS	16

MODULE_LICENSE("GPL");

/* To support more rmnet interfaces, increase the default max_intfs or
 * pass kernel module parameter.
 * e.g. insmod raw_ip_net.ko max_intfs=5
 */
static int max_intfs = 2;	/* default number of interfaces */

static const unsigned long USB_NET_RAW_IP_INTF[MAX_INTFS] = { 3, 5, 9, 11, 13};
unsigned long usb_net_raw_ip_rx_debug;
unsigned long usb_net_raw_ip_tx_debug;

/* max_intfs should not be changed at runtime */
module_param(max_intfs, int, S_IRUGO);
MODULE_PARM_DESC(max_intfs, "usb net (raw-ip) - max. interfaces supported");
module_param(usb_net_raw_ip_rx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_rx_debug, "usb net (raw-ip) - rx debug");
module_param(usb_net_raw_ip_tx_debug, ulong, 0644);
MODULE_PARM_DESC(usb_net_raw_ip_tx_debug, "usb net (raw-ip) - tx debug");

struct baseband_usb {
	/* semaphore between disconnect/suspend/resume */
	struct semaphore sem;
	/* instance */
	int baseband_index;
	/* network statistics */
	struct net_device_stats stats;
	struct net_device *usb_net_raw_ip_dev;
	/* usb context */
	struct {
		struct usb_driver *driver;
		struct usb_device *device;
		struct usb_interface *interface;
		struct {
			struct {
				unsigned int in;
				unsigned int out;
			} isoch, bulk, interrupt;
		} pipe;
		/* currently active rx urb */
		struct urb *rx_urb;
		/* currently active tx urb */
		struct urb *tx_urb;
		struct usb_anchor tx_urb_deferred;
		struct workqueue_struct *tx_workqueue;
		struct work_struct tx_work;
	} usb;
	/* re-usable rx urb */
	struct urb *urb_r[RAWIP_RX_BUFS];
	void *buff[RAWIP_RX_BUFS];
	/* suspend count */
	int susp_count;
};

static struct baseband_usb *baseband_usb_net[MAX_INTFS] = { 0, 0, 0, 0, 0};

static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb);
static void usb_net_raw_ip_rx_urb_comp(struct urb *urb);
static int usb_net_raw_ip_setup_rx_urb( struct baseband_usb *usb);
static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb);
static void usb_net_raw_ip_free_rx_urb(struct baseband_usb *usb);

static int usb_net_raw_ip_tx_urb_submit(struct baseband_usb *usb,
	struct sk_buff *skb);
static void usb_net_raw_ip_tx_urb_work(struct work_struct *work);
static void usb_net_raw_ip_tx_urb_comp(struct urb *urb);

struct baseband_usb *baseband_usb_create(int index, unsigned int intf);
static int baseband_usb_open(
		int index, struct baseband_usb *usb, struct usb_interface *intf);

static struct net_device_ops usb_net_raw_ip_ops;

static int intf_index(struct usb_interface *intf)
{
	int i;
	for (i = 0; i < max_intfs; ++i) {
		if (USB_NET_RAW_IP_INTF[i] ==
				intf->cur_altsetting->desc.bInterfaceNumber) {
			return i;
		}
	}
	return -1;
}

static int baseband_usb_driver_probe(struct usb_interface *intf,
	const struct usb_device_id *id)
{
	int j;
	struct net_device *net_dev = NULL;
	int err;

	struct usb_device *usb_dev = interface_to_usbdev(intf);

	if (5 == usb_dev->actconfig->desc.bNumInterfaces) {
		pr_debug("%s: XMM 6260 core dump state, do not work with it.\n",
				__func__);
		return -ENODEV;
	}

	pr_debug("%s(%d) { intf %p id %p\n", __func__, __LINE__, intf, id);

	pr_debug("intf->cur_altsetting->desc.bInterfaceNumber %02x\n",
		intf->cur_altsetting->desc.bInterfaceNumber);
	pr_debug("intf->cur_altsetting->desc.bAlternateSetting %02x\n",
		intf->cur_altsetting->desc.bAlternateSetting);
	pr_debug("intf->cur_altsetting->desc.bNumEndpoints %02x\n",
		intf->cur_altsetting->desc.bNumEndpoints);
	pr_debug("intf->cur_altsetting->desc.bInterfaceClass %02x\n",
		intf->cur_altsetting->desc.bInterfaceClass);
	pr_debug("intf->cur_altsetting->desc.bInterfaceSubClass %02x\n",
		intf->cur_altsetting->desc.bInterfaceSubClass);
	pr_debug("intf->cur_altsetting->desc.bInterfaceProtocol %02x\n",
		intf->cur_altsetting->desc.bInterfaceProtocol);
	pr_debug("intf->cur_altsetting->desc.iInterface %02x\n",
		intf->cur_altsetting->desc.iInterface);

	/* register interfaces that are assigned to raw-ip */
	j = intf_index(intf);
	if (j < 0) {
		pr_info("%s: interface %d not in list\n", __func__,
				intf->cur_altsetting->desc.bInterfaceNumber);
		return -ENODEV;
	}

	pr_info("%s: raw_ip using interface %d\n", __func__,
		intf->cur_altsetting->desc.bInterfaceNumber);

	/* register network device */
	net_dev = alloc_netdev(0,
			BASEBAND_USB_NET_DEV_NAME,
			ether_setup);
	if (!net_dev) {
		pr_err("alloc_netdev() failed\n");
		err = -ENOMEM;
		goto error_exit;
	}
	net_dev->netdev_ops = &usb_net_raw_ip_ops;
	net_dev->watchdog_timeo = TX_TIMEOUT;
	random_ether_addr(net_dev->dev_addr);
	net_dev->flags |= IFF_NOARP;
	err = register_netdev(net_dev);
	if (err < 0) {
		pr_err("cannot register network device - %d\n", err);
		goto error_exit;
	}
	baseband_usb_net[j]->usb_net_raw_ip_dev = net_dev;
	pr_debug("registered baseband usb network device"
			" - dev %p name %s\n", net_dev,
			BASEBAND_USB_NET_DEV_NAME);

	/* finish initialize */
	err = baseband_usb_open(j, baseband_usb_net[j], intf);
	if (err < 0) {
		pr_err("cannot finish open network device\n");
		err = -ENOMEM;
		goto error_exit;
	}

	return 0;

error_exit:
	/* unregister network device */
	if (net_dev) {
		unregister_netdev(net_dev);
		free_netdev(net_dev);
	}
	baseband_usb_net[j]->usb_net_raw_ip_dev = (struct net_device *) 0;
	return err;
}

static void baseband_usb_driver_disconnect(struct usb_interface *intf)
{
	int i, j;
	struct urb *urb;

	pr_debug("%s intf %p\n", __func__, intf);

	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			pr_debug("%p != %p\n",
				baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		/* acquire semaphore */
		if (down_interruptible(&baseband_usb_net[i]->sem)) {
			pr_err("%s: cannot acquire semaphore\n", __func__);
			continue;
		}
		/* unregister network device */
		if (baseband_usb_net[i]->usb_net_raw_ip_dev) {
			unregister_netdev(baseband_usb_net[i]->usb_net_raw_ip_dev);
			free_netdev(baseband_usb_net[i]->usb_net_raw_ip_dev);
			baseband_usb_net[i]->usb_net_raw_ip_dev = (struct net_device *) 0;
		}
		/* kill usb tx */
		while ((urb = usb_get_from_anchor(&baseband_usb_net[i]->
			usb.tx_urb_deferred)) != (struct urb *) 0) {
			pr_info("%s: kill deferred tx urb %p\n",
				__func__, urb);
			/* decrement count from usb_get_from_anchor() */
			usb_free_urb(urb);
			/* kill tx urb */
			usb_kill_urb(urb);
			/* free tx urb + tx urb transfer buffer */
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
		}
		if (baseband_usb_net[i]->usb.tx_workqueue) {
			destroy_workqueue(baseband_usb_net[i]
				->usb.tx_workqueue);
			baseband_usb_net[i]->usb.tx_workqueue
				= (struct workqueue_struct *) 0;
		}
		if (baseband_usb_net[i]->usb.tx_urb) {
			usb_kill_urb(baseband_usb_net[i]->usb.tx_urb);
			baseband_usb_net[i]->usb.tx_urb
				= (struct urb *) 0;
		}
		/* kill and stop usb rx */
		for (j = 0; j < RAWIP_RX_BUFS; j++)
			usb_kill_urb(baseband_usb_net[i]->urb_r[j]);
		baseband_usb_net[i]->usb.rx_urb	= (struct urb *) 0;
		usb_net_raw_ip_free_rx_urb(baseband_usb_net[i]);
		/* mark interface as disconnected */
		baseband_usb_net[i]->usb.interface
			= (struct usb_interface *) 0;
		/* clear suspend counter */
		baseband_usb_net[i]->susp_count = 0;
		/* release semaphore */
		up(&baseband_usb_net[i]->sem);
	}

}

#ifdef CONFIG_PM
static int baseband_usb_driver_suspend(struct usb_interface *intf,
	pm_message_t message)
{
	int i, susp_count, j;

	pr_debug("%s intf %p\n", __func__, intf);

	pr_debug("%s: cnt %d intf=%p &intf->dev=%p kobj=%s\n",
			__func__, atomic_read(&intf->dev.power.usage_count),
			intf, &intf->dev, kobject_name(&intf->dev.kobj));

	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			pr_debug("%p != %p\n",
				baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		/* increment suspend count */
		susp_count = (baseband_usb_net[i]->susp_count)++;
		if (susp_count > 0) {
			pr_debug("%s: susp_count %d > 0 (already suspended)\n",
				__func__, susp_count);
			continue;
		}
		if (susp_count < 0) {
			pr_debug("%s: susp_count %d < 0 (ILLEGAL VALUE)\n",
				__func__, susp_count);
			baseband_usb_net[i]->susp_count = 0;
			continue;
		}
		pr_debug("%s: susp_count = %d (suspending...)\n",
			__func__, susp_count);
		/* acquire semaphore */
		if (down_interruptible(&baseband_usb_net[i]->sem)) {
			pr_err("%s: cannot acquire semaphore\n", __func__);
			continue;
		}
		/* kill usb rx */
		pr_debug("%s: kill rx_urb {\n",__func__);
		for (j = 0; j < RAWIP_RX_BUFS; j++)
			usb_kill_urb(baseband_usb_net[i]->urb_r[j]);

		pr_debug("%s: kill rx_urb }\n",__func__);
		baseband_usb_net[i]->usb.rx_urb = (struct urb *) 0;
		/* cancel tx urb work (will restart after resume) */
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("%s: !tx_workqueue\n", __func__);
			up(&baseband_usb_net[i]->sem);
			continue;
		}
		pr_debug("%s: cancel_work_sync {\n",__func__);
		cancel_work_sync(&baseband_usb_net[i]->usb.tx_work);
		pr_debug("%s: cancel_work_sync }\n",__func__);
		/* release semaphore */
		up(&baseband_usb_net[i]->sem);
	}

	return 0;
}

static int baseband_usb_driver_resume(struct usb_interface *intf)
{
	int i, err, susp_count;

	pr_debug("%s intf %p\n", __func__, intf);

	pr_debug("%s: cnt %d intf=%p &intf->dev=%p kobj=%s\n",
			__func__, atomic_read(&intf->dev.power.usage_count),
			intf, &intf->dev, kobject_name(&intf->dev.kobj));

	for (i = 0; i < max_intfs; i++) {
		pr_debug("[%d]\n", i);
		if (!baseband_usb_net[i])
			continue;
		if (baseband_usb_net[i]->usb.interface != intf) {
			pr_debug("%p != %p\n",
				baseband_usb_net[i]->usb.interface, intf);
			continue;
		}
		/* decrement suspend count */
		susp_count = --(baseband_usb_net[i]->susp_count);
		if (susp_count > 0) {
			pr_debug("%s: susp_count %d > 0 (not resuming yet)\n",
				__func__, susp_count);
			continue;
		}
		if (susp_count < 0) {
			pr_debug("%s: susp_count %d < 0 (ILLEGAL VALUE)\n",
				__func__, susp_count);
			baseband_usb_net[i]->susp_count = 0;
			continue;
		}
		pr_debug("%s: susp_count = %d (resuming...)\n",
			__func__, susp_count);
		/* acquire semaphore */
		if (down_interruptible(&baseband_usb_net[i]->sem)) {
			pr_err("%s: cannot acquire semaphore\n", __func__);
			continue;
		}
		/* start usb rx */
		err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net[i]);
		if (err < 0) {
			pr_err("submit rx failed - err %d\n", err);
			up(&baseband_usb_net[i]->sem);
			continue;
		}
		/* restart tx urb work (cancelled in suspend) */
		if (!baseband_usb_net[i]->usb.tx_workqueue) {
			pr_err("%s: !tx_workqueue\n", __func__);
			up(&baseband_usb_net[i]->sem);
			continue;
		}
		queue_work(baseband_usb_net[i]->usb.tx_workqueue,
			&baseband_usb_net[i]->usb.tx_work);
		/* release semaphore */
		up(&baseband_usb_net[i]->sem);
	}

	return 0;
}
static int baseband_usb_driver_reset_resume(struct usb_interface *intf)
{
	pr_debug("%s intf %p\n", __func__, intf);
	return baseband_usb_driver_resume(intf);
}
#endif /* CONFIG_PM */

static struct usb_device_id baseband_usb_driver_id_table[] = {
	/* xmm modem vid, pid */
	{ USB_DEVICE(0x1519, 0x0020), },
	{ },
};

static struct usb_driver baseband_usb_driver = {
		.name = "bb_raw_ip_net",
		.probe = baseband_usb_driver_probe,
		.disconnect = baseband_usb_driver_disconnect,
		.id_table = baseband_usb_driver_id_table,
#ifdef CONFIG_PM
		.suspend = baseband_usb_driver_suspend,
		.resume = baseband_usb_driver_resume,
		.reset_resume = baseband_usb_driver_reset_resume,
		.supports_autosuspend = 1,
#endif
};

MODULE_DEVICE_TABLE(usb, baseband_usb_driver_id_table);

static void find_usb_pipe(struct baseband_usb *usb)
{
	struct usb_device *usbdev = usb->usb.device;
	struct usb_interface *intf = usb->usb.interface;
	unsigned char numendpoint = intf->cur_altsetting->desc.bNumEndpoints;
	struct usb_host_endpoint *endpoint = intf->cur_altsetting->endpoint;
	unsigned char n;

	for (n = 0; n < numendpoint; n++) {
		if (usb_endpoint_is_isoc_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] isochronous in\n", n);
			usb->usb.pipe.isoch.in = usb_rcvisocpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_isoc_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] isochronous out\n", n);
			usb->usb.pipe.isoch.out = usb_sndisocpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_bulk_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] bulk in\n", n);
			usb->usb.pipe.bulk.in = usb_rcvbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_bulk_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] bulk out\n", n);
			usb->usb.pipe.bulk.out = usb_sndbulkpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_int_in(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] interrupt in\n", n);
			usb->usb.pipe.interrupt.in = usb_rcvintpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else if (usb_endpoint_is_int_out(&endpoint[n].desc)) {
			pr_debug("endpoint[%d] interrupt out\n", n);
			usb->usb.pipe.interrupt.out = usb_sndintpipe(usbdev,
				endpoint[n].desc.bEndpointAddress);
		} else {
			pr_debug("endpoint[%d] skipped\n", n);
		}
	}
}

void baseband_usb_close(struct baseband_usb *usb);

struct baseband_usb *baseband_usb_create(int index, unsigned int intf)
{
	struct baseband_usb *usb;

	pr_debug("baseband_usb_create {\n");

	/* allocate baseband usb structure */
	usb = kzalloc(sizeof(struct baseband_usb),
		GFP_KERNEL);
	if (!usb)
		return (struct baseband_usb *) 0;

	/* create semaphores */
	sema_init(&usb->sem, 1);

	pr_debug("baseband_usb_create }\n");
	return usb;
}

static int baseband_usb_open(
		int index, struct baseband_usb *usb, struct usb_interface *intf)
{
	int err;
	char name[32];

	pr_debug("baseband_usb_open {\n");

	/* open usb interface */
	usb->baseband_index = index;
	usb->usb.driver = &baseband_usb_driver;
	usb->usb.device = interface_to_usbdev(intf);
	usb->usb.interface = intf;
	find_usb_pipe(usb);
	usb->usb.rx_urb = (struct urb *) 0;
	usb->usb.tx_urb = (struct urb *) 0;
	pr_debug("usb->usb.driver->name %s\n", usb->usb.driver->name);
	pr_debug("usb->usb.device %p\n", usb->usb.device);
	pr_debug("usb->usb.interface %p\n", usb->usb.interface);
	pr_debug("usb->usb.pipe.isoch.in %x\n", usb->usb.pipe.isoch.in);
	pr_debug("usb->usb.pipe.isoch.out %x\n", usb->usb.pipe.isoch.out);
	pr_debug("usb->usb.pipe.bulk.in %x\n", usb->usb.pipe.bulk.in);
	pr_debug("usb->usb.pipe.bulk.out %x\n", usb->usb.pipe.bulk.out);
	pr_debug("usb->usb.pipe.interrupt.in %x\n", usb->usb.pipe.interrupt.in);
	pr_debug("usb->usb.pipe.interrupt.out %x\n",
		usb->usb.pipe.interrupt.out);

	/* start usb tx */
	if (!baseband_usb_net[index]->usb.tx_workqueue) {
		sprintf(name, "raw_ip_tx_wq-%d",
			baseband_usb_net[index]->baseband_index);
		baseband_usb_net[index]->usb.tx_workqueue
			= create_singlethread_workqueue(name);
		if (!baseband_usb_net[index]->usb.tx_workqueue) {
			pr_err("cannot create workqueue\n");
			err = -ENOMEM;
			goto error_exit;
		}
	}
	/* start usb rx */
	err = usb_net_raw_ip_setup_rx_urb(baseband_usb_net[index]);
	if (err < 0) {
		pr_err("setup reusable rx urb failed - err %d\n", err);
		goto error_exit;
	}
	err = usb_net_raw_ip_rx_urb_submit(baseband_usb_net[index]);
	if (err < 0) {
		pr_err("submit rx failed - err %d\n", err);
		goto error_exit;
	}

	/* clear suspend counter */
	usb->susp_count = 0;

	pr_debug("baseband_usb_open }\n");
	return 0;

error_exit:
	/* stop usb tx */
	if (baseband_usb_net[index]->usb.tx_workqueue) {
		destroy_workqueue(baseband_usb_net[index]
			->usb.tx_workqueue);
		baseband_usb_net[index]->usb.tx_workqueue
			= (struct workqueue_struct *) 0;
	}
	if (baseband_usb_net[index]->usb.tx_urb) {
		usb_kill_urb(baseband_usb_net[index]->usb.tx_urb);
		baseband_usb_net[index]->usb.tx_urb
			= (struct urb *) 0;
	}
	/* stop usb rx */
	if (baseband_usb_net[index]->usb.rx_urb) {
		usb_kill_urb(baseband_usb_net[index]->usb.rx_urb);
		baseband_usb_net[index]->usb.rx_urb
			= (struct urb *) 0;
	}
	usb_net_raw_ip_free_rx_urb(baseband_usb_net[index]);
	return err;
}

void baseband_usb_close(struct baseband_usb *usb)
{
	pr_debug("baseband_usb_close {\n");

	/* check input */
	if (!usb)
		return;

	/* close usb driver */
	usb->usb.driver = (struct usb_driver *) 0;

	/* destroy semaphores */
	memset(&usb->sem, 0, sizeof(usb->sem));

	/* free baseband usb structure */
	kfree(usb);

	pr_debug("baseband_usb_close }\n");
}

static int baseband_usb_netdev_init(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_init\n");
	return 0;
}

static void baseband_usb_netdev_uninit(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_uninit\n");
}

static int baseband_usb_netdev_open(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_open\n");
	netif_start_queue(dev);
	return 0;
}

static int baseband_usb_netdev_stop(struct net_device *dev)
{
	pr_debug("baseband_usb_netdev_stop\n");
	netif_stop_queue(dev);
	return 0;
}

static netdev_tx_t baseband_usb_netdev_start_xmit(
	struct sk_buff *skb, struct net_device *dev)
{
	int i;
	struct baseband_usb *usb;
	int err;

	pr_debug("baseband_usb_netdev_start_xmit\n");

	/* check input */
	if (!skb) {
		pr_err("no skb\n");
		return NETDEV_TX_BUSY;
	}
	if (!dev) {
		pr_err("no net dev\n");
		return NETDEV_TX_BUSY;
	}

	/* find index of network device which is transmitting */
	for (i = 0; i < max_intfs; i++) {
		if (baseband_usb_net[i]->usb_net_raw_ip_dev == dev)
			break;
	}
	if (i >= max_intfs) {
		pr_err("unknown net dev %p\n", dev);
		return NETDEV_TX_BUSY;
	}
	usb = baseband_usb_net[i];

	/* autoresume if suspended */
	if (usb->usb.interface) {
		usb_autopm_get_interface_async(usb->usb.interface);
	} else {
		pr_err("%s: tx get interface error\n", __func__);
		netif_stop_queue(dev);
		usb->stats.tx_errors++;
		return NETDEV_TX_BUSY;
	}

	/* submit tx urb */
	err = usb_net_raw_ip_tx_urb_submit(usb, skb);
	if (err < 0) {
		pr_err("%s: tx urb submit error\n", __func__);
		netif_stop_queue(dev);
		usb->stats.tx_errors++;
		return NETDEV_TX_BUSY;
	}

	return NETDEV_TX_OK;
}

static struct net_device_stats *baseband_usb_netdev_get_stats(
				struct net_device *dev)
{
	int i;
	for (i = 0; i < max_intfs; i++) {
		if (dev == baseband_usb_net[i]->usb_net_raw_ip_dev) {
			pr_debug("%s idx(%d)\n", __func__, i);
			return &baseband_usb_net[i]->stats;
		}
	}
	pr_debug("%s mismatch dev, default idx(0)\n", __func__);
	return &baseband_usb_net[0]->stats;
}

static struct net_device_ops usb_net_raw_ip_ops = {
	.ndo_init =		baseband_usb_netdev_init,
	.ndo_uninit =		baseband_usb_netdev_uninit,
	.ndo_open =		baseband_usb_netdev_open,
	.ndo_stop =		baseband_usb_netdev_stop,
	.ndo_start_xmit =	baseband_usb_netdev_start_xmit,
	.ndo_get_stats = baseband_usb_netdev_get_stats,
};

static int usb_net_raw_ip_rx_urb_submit(struct baseband_usb *usb)
{
	struct urb *urb;
	void *buf;
	int err, j;

	pr_debug("usb_net_raw_ip_rx_urb_submit { usb %p\n", usb);

	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}
	if (!usb->usb.interface) {
		pr_err("usb interface disconnected - not submitting rx urb\n");
		return -EINVAL;
	}
	for (j = 0; j < RAWIP_RX_BUFS; j++) {
		urb = usb->urb_r[j];
		buf = usb->buff[j];
		if (!urb || !buf) {
			pr_err("no reusable rx urb found\n");
			return -ENOMEM;
		}

		usb_fill_bulk_urb(urb, usb->usb.device, usb->usb.pipe.bulk.in,
				buf, USB_NET_BUFSIZ,
				usb_net_raw_ip_rx_urb_comp,
				usb);
		urb->transfer_flags = 0;

		/* submit rx urb */
		usb_mark_last_busy(usb->usb.device);
		usb->usb.rx_urb = urb;
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err < 0) {
			pr_err("usb_submit_urb() failed - err %d\n", err);
			usb->usb.rx_urb = (struct urb *) 0;
			return err;
		}
	}

	pr_debug("usb_net_raw_ip_rx_urb_submit }\n");
	return err;
}

static void usb_net_raw_ip_rx_urb_comp(struct urb *urb)
{
	struct baseband_usb *usb;
	int i, err;
	struct sk_buff *skb;
	unsigned char *dst;
	unsigned char ethernet_header[14] = {
		/* Destination MAC */
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		/* Source MAC */
		0x00, 0x00,
		0x00, 0x00,
		0x00, 0x00,
		/* EtherType */
		NET_IP_ETHERTYPE,
	};

	/* check input */
	if (!urb) {
		pr_err("no urb\n");
		return;
	}
	pr_debug("usb_net_raw_ip_rx_urb_comp { urb %p\n", urb);

	usb = (struct baseband_usb *)urb->context;
	i = usb->baseband_index;
	switch (urb->status) {
	case 0:
		break;
	case -ESHUTDOWN:
		/* fall through */
		pr_info("%s: rx urb %p - link shutdown %d\n",
			__func__, urb, urb->status);
		goto err_exit;
	case -EPROTO:
		pr_info("%s: rx urb %p - link shutdown %d EPROTO\n",
			__func__, urb, urb->status);
		goto err_exit;
	case -ENOENT:
		pr_debug("%s: rx urb %p status %d\n",
				__func__, urb, urb->status);
		break;
	default:
		pr_debug("%s: rx urb %p - status %d\n",
			__func__, urb, urb->status);
		break;
	}

	/* put rx urb data in rx buffer */
	if (urb->actual_length > 0) {
		pr_debug("usb_net_raw_ip_rx_urb_comp - "
			"urb->actual_length %d\n", urb->actual_length);
		/* allocate skb with space for
		 * - dummy ethernet header
		 * - rx IP packet from modem
		 */
		skb = netdev_alloc_skb(usb->usb_net_raw_ip_dev,
			NET_IP_ALIGN + 14 + urb->actual_length);
		if (skb) {
			/* generate a dummy ethernet header
			 * since modem sends IP packets without
			 * any ethernet headers
			 */
			memcpy(ethernet_header + 0, usb->usb_net_raw_ip_dev->dev_addr, 6);
			memcpy(ethernet_header + 6, "0x01\0x02\0x03\0x04\0x05\0x06", 6);
			/* fill skb with
			 * - dummy ethernet header
			 * - rx IP packet from modem
			 */
			skb_reserve(skb, NET_IP_ALIGN);
			dst = skb_put(skb, 14);
			memcpy(dst, ethernet_header, 14);
			if ((((unsigned char *) urb->transfer_buffer)[0]
				& 0xf0) == 0x60) {
				/* ipv6 ether type */
				dst[12] = 0x86;
				dst[13] = 0xdd;
			}
			dst = skb_put(skb, urb->actual_length);
			memcpy(dst, urb->transfer_buffer, urb->actual_length);
			skb->protocol = eth_type_trans(skb,
				usb->usb_net_raw_ip_dev);
			pr_debug("%s: ntohs(skb->protocol) %04x (%s)\n",
				__func__, ntohs(skb->protocol),
				(ntohs(skb->protocol) == 0x0800)
					? "IPv4"
					: (ntohs(skb->protocol) == 0x86dd)
					? "IPv6"
					: "unknown");
			pr_debug("%s: %02x %02x %02x %02x\n", __func__,
				((unsigned char *)urb->transfer_buffer)[0],
				((unsigned char *)urb->transfer_buffer)[1],
				((unsigned char *)urb->transfer_buffer)[2],
				((unsigned char *)urb->transfer_buffer)[3]);
			/* pass skb to network stack */
			if (netif_rx(skb) < 0) {
				pr_err("usb_net_raw_ip_rx_urb_comp_work - "
					"netif_rx(%p) failed\n", skb);
				kfree_skb(skb);
				usb->stats.rx_errors++;
			} else {
				usb->stats.rx_packets++;
				usb->stats.rx_bytes +=
				    (14 + urb->actual_length);
			}
		} else {
			pr_err("usb_net_raw_ip_rx_urb_comp_work - "
				"netdev_alloc_skb() failed\n");
			usb->stats.rx_dropped++;
		}
	}

	/* do not submit urb if interface is suspending */
	if (urb->status == -ENOENT)
		return;

	/* submit this rx urb again */
	usb_mark_last_busy(usb->usb.device);
	usb->usb.rx_urb = urb;
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		pr_err("usb_submit_urb() failed - err %d\n", err);
		usb->usb.rx_urb = NULL;
		return;
	}
	pr_debug("usb_net_raw_ip_rx_urb_comp }\n");
	return;

err_exit:
	/* mark rx urb complete */
	usb->usb.rx_urb = (struct urb *) 0;

	pr_debug("usb_net_raw_ip_rx_urb_comp }\n");
	usb->stats.rx_dropped++;
	return;
}

static int usb_net_raw_ip_setup_rx_urb( struct baseband_usb *usb)
{
	int i;
	pr_debug("usb_net_raw_ip_setup_rx_urb {\n");

	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < RAWIP_RX_BUFS; i++) {
		/* allocate reusable rx urbs */
		usb->urb_r[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!usb->urb_r[i]) {
			pr_err("usb_alloc_urb() failed\n");
			return -ENOMEM;
		}
		usb->buff[i] = kzalloc(USB_NET_BUFSIZ, GFP_KERNEL);
		if (!usb->buff[i]) {
			pr_err("usb buffer kzalloc() failed\n");
			usb_free_urb(usb->urb_r[i]);
			usb->urb_r[i] = (struct urb *) 0;
			return -ENOMEM;
		}
	}

	pr_debug("usb_net_raw_setup_ip_rx_urb }\n");
	return 0;
}

static void usb_net_raw_ip_free_rx_urb(struct baseband_usb *usb)
{
	int i;
	pr_debug("usb_net_raw_ip_free_rx_urb {\n");

	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return;
	}

	for (i = 0; i < RAWIP_RX_BUFS; i++) {
		/* free reusable rx urb */
		if (usb->urb_r[i]) {
			pr_debug("%s: freeing urb %p\n", __func__,
							usb->urb_r[i]);
			usb_free_urb(usb->urb_r[i]);
		}

		kfree(usb->buff[i]);
		usb->buff[i] = NULL;
	}

	pr_debug("usb_net_raw_ip_free_rx_urb }\n");
}

static int usb_net_raw_ip_tx_urb_submit(struct baseband_usb *usb,
	struct sk_buff *skb)
{
	struct urb *urb;
	unsigned char *buf;
	int err;

	pr_debug("usb_net_raw_ip_tx_urb_submit {\n");

	/* check input */
	if (!usb) {
		pr_err("%s: !usb\n", __func__);
		return -EINVAL;
	}
	if (!usb->usb.interface) {
		pr_err("usb interface disconnected - not submitting tx urb\n");
		return -EINVAL;
	}
	if (!skb) {
		pr_err("%s: !skb\n", __func__);
		usb_autopm_put_interface_async(usb->usb.interface);
		usb->stats.tx_dropped++;
		return -EINVAL;
	}

	/* allocate urb */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		pr_err("usb_alloc_urb() failed\n");
		usb_autopm_put_interface_async(usb->usb.interface);
		usb->stats.tx_dropped++;
		return -ENOMEM;
	}
	buf = kzalloc(skb->len - 14, GFP_ATOMIC);
	if (!buf) {
		pr_err("usb buffer kzalloc() failed\n");
		usb_free_urb(urb);
		usb_autopm_put_interface_async(usb->usb.interface);
		usb->stats.tx_dropped++;
		return -ENOMEM;
	}
	err = skb_copy_bits(skb, 14, buf, skb->len - 14);
	if (err < 0) {
		pr_err("skb_copy_bits() failed - %d\n", err);
		kfree(buf);
		usb_free_urb(urb);
		usb_autopm_put_interface_async(usb->usb.interface);
		usb->stats.tx_errors++;
		return err;
	}
	usb_fill_bulk_urb(urb, usb->usb.device, usb->usb.pipe.bulk.out,
		buf, skb->len - 14,
		usb_net_raw_ip_tx_urb_comp,
		usb);
	urb->transfer_flags = URB_ZERO_PACKET;
	pr_debug("%s: ntohs(skb->protocol) %04x (%s)\n",
		__func__, ntohs(skb->protocol),
		(ntohs(skb->protocol) == 0x0800)
			? "IPv4"
			: (ntohs(skb->protocol) == 0x86dd)
			? "IPv6"
			: "unknown");
	pr_debug("%s: %02x %02x %02x %02x\n", __func__,
		((unsigned char *)urb->transfer_buffer)[0],
		((unsigned char *)urb->transfer_buffer)[1],
		((unsigned char *)urb->transfer_buffer)[2],
		((unsigned char *)urb->transfer_buffer)[3]);

	/* queue tx urb work */
	usb_anchor_urb(urb, &usb->usb.tx_urb_deferred);
	queue_work(usb->usb.tx_workqueue, &usb->usb.tx_work);

	/* free skb */
	consume_skb(skb);

	pr_debug("usb_net_raw_ip_tx_urb_submit }\n");
	return 0;
}

static void usb_net_raw_ip_tx_urb_work(struct work_struct *work)
{
	struct baseband_usb *usb
		= container_of(work, struct baseband_usb, usb.tx_work);
	struct urb *urb;
	int err;

	pr_debug("usb_net_raw_ip_tx_urb_work {\n");

	/* check if tx urb(s) queued */
	if (usb == NULL ||
		(!usb->usb.tx_urb &&
		usb_anchor_empty(&usb->usb.tx_urb_deferred))) {
		pr_debug("%s: nothing to do!\n", __func__);
		return;
	}

	/* check if usb interface disconnected */
	if (!usb->usb.interface) {
		pr_err("%s: not submitting tx urb -interface disconnected\n",
			__func__);
		return;
	}

	/* check if suspended */
	if (usb->susp_count > 0) {
		pr_debug("%s: usb->susp_count %d > 0 (suspended)\n",
			__func__, usb->susp_count);
		return;
	}

	/* submit queued tx urb(s) */
	while ((urb = usb_get_from_anchor(&usb->usb.tx_urb_deferred))
		!= (struct urb *) 0) {
		/* decrement count from usb_get_from_anchor() */
		usb_free_urb(urb);
		/* check if usb interface disconnected */
		if (!usb->usb.interface) {
			pr_err("%s: not submitting tx urb %p"
				" - interface disconnected\n",
				__func__, urb);
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
			usb->stats.tx_errors++;
			continue;
		}
		/* autoresume before tx */
		usb_mark_last_busy(usb->usb.device);
		/* submit tx urb */
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err < 0) {
			pr_err("%s: usb_submit_urb(%p) failed - err %d\n",
				__func__, urb, err);
			usb_autopm_put_interface_async(usb->usb.interface);
			if (urb->transfer_buffer) {
				kfree(urb->transfer_buffer);
				urb->transfer_buffer = (void *) 0;
			}
			usb_free_urb(urb);
			usb->stats.tx_errors++;
			continue;
		}
		/* free tx urb
		 * - actual urb free occurs when refcnt which was incremented
		 *   in usb_submit_urb is decremented to 0 (usually after urb
		 *   completion function returns)
		 * - tx urb transfer buffer will be freed in urb completion
		 *   function
		 */
		usb_free_urb(urb);
	}

	pr_debug("usb_net_raw_ip_tx_urb_work }\n");
}

static void usb_net_raw_ip_tx_urb_comp(struct urb *urb)
{
	struct baseband_usb *usb;

	pr_debug("usb_net_raw_ip_tx_urb_comp {\n");

	/* check input */
	if (!urb) {
		pr_err("no urb\n");
		return;
	}
	usb = (struct baseband_usb *)urb->context;
	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
		/* fall through */
	case -ESHUTDOWN:
		/* fall through */
	case -EPROTO:
		pr_info("%s: tx urb %p - link shutdown %d\n",
			__func__, urb, urb->status);
		usb->stats.tx_dropped++;
		usb_autopm_put_interface_async(usb->usb.interface);
		goto err_exit;
	default:
		pr_debug("%s: tx urb %p - status %d\n",
			__func__, urb, urb->status);
		break;
	}
	if (urb->status)
		usb->stats.tx_errors++;
	else {
		usb->stats.tx_packets++;
		usb->stats.tx_bytes += urb->transfer_buffer_length;
	}

	/* autosuspend after tx completed */
	if (!usb->usb.interface) {
		pr_err("%s: usb interface disconnected"
			" before tx urb completed!\n",
			__func__);
		goto err_exit;
	}
	usb_autopm_put_interface_async(usb->usb.interface);

err_exit:
	/* free tx urb transfer buffer */
	if (urb->transfer_buffer) {
		kfree(urb->transfer_buffer);
		urb->transfer_buffer = (void *) 0;
	}
	pr_debug("usb_net_raw_ip_tx_urb_comp }\n");
}

static int usb_net_raw_ip_init(void)
{
	int i;
	int err;

	pr_debug("usb_net_raw_ip_init { max_intfs %d\n", max_intfs);

	err = usb_register(&baseband_usb_driver);
	if (err < 0) {
		pr_err("cannot open usb driver - err %d\n", err);
		return err;
	}

	/* create multiple raw-ip network devices */
	for (i = 0; i < max_intfs; i++) {
		/* open baseband usb */
		baseband_usb_net[i] = baseband_usb_create(i,
						USB_NET_RAW_IP_INTF[i]);
		if (!baseband_usb_net[i]) {
			pr_err("cannot open baseband usb net\n");
			err = -1;
			goto error_exit;
		}
		init_usb_anchor(&baseband_usb_net[i]->usb.tx_urb_deferred);
		/* init usb tx; start tx/rx workqueue after probe */
		INIT_WORK(&baseband_usb_net[i]->usb.tx_work,
			usb_net_raw_ip_tx_urb_work);
	}

	pr_debug("usb_net_raw_ip_init }\n");
	return 0;

error_exit:
	/* destroy multiple raw-ip network devices */
	for (i = 0; i < max_intfs; i++) {
		/* close baseband usb */
		if (baseband_usb_net[i]) {
			/* close usb */
			baseband_usb_close(baseband_usb_net[i]);
			baseband_usb_net[i] = (struct baseband_usb *) 0;
		}
	}
	usb_deregister(&baseband_usb_driver);

	return err;
}

static void usb_net_raw_ip_exit(void)
{
	struct baseband_usb *usb;
	int i, j;

	pr_debug("usb_net_raw_ip_exit {\n");

	/* destroy multiple raw-ip network devices */
	for (i = 0; i < max_intfs; i++) {
		usb = baseband_usb_net[i];
		/* stop baseband usb urbs */
		if (usb) {
			if (usb->usb.interface)
				usb_autopm_get_interface(usb->usb.interface);

			if (usb->usb.tx_workqueue)
				cancel_work_sync(&usb->usb.tx_work);

			/* stop usb tx */
			pr_debug("%s: kill tx urb\n", __func__);
			if (usb->usb.tx_urb) {
				usb_kill_urb(usb->usb.tx_urb);
				usb->usb.tx_urb = NULL;
			}

			/* stop usb rx */
			pr_debug("%s: kill rx urbs\n", __func__);
			for (j = 0; j < RAWIP_RX_BUFS; j++) {
				if (usb->urb_r[j])
					usb_kill_urb(usb->urb_r[j]);
			}
			usb_net_raw_ip_free_rx_urb(usb);

			if (usb->usb.interface)
				usb_autopm_put_interface(usb->usb.interface);
		}

		pr_debug("%s: unregister netdev\n", __func__);
		/* unregister network device */
		if (baseband_usb_net[i]->usb_net_raw_ip_dev) {
			unregister_netdev(baseband_usb_net[i]->usb_net_raw_ip_dev);
			free_netdev(baseband_usb_net[i]->usb_net_raw_ip_dev);
			baseband_usb_net[i]->usb_net_raw_ip_dev = (struct net_device *) 0;
		}
		if (usb) {
			pr_debug("%s: destroy tx workqueue\n", __func__);
			if (usb->usb.tx_workqueue) {
				destroy_workqueue(usb->usb.tx_workqueue);
				usb->usb.tx_workqueue = NULL;
			}

			/* close usb */
			baseband_usb_close(usb);
			baseband_usb_net[i] = NULL;
		}
	}

	pr_debug("close usb driver {\n");
	usb_deregister(&baseband_usb_driver);
	pr_debug("close usb driver }\n");

	pr_debug("usb_net_raw_ip_exit }\n");
}

module_init(usb_net_raw_ip_init)
module_exit(usb_net_raw_ip_exit)
