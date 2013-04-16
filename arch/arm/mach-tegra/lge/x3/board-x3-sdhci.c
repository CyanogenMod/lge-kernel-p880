/*
 * arch/arm/mach-tegra/board-x3-sdhci.c
 *
 * Copyright (C) 2011 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/fs.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/io_dpd.h>

#include <mach-tegra/gpio-names.h>
#include <mach-tegra/board.h>
#include <lge/board-x3.h>
#include "../../../include/asm/hw_irq.h" //                                                   

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#endif

#include "LG_CC_Table.h"

//                                                         
#define X3_WLAN_RST	TEGRA_GPIO_PV3
//                                                       
#define X3_WLAN_WOW	TEGRA_GPIO_PU6
#define X3_SD_CD TEGRA_GPIO_PW5
#define X3_SD_WP TEGRA_GPIO_PT3

#define NV_WIFI_MACADDR "/data/misc/wifi/config_mac"
#define ETHER_ADDR_LEN 6

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int x3_wifi_status_register(void (*callback)(int , void *), void *);

//                                                         
static int x3_wifi_reset(int on);
static int x3_wifi_power(int on);
static int x3_wifi_set_carddetect(int val);
static int x3_wifi_get_mac_addr(unsigned char *buf);
static void * x3_wifi_get_country_code(char *ccode);


#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160 *2
#define PREALLOC_WLAN_SECTION_HEADER	24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)  //20K *2
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)  //20K *2
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)  //80K *2
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024) //160K* 2

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM	17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_static_scan_buf0;
void *wlan_static_scan_buf1;
static void *x3_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int x3_init_wlan_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wlan_mem_array[i].mem_ptr =
				kmalloc(wlan_mem_array[i].size, GFP_KERNEL);

		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	wlan_static_scan_buf0 = kmalloc (65536, GFP_KERNEL);
	if(!wlan_static_scan_buf0)
		goto err_mem_alloc;
	wlan_static_scan_buf1 = kmalloc (65536, GFP_KERNEL);
	if(!wlan_static_scan_buf1)
		goto err_mem_alloc;

	printk("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

static struct wifi_platform_data x3_wifi_control = {
	.set_power      = x3_wifi_power,
	.set_reset      = x3_wifi_reset,
	.set_carddetect = x3_wifi_set_carddetect,
	.get_mac_addr   = x3_wifi_get_mac_addr,
	.get_country_code = x3_wifi_get_country_code,
#if defined(CONFIG_BROADCOM_WIFI_RESERVED_MEM)
	.mem_prealloc   = x3_wifi_mem_prealloc,
#endif
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		.start	= TEGRA_GPIO_TO_IRQ(X3_WLAN_WOW),
		.end	= TEGRA_GPIO_TO_IRQ(X3_WLAN_WOW),
//for HW_OOB		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_SHAREABLE,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device x3_wifi_device = {
	.name           = "bcmdhd_wlan",
	.id             = 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev            = {
		.platform_data = &x3_wifi_control,
	},
};
//                                                       

static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

#ifdef CONFIG_MMC_EMBEDDED_SDIO
static struct embedded_sdio_data embedded_sdio_data0 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4330,	//                                                 
	},
};
#endif
//#define NVIDIA_RECOMMAND
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.mmc_data = {
		.register_status_notify	= x3_wifi_status_register,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		.embedded_sdio = &embedded_sdio_data0,
#endif
		.built_in = 0,
		.ocr_mask = MMC_OCR_1V8_MASK, //                                          
	},
#ifndef CONFIG_MMC_EMBEDDED_SDIO
	.pm_flags = MMC_PM_KEEP_POWER,
#endif
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.max_clk_limit = 45000000,
	.ddr_clk_limit = 41000000,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
	.mmc_data = {
		.built_in = 0,
	}
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
	.mmc_data = {
		.built_in = 1,
	}
};

static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int x3_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

//                                                         
static int x3_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}



void init_mac(char *buf)
{
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x00;
	buf[5] = 0x00;
}

static unsigned int util_hex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}
static int util_ascii_to_hex(char* keystr, unsigned int keystrlen, char* dst)
{
	#define TEMP_BUF_LEN 10
	char temp_buf[TEMP_BUF_LEN] = {0,};
	unsigned int hex_len = keystrlen;
	unsigned int i1 = 0, i2 = 0, num1, num2;

	if (!keystr) {
		return -1;
	}

	if (hex_len > (TEMP_BUF_LEN*2)) {
		printk(KERN_INFO "keystrlen is too long %u\n", keystrlen);
		return -1;
	}

	while (hex_len)
	{
		num1 = util_hex2num(keystr[i1++]);
		num2 = util_hex2num(keystr[i1++]);
		if (num1 < 0 || num2 < 0)
		{
			// error
			return -1;
		}
		num1 <<= 4;
		temp_buf[i2++] = num1 | num2;
		hex_len -= 2;
	}

	memcpy(dst, temp_buf, keystrlen/2);
	return keystrlen/2;
}

static bool check_nvmac_is_valid(char* nvmac)
{
	bool ret = true;
	const char init_val[3][ETHER_ADDR_LEN] = {
		{0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
		{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
		{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	};

	if (!nvmac
		|| !memcmp(nvmac, init_val[0], ETHER_ADDR_LEN)
		|| !memcmp(nvmac, init_val[1], ETHER_ADDR_LEN)
		|| !memcmp(nvmac, init_val[2], ETHER_ADDR_LEN)) {
		ret = false;
	}
	return ret;
}

static unsigned char mymac[ETHER_ADDR_LEN] = {0,};
static int x3_wifi_get_mac_addr(unsigned char *buf)
{
#if 1
	int ret = 0;
	int rdlen;

	char addr_tmp[ETHER_ADDR_LEN*2 + 1] = {0,};		// +1 for NULL
	mm_segment_t old_fs;
	struct kstat stat;
	struct file *fp;	

	init_mac(buf);
/*
	if(check_nvmac_is_valid(mymac))
	{
		memcpy( buf,mymac, ETHER_ADDR_LEN );
		return 0;	
	}
*/
	old_fs = get_fs();
	set_fs(get_ds());
	if ((ret = vfs_stat(NV_WIFI_MACADDR, &stat))) {
		set_fs(old_fs);
		printk(KERN_ERR "%s: Failed to get information (%d)\n", NV_WIFI_MACADDR, ret);
		return ret;
	}
	set_fs(old_fs);

	fp = filp_open(NV_WIFI_MACADDR, O_RDONLY, 0);
  if (IS_ERR(fp))
 	{
		printk(KERN_ERR "Failed to read file \n");	
		return ret;
 	}

	memset(mymac,0x00,ETHER_ADDR_LEN);
	
	rdlen = kernel_read(fp, fp->f_pos, addr_tmp, ETHER_ADDR_LEN*2);

	if (rdlen > 0)
	{
		if(util_ascii_to_hex(addr_tmp, ETHER_ADDR_LEN*2, mymac) <= 0)
		{
			printk(KERN_ERR "=== convert error ==== \n");		
		}

		printk(KERN_ERR "get_mac_addr = %x:%x:%x:%x:%x:%x\n",mymac[0],mymac[1],mymac[2],mymac[3],mymac[4],mymac[5]);

		if(check_nvmac_is_valid(mymac))
		{		
			memcpy( buf,mymac, ETHER_ADDR_LEN );			
		}
	}
	filp_close((struct file *)fp, NULL);
	
	return ret;

#else
	 buf[0] = 0x00;
	 buf[1] = 0x90;
	 buf[2] = 0x4c;
	 buf[3] = (unsigned char)rand_mac;
	 buf[4] = (unsigned char)(rand_mac >> 8);
	 buf[5] = (unsigned char)(rand_mac >> 16);
#endif
	return 0;
}

#define WLC_CNTRY_BUF_SZ	4		/* Country string is 3 bytes + NUL */
struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];	
	char custom_locale[WLC_CNTRY_BUF_SZ];	
	s32 custom_locale_rev;		
};
struct cntry_locales_custom cloc_ptr_temp;
static void * x3_wifi_get_country_code(char *ccode)
{
	char ccode_mapped[8] = {0x00,};
	int result;

	memset(&cloc_ptr_temp,0x00,sizeof(cloc_ptr_temp));
	result = _getWiFiMappedCode( ccode, ccode_mapped, 8 );
	printk(KERN_ERR "*** x3 country ccode=%s ccode_mapped=%s result=%d\n",ccode,ccode_mapped,result);
	
	//                              
	if( strchr( ccode_mapped, '/' ) != NULL )
	{
		char ccstr[32] = {0,};
		char* ccptr = ccstr;
		strcpy( ccstr, ccode_mapped );

		ccptr = strchr( ccstr, '/' );
		*ccptr++ = '\0';
		strcpy( cloc_ptr_temp.custom_locale, ccstr );
		cloc_ptr_temp.custom_locale_rev = (int)simple_strtol( ccptr, NULL, 0 );		
	}
	else
	{
		strcpy( cloc_ptr_temp.custom_locale, ccode_mapped );
	}

	printk(KERN_ERR "custom_locale=%s custom_locale=%d\n",cloc_ptr_temp.custom_locale,cloc_ptr_temp.custom_locale_rev);
	
	return &cloc_ptr_temp;	
}

static int x3_wifi_power(int on)
{
//	struct tegra_io_dpd *sd_dpd;

	pr_debug("%s: %d\n", __func__, on);

#if 0 /* does not use dpd for sdmmc */
	/*
	 * FIXME : we need to revisit IO DPD code
	 * on how should multiple pins under DPD get controlled
	 *
	 * enterprise GPIO WLAN enable is part of SDMMC1 pin group
	 */
	sd_dpd = tegra_io_dpd_get(&tegra_sdhci_device0.dev);
	if (sd_dpd) {
		mutex_lock(&sd_dpd->delay_lock);
		tegra_io_dpd_disable(sd_dpd);
		mutex_unlock(&sd_dpd->delay_lock);
	}
#endif

//	gpio_set_value(X3_WLAN_PWR, on);
//	mdelay(100);
	gpio_set_value(X3_WLAN_RST, on);
	mdelay(200);

#if 0 /* does not use dpd for sdmmc */
	if (sd_dpd) {
		mutex_lock(&sd_dpd->delay_lock);
		tegra_io_dpd_enable(sd_dpd);
		mutex_unlock(&sd_dpd->delay_lock);
	}
#endif

	return 0;
}

static int x3_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}
//                                                       

#ifdef CONFIG_TEGRA_PREPOWER_WIFI
static int __init enterprise_wifi_prepower(void)
{
	if (!machine_is_tegra_enterprise())
		return 0;

	enterprise_wifi_power(1);

	return 0;
}

subsys_initcall_sync(enterprise_wifi_prepower);
#endif


//                                                         
static int __init x3_wifi_init(void)
{
	int rc;
//	int irq;

//	rc = gpio_request(X3_WLAN_PWR, "wlan_power");
//	if (rc)
//		pr_err("WLAN_PWR gpio request failed:%d\n", rc);

	rc = gpio_request(X3_WLAN_RST, "wlan_rst");
	if (rc)
		pr_err("WLAN_RST gpio request failed:%d\n", rc);

	rc = gpio_request(X3_WLAN_WOW, "bcmsdh_sdmmc");
	if (rc)
		pr_err("WLAN_WOW gpio request failed:%d\n", rc);

//	tegra_gpio_enable(X3_WLAN_PWR);
	tegra_gpio_enable(X3_WLAN_RST);
	tegra_gpio_enable(X3_WLAN_WOW);

//	rc = gpio_direction_output(X3_WLAN_PWR, 0);
//	if (rc)
//		pr_err("WLAN_PWR gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_output(X3_WLAN_RST, 0);
	if (rc)
		pr_err("WLAN_RST gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(X3_WLAN_WOW);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);

//                                                           
//	tegra_gpio_enable(X3_WLAN_WOW); //nvidia recommend
//	irq = gpio_to_irq(X3_WLAN_WOW);
//	set_irq_flags(irq, IRQF_VALID); //add this line!
//	rc = enable_irq_wake(irq);
//	tegra_gpio_disable(198); //nvidia recommend
//                                                          

	platform_device_register(&x3_wifi_device);

	return 0;
}
//                                                         

int __init x3_sdhci_init(void)
{
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device0);
	//                                                        
	//                                                         
//PTEST	tegra_sdhci_platform_data0.cd_gpio = X3_WLAN_RST;
//			tegra_sdhci_platform_data0.cd_gpio_polarity = 1;
	//                                                       
	//                                                      
	tegra_gpio_enable(X3_SD_CD);
	tegra_sdhci_platform_data2.cd_gpio = X3_SD_CD;
	platform_device_register(&tegra_sdhci_device2);

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
		x3_init_wlan_mem();
#endif

	x3_wifi_init();	

	return 0;
}
