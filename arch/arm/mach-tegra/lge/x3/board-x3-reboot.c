#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/io.h>
#include <asm/mach-types.h>
#include <mach-tegra/board.h>
#include <mach-tegra/clock.h>
#include <lge/board-x3.h>
#include <mach-tegra/devices.h>
#include <mach-tegra/gpio-names.h>

#include <linux/reboot.h>

#if defined(CONFIG_MFD_MAX77663)
#include <linux/mfd/max77663-core.h>
#endif

#define _COLD_BOOT       0
#define _NORMAL_WARMBOOT  1
#define _HIDDEN_RESET    2

#ifdef CONFIG_TEGRA_WATCHDOG
#ifndef MIN_WDT_PERIOD
#define MIN_WDT_PERIOD  5
#define MAX_WDT_PERIOD  1000
#endif

struct tegra_wdt {
        struct miscdevice       miscdev;
        struct notifier_block   notifier;
        struct resource         *res_src;
        struct resource         *res_wdt;
        unsigned long           users;
        void __iomem            *wdt_source;
        void __iomem            *wdt_timer;
        int                     irq;
        int                     timeout;
        bool                    enabled;
};

extern struct platform_device *tegra_wdt_dev;
extern void tegra_wdt_enable(struct tegra_wdt *wdt);
extern void tegra_wdt_disable(struct tegra_wdt *wdt);
int  x3_wdt_kick_disabled = 0;
#endif

/*
 * warmboot commmon : wXX
 * panic tag        : wan
 * normal warmboot  : wrm
 * factor reset     : wec
 * system reboot    : wmm
 */

extern void write_cmd_reserved_buffer(unsigned char *buf, size_t len);
extern void read_cmd_reserved_buffer(unsigned char *buf, size_t len);
static int x3_panic_notify(struct notifier_block *this,
                               unsigned long event, void *ptr)
{
        unsigned char buf[3] = { 'w','a','n' };
        write_cmd_reserved_buffer(buf,3);
        printk("x3_panic_notify : buf = %s\n", (unsigned char *)buf);
        return NOTIFY_DONE;
}

static int x3_reboot_notify(struct notifier_block *nb,
                                unsigned long event, void *data)
{
        unsigned char rsbuf[3] = {0,};

        printk("x3_reboot_notify\n");
        if(data)
        {
            printk("x3_reboot_notify : data = %s\n", (unsigned char *)data);
            memcpy(rsbuf,(unsigned char*) data,3);
        }
        else
        {
             rsbuf[1] ='r';
             rsbuf[2] ='m';
        }

        rsbuf[0] ='w';
        write_cmd_reserved_buffer(rsbuf,3);

        printk("x3_reboot_notify : rsbuf = %s [%d]\n", (unsigned char *)rsbuf, event);
        switch (event) {
        case SYS_RESTART:
//                                                           
                /* USB power rail must be enabled during boot */
            if (data) {
                printk("x3_reboot_notify : cmd = %s\n", (char *)data);
                if (!strcmp((char *)data, "bootloader")
                    || !strcmp((char *)data, "oem-unlock")) {
                    printk("x3_reboot_notify : oem-unlock mode\n");
                    max77663_set_ScratchRegister(MAX77663_SCRATCH_REG_BL_UNLOCK);
                }                
            }
            return NOTIFY_OK;
//                                  
         case SYS_HALT:
         case SYS_POWER_OFF:
            return NOTIFY_OK;
        }
        return NOTIFY_DONE;
}

static struct notifier_block x3_reboot_nb = {
        .notifier_call = x3_reboot_notify,
};

struct notifier_block x3_panic_nb = {
        .notifier_call = x3_panic_notify,
};

void x3_setup_reboot(void)
{
    int rc = -1;
    printk("x3_setup_reboot\n");
    rc = register_reboot_notifier(&x3_reboot_nb);
    if (rc)
        pr_err("%s: failed to regsiter platform reboot notifier\n",
                __func__);

    atomic_notifier_chain_register(&panic_notifier_list, & x3_panic_nb);

}

static ssize_t x3_rs_reset_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

    *(int *)0 = 0;
    return count;
}

static ssize_t x3_rs_reset_read(struct device *dev, struct device_attribute *attr, const char *buf)
{
    unsigned char rsbuf[3] = { 0,};
    ssize_t ret = 0;
    int rs = -1;
    read_cmd_reserved_buffer(rsbuf,3);

    printk("x3_rs_reset_read : %s\n", (unsigned char*)rsbuf);

    if ('w' == rsbuf[0])
    {
        switch (rsbuf[1])
        {
            case 'm' : // reboot immediately
                printk("x3_rs_reset : reboot immediately detected\n");
                rs = _HIDDEN_RESET;
                rsbuf[2] = 0;
                break;
            case 'a' : // panic
                printk("x3_rs_reset : panic detected\n");
                rs = _HIDDEN_RESET;
                rsbuf[2] = 0;
                break;
            case 'e' : // recovery
                printk("x3_rs_reset : factory reset detected\n");
                rs = _NORMAL_WARMBOOT;
                rsbuf[2] = 0;
                break;
            default : // reboot other case
                printk("x3_rs_reset : warm boot detected\n");
                rs = _NORMAL_WARMBOOT;
                memset(rsbuf+1,0,2);
                break;
        }
    }
    else
    {
        printk("x3_rs_reset : cold boot detected\n");
        rs = _COLD_BOOT;
        memset(rsbuf,0,3);
    }

    ret = sprintf((char*)buf,"%d\n",rs);
    write_cmd_reserved_buffer(rsbuf,3);
    return ret;
}

#ifdef CONFIG_TEGRA_WATCHDOG


void x3_wdt_enable(const char *tag, int timeout)
{
	struct tegra_wdt *wdt = platform_get_drvdata(tegra_wdt_dev);
	static DEFINE_SPINLOCK(lock);
        unsigned char rsbuf[3] = {'w','d','t'};
        spin_lock(&lock);
	tegra_wdt_disable(wdt);
	wdt->timeout = clamp(timeout, MIN_WDT_PERIOD, MAX_WDT_PERIOD);
        printk("%s : timeout = %d\n", __func__, wdt->timeout);
        if (tag) memcpy(rsbuf,tag,3);
        write_cmd_reserved_buffer(rsbuf,3);
	tegra_wdt_enable(wdt);
	spin_unlock(&lock);

        x3_wdt_kick_disabled = 1;
}

void x3_wdt_disable(void)
{
        unsigned char rsbuf[3] = {0,};
	struct tegra_wdt *wdt = platform_get_drvdata(tegra_wdt_dev);
        printk("%s\n", __func__);
	tegra_wdt_disable(wdt);
        write_cmd_reserved_buffer(rsbuf,3);
        x3_wdt_kick_disabled = 0;
}

static ssize_t x3_rs_wdt_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u32 val =0;
    val = simple_strtoul(buf, NULL, 10);

    printk("%s : val = %d\n", __func__, val);

    if (val) {
      x3_wdt_enable("wdt",val);
    }
    else {
     x3_wdt_disable();
    }

    return (ssize_t)count;
}

static ssize_t x3_rs_wdt_read(struct device *dev, struct device_attribute *attr, const char *buf)
{
    ssize_t ret = -1;
    struct tegra_wdt *wdt = platform_get_drvdata(tegra_wdt_dev);
    ret = sprintf((char*)buf,"%d\n", wdt->timeout);
    return ret;
}

static DEVICE_ATTR(wdt, 0644, x3_rs_wdt_read , x3_rs_wdt_write);

static struct attribute *x3_rs_wdt_attr[] = {
    &dev_attr_wdt.attr,
    NULL,
};

static struct attribute_group x3_rs_wdt_attr_group = {
    .attrs = x3_rs_wdt_attr,
};

#endif

static DEVICE_ATTR(reset, 0644, x3_rs_reset_read , x3_rs_reset_write);

static struct attribute *x3_rs_reset_attr[] = {
    &dev_attr_reset.attr,
    NULL,
};

static const struct attribute_group x3_rs_reset_attr_group = {
    .attrs = x3_rs_reset_attr,
};

static int x3_rs_probe(struct platform_device *pdev)
{
    int retval = -1;
    struct device *dev = &pdev->dev;

    if (sysfs_create_group(&dev->kobj, &x3_rs_reset_attr_group)) {

        printk("[x3_rs] Failed to create sys filesystem\n");
        retval = -ENOSYS;
        goto error;
    }

#ifdef CONFIG_TEGRA_WATCHDOG
    if (sysfs_create_group(&dev->kobj, &x3_rs_wdt_attr_group)) {

        printk("[x3_rs] Failed to create sys filesystem\n");
        retval = -ENOSYS;
        goto error;
    }
#endif
    return 0;

error:
    return retval;
}

struct platform_device x3_rs_device = {
    .name = "x3_rs",
    .id = -1,
};

static struct platform_driver x3_rs_driver = {
    .probe      = x3_rs_probe,
    .driver     = {
        .name = "x3_rs",
        .owner = THIS_MODULE,
    },
};

static int __init x3_rs_init(void)
{
    int retval = 0;
    retval = platform_device_register(&x3_rs_device);
    if (retval < 0) {
        printk(KERN_ERR "platform_device_register failed!\n");
        goto out;
    }
    retval =  platform_driver_register(&x3_rs_driver);
    if (retval < 0) {
        printk(KERN_ERR "platform_driver_register failed!\n");
        goto out;
    }
out:
    return retval;
}

static void __exit x3_rs_exit(void)
{
    platform_driver_unregister(&x3_rs_driver);
}

module_init(x3_rs_init);
module_exit(x3_rs_exit);

MODULE_LICENSE("Dual BSD/GPL");

