/*
 * drivers/media/video/tegra/tegra_camera.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2010-2012, NVIDIA Corporation.  All rights reserved.
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

#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos_params.h>  //                                                                                          
#include <mach/iomap.h>
#include <mach/clk.h>
#include <mach/powergate.h>

#include <media/tegra_camera.h>

/* Eventually this should handle all clock and reset calls for the isp, vi,
 * vi_sensor, and csi modules, replacing nvrm and nvos completely for camera
 */
#define TEGRA_CAMERA_NAME "tegra_camera"

struct tegra_camera_dev {
	struct device *dev;
	struct miscdevice misc_dev;
	struct clk *isp_clk;
	struct clk *vi_clk;
	struct clk *vi_sensor_clk;
	struct clk *csus_clk;
	struct clk *csi_clk;
	struct clk *emc_clk;
	struct regulator *reg;
	struct tegra_camera_clk_info info;
	struct mutex tegra_camera_lock;
	atomic_t in_use;
	int power_on;

	bool power_save;
	bool power_save_preview;
	bool power_save_rec;
	int xres;  //                                                                                          
	int yres;  //                                                                                          
	
};

struct tegra_camera_block {
	int (*enable) (struct tegra_camera_dev *dev);
	int (*disable) (struct tegra_camera_dev *dev);
	bool is_enabled;
};

//                                                                                          
static struct tegra_camera_dev *tegra_camera_dev;

/*
 * Declare and define two static variables to provide hint to
 * gr3d module
 */
static int tegra_camera_on;
static struct tegra_camera_platform_data *pdata;

int is_tegra_camera_on(void)
{
	if (pdata) {
		if (pdata->limit_3d_emc_clk)
			return tegra_camera_on;
		else
			return 0;
	} else {
		return 0;
	}
}

static int tegra_camera_enable_clk(struct tegra_camera_dev *dev)
{
	clk_enable(dev->vi_clk);
	clk_enable(dev->vi_sensor_clk);
	clk_enable(dev->csus_clk);
	clk_enable(dev->isp_clk);
	clk_enable(dev->csi_clk);

	return 0;
}

static int tegra_camera_disable_clk(struct tegra_camera_dev *dev)
{
	clk_disable(dev->csi_clk);
	clk_disable(dev->isp_clk);
	clk_disable(dev->csus_clk);
	clk_disable(dev->vi_sensor_clk);
	clk_disable(dev->vi_clk);

	return 0;
}

static int tegra_camera_enable_emc(struct tegra_camera_dev *dev)
{
	int ret = tegra_emc_disable_eack();
	clk_enable(dev->emc_clk);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	clk_set_rate(dev->emc_clk, 300000000);                                                                       
#endif
	return ret;
}

static int tegra_camera_disable_emc(struct tegra_camera_dev *dev)
{
	clk_disable(dev->emc_clk);
	return tegra_emc_enable_eack();
}

static int tegra_camera_clk_set_rate(struct tegra_camera_dev *dev)
{
	struct clk *clk, *clk_parent;
	struct tegra_camera_clk_info *info = &dev->info;
	unsigned long parent_rate, parent_div_rate, parent_div_rate_pre;

	if (!info) {
		dev_err(dev->dev,
				"%s: no clock info %d\n",
				__func__, info->id);
		return -EINVAL;
	}

	if (info->id != TEGRA_CAMERA_MODULE_VI &&
		info->id != TEGRA_CAMERA_MODULE_EMC) {
		dev_err(dev->dev,
				"%s: set rate only aplies to vi module %d\n",
				__func__, info->id);
		return -EINVAL;
	}

	switch (info->clk_id) {
	case TEGRA_CAMERA_VI_CLK:
		clk = dev->vi_clk;
		break;
	case TEGRA_CAMERA_VI_SENSOR_CLK:
		clk = dev->vi_sensor_clk;
		break;
	case TEGRA_CAMERA_EMC_CLK:
		clk = dev->emc_clk;
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		dev_dbg(dev->dev, "%s: emc_clk rate=%lu\n",
			__func__, info->rate);
		clk_set_rate(dev->emc_clk, info->rate);
#endif
		goto set_rate_end;
	default:
		dev_err(dev->dev,
				"%s: invalid clk id for set rate %d\n",
				__func__, info->clk_id);
		return -EINVAL;
	}

	clk_parent = clk_get_parent(clk);
	parent_rate = clk_get_rate(clk_parent);
	dev_dbg(dev->dev, "%s: clk_id=%d, parent_rate=%lu, clk_rate=%lu\n",
			__func__, info->clk_id, parent_rate, info->rate);
	parent_div_rate = parent_rate;
	parent_div_rate_pre = parent_rate;

	/*
	 * The requested clock rate from user space should be respected.
	 * This loop is to search the clock rate that is higher than requested
	 * clock.
	 */
	while (parent_div_rate >= info->rate) {
		parent_div_rate_pre = parent_div_rate;
		parent_div_rate = clk_round_rate(clk, parent_div_rate-1);
	}

	dev_dbg(dev->dev, "%s: set_rate=%lu",
			__func__, parent_div_rate_pre);

	clk_set_rate(clk, parent_div_rate_pre);

	if (info->clk_id == TEGRA_CAMERA_VI_CLK) {
		/*
		 * bit 25: 0 = pd2vi_Clk, 1 = vi_sensor_clk
		 * bit 24: 0 = internal clock, 1 = external clock(pd2vi_clk)
		 */
		if (info->flag == TEGRA_CAMERA_ENABLE_PD2VI_CLK)
			tegra_clk_cfg_ex(clk, TEGRA_CLK_VI_INP_SEL, 2);

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
		{
			u32 val;
			void __iomem *apb_misc =
				IO_ADDRESS(TEGRA_APB_MISC_BASE);
			val = readl(apb_misc + 0x42c);
			writel(val | 0x1, apb_misc + 0x42c);
		}
#endif
	}

set_rate_end:
	info->rate = clk_get_rate(clk);
	dev_dbg(dev->dev, "%s: get_rate=%lu",
			__func__, info->rate);
	return 0;

}

static int tegra_camera_power_on(struct tegra_camera_dev *dev)
{
	int ret = 0;

	dev_dbg(dev->dev, "%s++\n", __func__);

	/* Enable external power */
	if (dev->reg) {
		ret = regulator_enable(dev->reg);
		if (ret) {
			dev_err(dev->dev,
				"%s: enable csi regulator failed.\n",
				__func__);
			return ret;
		}
	}
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* Unpowergate VE */
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_VENC);
	if (ret)
		dev_err(dev->dev,
			"%s: unpowergate failed.\n",
			__func__);
#endif
	dev->power_on = 1;
	tegra_camera_on = dev->power_on;
	return ret;
}

static int tegra_camera_power_off(struct tegra_camera_dev *dev)
{
	int ret = 0;

	dev_dbg(dev->dev, "%s++\n", __func__);

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	/* Powergate VE */
	ret = tegra_powergate_partition(TEGRA_POWERGATE_VENC);
	if (ret)
		dev_err(dev->dev,
			"%s: powergate failed.\n",
			__func__);
#endif
	/* Disable external power */
	if (dev->reg) {
		ret = regulator_disable(dev->reg);
		if (ret) {
			dev_err(dev->dev,
				"%s: disable csi regulator failed.\n",
				__func__);
			return ret;
		}
	}
	dev->power_on = 0;
	tegra_camera_on = dev->power_on;
	return ret;
}

static long tegra_camera_ioctl(struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	uint id;
	struct tegra_camera_dev *dev = file->private_data;

	/* first element of arg must be u32 with id of module to talk to */
	if (copy_from_user(&id, (const void __user *)arg, sizeof(uint))) {
		dev_err(dev->dev,
				"%s: Failed to copy arg from user", __func__);
		return -EFAULT;
	}

	if (id >= TEGRA_CAMERA_MODULE_MAX) {
		dev_err(dev->dev,
				"%s: Invalid id to tegra isp ioctl%d\n",
				__func__, id);
		return -EINVAL;
	}

	switch (cmd) {
	/*
	 * Clock enable/disable and reset should be handled in kernel.
	 * In order to support legacy code in user space, we don't remove
	 * these IOCTL.
	 */
	case TEGRA_CAMERA_IOCTL_ENABLE:
	case TEGRA_CAMERA_IOCTL_DISABLE:
	case TEGRA_CAMERA_IOCTL_RESET:
		return 0;
	case TEGRA_CAMERA_IOCTL_CLK_SET_RATE:
	{
		int ret;

		if (copy_from_user(&dev->info, (const void __user *)arg,
				   sizeof(struct tegra_camera_clk_info))) {
			dev_err(dev->dev,
				"%s: Failed to copy arg from user\n", __func__);
			return -EFAULT;
		}
		ret = tegra_camera_clk_set_rate(dev);
		if (ret)
			return ret;
		if (copy_to_user((void __user *)arg, &dev->info,
				 sizeof(struct tegra_camera_clk_info))) {
			dev_err(dev->dev,
				"%s: Failed to copy arg to user\n", __func__);
			return -EFAULT;
		}
		return 0;
	}
	default:
		dev_err(dev->dev,
				"%s: Unknown tegra_camera ioctl.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int tegra_camera_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct tegra_camera_dev *dev = container_of(miscdev,
						struct tegra_camera_dev,
						misc_dev);
	int ret = 0;

	dev_info(dev->dev, "%s\n", __func__);

	if (atomic_xchg(&dev->in_use, 1))
		return -EBUSY;

	file->private_data = dev;

	mutex_lock(&dev->tegra_camera_lock);
	/* turn on CSI regulator */
	ret = tegra_camera_power_on(dev);
	if (ret)
		goto open_exit;
	/* set EMC request */
	ret = tegra_camera_enable_emc(dev);
	if (ret)
		goto open_exit;
	/* enable camera HW clock */
	ret = tegra_camera_enable_clk(dev);
	if (ret)
		goto open_exit;
open_exit:
	mutex_unlock(&dev->tegra_camera_lock);
	return ret;
}

static int tegra_camera_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct tegra_camera_dev *dev = file->private_data;

	dev_info(dev->dev, "%s\n", __func__);


	mutex_lock(&dev->tegra_camera_lock);
	/* disable HW clock */
	ret = tegra_camera_disable_clk(dev);
	if (ret)
		goto release_exit;
	/* nullify EMC request */
	ret = tegra_camera_disable_emc(dev);
	if (ret)
		goto release_exit;
	/* turn off CSI regulator */
	ret = tegra_camera_power_off(dev);
	if (ret)
		goto release_exit;

release_exit:
	mutex_unlock(&dev->tegra_camera_lock);
	WARN_ON(!atomic_xchg(&dev->in_use, 0));
	return 0;
}

static const struct file_operations tegra_camera_fops = {
	.owner = THIS_MODULE,
	.open = tegra_camera_open,
	.unlocked_ioctl = tegra_camera_ioctl,
	.release = tegra_camera_release,
};

static int tegra_camera_clk_get(struct platform_device *pdev, const char *name,
				struct clk **clk)
{
	*clk = clk_get(&pdev->dev, name);
	if (IS_ERR_OR_NULL(*clk)) {
		dev_err(&pdev->dev, "%s: unable to get clock for %s\n",
			__func__, name);
		*clk = NULL;
		return PTR_ERR(*clk);
	}
	return 0;
}

//                                                                                          
//#define POWER_SAVE_REC_CPU_USER_CAP_RATE	640000
#define POWER_SAVE_BOOST_STEP			1
#define POWER_SAVE_CPU_FREQ_MIN			640000
#define POWER_SAVE_CPU_FREQ_MAX			640000
#define POWER_SAVE_MIN_CPUS			2
#define POWER_SAVE_MAX_CPUS			2
//                                                                                          
static unsigned long boost_step_default;

static inline void tegra_camera_do_power_save(struct tegra_camera_dev *dev)
{
	int preview, rec;
	
    pr_info("%s \n", __func__);

	preview = dev->power_save_preview;
	rec = dev->power_save_rec;

	if (!dev->power_save && (preview || rec)) {
		boost_step_default = cpufreq_interactive_get_boost_step();
		dev->power_save = true;
	}

	if (!dev->power_save)
		return;

	if (preview && rec) {    
    pr_info("%s : when preview && rec \n", __func__);
		cpufreq_interactive_set_boost_step(POWER_SAVE_BOOST_STEP);
//                                                                                          
		//                                                                      
		cpufreq_set_max_freq(NULL, POWER_SAVE_CPU_FREQ_MAX);
		if ((dev->xres == 1280 && dev->yres == 720) ||
			(dev->xres == 1440 && dev->yres == 1080) ||
				(dev->xres == 1920 && dev->yres == 1080)) {
			cpufreq_set_min_freq(NULL, POWER_SAVE_CPU_FREQ_MIN);
			tegra_auto_hotplug_set_min_cpus(POWER_SAVE_MIN_CPUS);
			tegra_auto_hotplug_set_max_cpus(POWER_SAVE_MAX_CPUS);
		}
//                                                                                          
	} else if (preview && !rec) {
	  pr_info("%s : preview && !rec \n", __func__);
		cpufreq_interactive_set_boost_step(POWER_SAVE_BOOST_STEP);
//                                                                                          
		#if 0 //kwanghee.choi 20120912 Vu1.0 Global fix the frame drop on Camera by setting full CPU clock(start)
		cpufreq_set_min_freq(NULL, POWER_SAVE_CPU_FREQ_MIN);
		cpufreq_set_max_freq(NULL, PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
		tegra_auto_hotplug_set_min_cpus(0);
		tegra_auto_hotplug_set_max_cpus(0);
		#else
		cpufreq_set_min_freq(NULL, PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
		cpufreq_set_max_freq(NULL, PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
		tegra_auto_hotplug_set_min_cpus(0);
		tegra_auto_hotplug_set_max_cpus(0);
		#endif //kwanghee.choi 20120912 Vu1.0 Global fix the frame drop on Camera by setting full CPU clock(end)
//                                                                                          
	} else if (!preview && !rec) {
	  pr_info("%s : !preview && !rec \n", __func__);
		cpufreq_interactive_set_boost_step(boost_step_default);
//                                                                                          
		cpufreq_set_min_freq(NULL, PM_QOS_CPU_FREQ_MIN_DEFAULT_VALUE);
		cpufreq_set_max_freq(NULL, PM_QOS_CPU_FREQ_MAX_DEFAULT_VALUE);
		tegra_auto_hotplug_set_min_cpus(0);
		tegra_auto_hotplug_set_max_cpus(0);
//                                                                                          
		dev->power_save = false;
	}
}

//                                                                                          
int tegra_camera_set_size(int xres, int yres)
{
	struct tegra_camera_dev *camera_dev = tegra_camera_dev;
	if ((xres <= 0) || (yres <= 0))
		return -EINVAL;

	mutex_lock(&camera_dev->tegra_camera_lock);
	camera_dev->xres = xres;
	camera_dev->yres = yres;
	tegra_camera_do_power_save(camera_dev);
	mutex_unlock(&camera_dev->tegra_camera_lock);

	return 0;
}
//                                                                                          

static ssize_t tegra_camera_show_power_save_preview(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tegra_camera_dev *camera_dev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", camera_dev->power_save_preview);
}

static ssize_t tegra_camera_store_power_save_preview(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct tegra_camera_dev *camera_dev = dev_get_drvdata(dev);
	int val, ret;

	ret = sscanf(buf, "%d", &val);
	if (!ret)
		return ret;

	mutex_lock(&camera_dev->tegra_camera_lock);
	if (val == 1 && !camera_dev->power_save_preview)
		camera_dev->power_save_preview = true;
	else if (val == 0 && camera_dev->power_save_preview)
		camera_dev->power_save_preview = false;

	tegra_camera_do_power_save(camera_dev);
	mutex_unlock(&camera_dev->tegra_camera_lock);

	return count;
}

static DEVICE_ATTR(power_save_preview, S_IRUGO | S_IWUSR | S_IRGRP | S_IWGRP,
		   tegra_camera_show_power_save_preview,
		   tegra_camera_store_power_save_preview);

static ssize_t tegra_camera_show_power_save_rec(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tegra_camera_dev *camera_dev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", camera_dev->power_save_rec);
}

static ssize_t tegra_camera_store_power_save_rec(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct tegra_camera_dev *camera_dev = dev_get_drvdata(dev);
	int val, ret;

	ret = sscanf(buf, "%d", &val);
	if (!ret)
		return ret;

	mutex_lock(&camera_dev->tegra_camera_lock);
	if (val == 1 && !camera_dev->power_save_rec)
		camera_dev->power_save_rec = true;
	else if (val == 0 && camera_dev->power_save_rec)
		camera_dev->power_save_rec = false;

	tegra_camera_do_power_save(camera_dev);
	mutex_unlock(&camera_dev->tegra_camera_lock);

	return count;
}

static DEVICE_ATTR(power_save_rec, S_IRUGO | S_IWUSR | S_IRGRP | S_IWGRP,
		   tegra_camera_show_power_save_rec,
		   tegra_camera_store_power_save_rec);


bool tegra_camera_get_power_save_rec(void)
{
	if(tegra_camera_dev != NULL)
	{
		return tegra_camera_dev->power_save_rec;
	}
	return false;
}

EXPORT_SYMBOL(tegra_camera_get_power_save_rec);

static int tegra_camera_probe(struct platform_device *pdev)
{
	int err;
	struct tegra_camera_dev *dev;

	dev_info(&pdev->dev, "%s\n", __func__);
	dev = devm_kzalloc(&pdev->dev, sizeof(struct tegra_camera_dev),
			GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "%s: unable to allocate memory\n",
			__func__);
		goto alloc_err;
	}
	
	tegra_camera_dev = dev;  //                                                                                          

	mutex_init(&dev->tegra_camera_lock);

	/* Powergate VE when boot */
	mutex_lock(&dev->tegra_camera_lock);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra_powergate_partition(TEGRA_POWERGATE_VENC);
	if (err)
		dev_err(&pdev->dev, "%s: powergate failed.\n", __func__);
#endif
	dev->power_save = false;
	mutex_unlock(&dev->tegra_camera_lock);

	dev->dev = &pdev->dev;
	pdata = pdev->dev.platform_data;

	/* Get regulator pointer */
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	dev->reg = regulator_get(&pdev->dev, "vcsi");
#else
	dev->reg = regulator_get(&pdev->dev, "avdd_dsi_csi");
#endif
	if (IS_ERR_OR_NULL(dev->reg)) {
		if (dev->reg == ERR_PTR(-ENODEV)) {
			dev->reg = NULL;
			dev_info(&pdev->dev, "%s: no regulator device, overriding\n",
							__func__);
		} else {
			dev_err(&pdev->dev, "%s: couldn't get regulator\n",
							__func__);
			return PTR_ERR(dev->reg);
		}
	}

	dev->misc_dev.minor = MISC_DYNAMIC_MINOR;
	dev->misc_dev.name = TEGRA_CAMERA_NAME;
	dev->misc_dev.fops = &tegra_camera_fops;
	dev->misc_dev.parent = &pdev->dev;

	err = misc_register(&dev->misc_dev);
	if (err) {
		dev_err(&pdev->dev, "%s: Unable to register misc device!\n",
		       TEGRA_CAMERA_NAME);
		goto misc_register_err;
	}

	err = tegra_camera_clk_get(pdev, "isp", &dev->isp_clk);
	if (err)
		goto misc_register_err;
	err = tegra_camera_clk_get(pdev, "vi", &dev->vi_clk);
	if (err)
		goto vi_clk_get_err;
	err = tegra_camera_clk_get(pdev, "vi_sensor", &dev->vi_sensor_clk);
	if (err)
		goto vi_sensor_clk_get_err;
	err = tegra_camera_clk_get(pdev, "csus", &dev->csus_clk);
	if (err)
		goto csus_clk_get_err;
	err = tegra_camera_clk_get(pdev, "csi", &dev->csi_clk);
	if (err)
		goto csi_clk_get_err;
	err = tegra_camera_clk_get(pdev, "emc", &dev->emc_clk);
	if (err)
		goto emc_clk_get_err;

	/* dev is set in order to restore in _remove */
	platform_set_drvdata(pdev, dev);

	err = device_create_file(dev->dev, &dev_attr_power_save_preview);
	if (err < 0)
		dev_warn(dev->dev,
			 "%s: failed to add power_save_preview sysfs: %d\n",
			 __func__, err);

	err = device_create_file(dev->dev, &dev_attr_power_save_rec);
	if (err < 0)
		dev_warn(dev->dev,
			 "%s: failed to add power_save_rec sysfs: %d\n",
			 __func__, err);

	return 0;

emc_clk_get_err:
	clk_put(dev->emc_clk);
csi_clk_get_err:
	clk_put(dev->csus_clk);
csus_clk_get_err:
	clk_put(dev->vi_sensor_clk);
vi_sensor_clk_get_err:
	clk_put(dev->vi_clk);
vi_clk_get_err:
	clk_put(dev->isp_clk);
misc_register_err:
	regulator_put(dev->reg);
alloc_err:
	return err;
}

static int tegra_camera_remove(struct platform_device *pdev)
{
	struct tegra_camera_dev *dev = platform_get_drvdata(pdev);

	clk_put(dev->isp_clk);
	clk_put(dev->vi_clk);
	clk_put(dev->vi_sensor_clk);
	clk_put(dev->csus_clk);
	clk_put(dev->csi_clk);

	misc_deregister(&dev->misc_dev);
	regulator_put(dev->reg);
	mutex_destroy(&dev->tegra_camera_lock);

	return 0;
}

static int tegra_camera_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tegra_camera_dev *dev = platform_get_drvdata(pdev);
	int ret = 0;

	mutex_lock(&dev->tegra_camera_lock);
	if (dev->power_on) {
		ret = -EBUSY;
		dev_err(&pdev->dev,
		"tegra_camera cannot suspend, "
		"application is holding on to camera. \n");
	}
	mutex_unlock(&dev->tegra_camera_lock);

	return ret;
}

static int tegra_camera_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver tegra_camera_driver = {
	.probe = tegra_camera_probe,
	.remove = tegra_camera_remove,
	.suspend = tegra_camera_suspend,
	.resume = tegra_camera_resume,
	.driver = { .name = TEGRA_CAMERA_NAME }
};

static int __init tegra_camera_init(void)
{
	return platform_driver_register(&tegra_camera_driver);
}

static void __exit tegra_camera_exit(void)
{
	platform_driver_unregister(&tegra_camera_driver);
}

module_init(tegra_camera_init);
module_exit(tegra_camera_exit);

