/*
 *  MAX17043_fuelgauge.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 LG Electronics
 *  Dajin Kim <dajin.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/power/max17043_fuelgauge.h>
#include <mach/gpio.h>
#include <linux/slab.h>
#include "../../arch/arm/mach-tegra/gpio-names.h"
#include "../../arch/arm/mach-tegra/lge/x3/include/lge/board-x3.h"

#define GAUGE_INT		(TEGRA_GPIO_PS0)
#define RCOMP_BL44JN		(0x44)	/* Default Value for LGP970 Battery */
#define RCOMP_BLT3		(0xD8)
#define MONITOR_LEVEL		(2)
#define MAX17043_VCELL_REG	0x02
#define MAX17043_SOC_REG	0x04
#define MAX17043_MODE_REG	0x06
#define MAX17043_VER_REG	0x08
#define MAX17043_CONFIG_REG	0x0C
#define MAX17043_CMD_REG	0xFE

#define MAX17043_VCELL_MSB	0x02
#define MAX17043_VCELL_LSB	0x03
#define MAX17043_SOC_MSB	0x04
#define MAX17043_SOC_LSB	0x05
#define MAX17043_MODE_MSB	0x06
#define MAX17043_MODE_LSB	0x07
#define MAX17043_VER_MSB	0x08
#define MAX17043_VER_LSB	0x09
#define MAX17043_CONFIG_MSB	0x0C
#define MAX17043_CONFIG_LSB	0x0D
#define MAX17043_CMD_MSB	0xFE
#define MAX17043_CMD_LSB	0xFF

#define MAX17043_RCOMP_MSB	0x0C
#define MAX17043_RCOMP_LSB	0x0D

#define MAX17043_OCV_MSB	0x0E
#define MAX17043_OCV_LSB	0x0F

#define MAX17043_WORK_DELAY	(10 * HZ)	// 10s
#define MAX17043_BATTERY_FULL	95		// Tuning Value
#define MAX17043_TOLERANCE	20		// Tuning Value
#define	BATT_VCELL_LOW		3400	// Low Battery Cut off soc

#define __DEBUG_FUEL

#ifdef __DEBUG_FUEL
#define DFUEL(fmt, args...) printk("[FUEL] " fmt, ##args)
#else
#define DFUEL(fmt, args...)
#endif
struct max17043_chip {
	struct i2c_client		*client;
	struct power_supply		battery;
	struct max17043_platform_data	*pdata;
	struct delayed_work		work;
	struct work_struct		alert_work;

	/* Max17043 Registers.(Raw Data) */
	int vcell;			// VCELL Register vaule
	int soc;			// SOC Register value
	int version;			// Max17043 Chip version
	int config;			// RCOMP, Sleep, ALRT, ATHD

	/* Interface with Android */
	int voltage;			// Battery Voltage   (Calculated from vcell)
	int capacity;			// Battery Capacity  (Calculated from soc)
	max17043_status status;	// State Of max17043
};

/*
 * Voltage Calibration Data
 *   voltage = (capacity * gradient) + intercept
 *   voltage must be in +-15%
 */
struct max17043_calibration_data {
	int voltage;	/* voltage in mA */
	int capacity;	/* capacity in % */
	int gradient;	/* gradient * 1000 */
	int intercept;	/* intercept * 1000 */
};
// 240mA Load for Battery
static struct max17043_calibration_data without_charger[] = {
    {4030,  77, 10, 3263},
    {3866,  58, 9,  3349},
    {3800,  47, 6,  3530},
    {3695,  23, 4,  3592},
    {3651,  18, 8,  3505},
    {3613,   9, 4,  3576},
    {3400,   0, 24, 3404},
    {   -1, -1, -1, -1},  // End of Data
};

// 770mA Charging Battery
static struct max17043_calibration_data with_charger[] = {
	{3865,		2,		66,		3709},
	{3956,		19,		5,		3851},
	{4021,		46,		2,		3912},
	{4088,		61,		5,		3813},
	{4158,		71,		7,		3689},
	{4200,		100,	2,		4042},
	{ -1, -1, -1, -1},	// End of Data
};

static struct max17043_chip* reference = NULL;
int need_to_quickstart = 0;
EXPORT_SYMBOL(need_to_quickstart);

static int max17043_write_data(struct i2c_client *client, int reg, const u8 *values, int length)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, length, values);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17043_read_data(struct i2c_client *client, int reg, u8 *values, int length)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, length, values);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17043_write_reg(struct i2c_client *client, int reg, u16 value)
{
	int ret = 0;

	value = ((value & 0xFF00) >> 8) | ((value & 0xFF) << 8);

	ret = i2c_smbus_write_word_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
static int max17043_read_reg(struct i2c_client *client, int reg)
{
	int ret = 0;

	ret = i2c_smbus_read_word_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ((ret & 0xFF00) >> 8) | ((ret & 0xFF) << 8);
}
static int max17043_reset(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);

	max17043_write_reg(client, MAX17043_CMD_REG, 0x5400);

	chip->status = MAX17043_RESET;
	dev_info(&client->dev, "MAX17043 Fuel-Gauge Reset\n");
	return 0;
}

static int max17043_quickstart(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);

	max17043_write_reg(client, MAX17043_MODE_REG, 0x4000);

	chip->status = MAX17043_QUICKSTART;
	dev_info(&client->dev, "MAX17043 Fuel-Gauge Quick-Start in kernel\n");
	return 0;
}
static int max17043_read_vcell(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	u16 value;

	value = max17043_read_reg(client, MAX17043_VCELL_REG);

	if(value < 0) return value;

	chip->vcell = value >> 4;
	dev_dbg(&client->dev, "MAX17043 vcell [%d]\n",chip->vcell);
	return 0;
}
static int max17043_read_soc(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	u16 value;

	value = max17043_read_reg(client, MAX17043_SOC_REG);

	if(value < 0) return value;

	chip->soc = value;
	dev_dbg(&client->dev, "MAX17043 soc [%d]\n",chip->soc);

	return 0;
}
static int max17043_read_version(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	u16 value;

	value = max17043_read_reg(client, MAX17043_VER_REG);

	chip->version = value;

	dev_info(&client->dev, "MAX17043 Fuel-Gauge Ver %d\n", value);

	return 0;
}
static int max17043_read_config(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	u16 value;

	value = max17043_read_reg(client, MAX17043_CONFIG_REG);

	if(value < 0) return value;

	chip->config = value;

	return 0;
}
static int max17043_write_config(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);

	max17043_write_reg(client, MAX17043_CONFIG_REG, chip->config);

	return 0;
}

static int max17043_need_quickstart(struct max17043_chip *chip, int charging)
{
	struct max17043_calibration_data* data;
	int i = 0;
	int expected;
	int diff;
	int vol;
	int level;

	// Get Current Data
	vol = chip->voltage;
	level = chip->soc >> 8;
	if(level > 100)
		level = 100;
	else if(level < 0)
		level = 0;

	// choose data to use
	if(charging) {
		data = with_charger;
		while(data[i].voltage != -1) {
			if(vol <= data[i].voltage)
				break;
			i++;
		}
	} else {
		data = without_charger;
		while(data[i].voltage != -1) {
			if(vol >= data[i].voltage)
				break;
			i++;
		}
	}

	// absense of data
	if(data[i].voltage == -1) {
		if(charging) {
			if(level == 100)
				return 0;
			else
				return 1;
		}
		else {
			if(level == 0)
				return 0;
			else
				return 1;
		}
	}

	// calculate diff
	expected = (vol - data[i].intercept) / data[i].gradient;
	if(expected > 100)
		expected = 100;
	else if(expected < 0)
		expected = 0;
	diff = expected - level;

	// judge
	if(diff < -MAX17043_TOLERANCE || diff > MAX17043_TOLERANCE) {
		//printk(KERN_DEBUG "[BATTERY] real : %d%% , expected : %d%%\n", level, expected);
		need_to_quickstart += 1;
	} else {
		need_to_quickstart = 0;
	}

	// Maximum continuous reset time is 2. If reset over 2 times, discard it.
	if(need_to_quickstart > 2)
		need_to_quickstart = 0;

	return need_to_quickstart;
}

static int max17043_next_alert_level(int level)
{
	int next_level;
	printk(KERN_DEBUG " [MAX17043] SOC_level = %d\n",level);


	if(level >= 15){
		next_level = 30; //15
	}else if(level <15 && level >=5){
		next_level = 10; //5
	}else if(level <5 && level >=0){
		next_level = 1;  //0
	}else{

	}

	printk(KERN_DEBUG " [MAX17043] Next_alert_level = %d\n",next_level/2);
	return next_level;
}

static int max17043_set_rcomp(int rcomp)
{
	if(reference == NULL)
		return -1;

	rcomp &= 0xff;
	reference->config = ((reference->config & 0x00ff) | (rcomp << 8));

	max17043_write_config(reference->client);

	return 0;
}
static int max17043_set_athd(int level)
{
	if(reference == NULL)
		return -1;

	if(level > 32)
		level = 32;
	else if(level < 1)
		level = 1;

	level = 32 - level;
	if(level == (reference->config & 0x1F))
		return level;

	reference->config = ((reference->config & 0xffe0) | level);
	max17043_write_config(reference->client);

	return level;
}
static int max17043_clear_interrupt(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);

	if(chip->config & 0x20) {
		chip->config &= 0xffdf;
		max17043_write_config(chip->client);
	}

	return 0;
}

static int max17043_get_soc(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	u8 buf[5];
	long soc, t_soc;
	int ret;

	ret = max17043_read_data(client, MAX17043_SOC_MSB, &buf[0], 2);


#if defined(CONFIG_MACH_VU10)
	soc = ((buf[0] * 256) + buf[1]);// * 19531; /* 0.001953125 */
	soc >>= 8;
#else

// taekyeong.kim 20120511 maxim recommend
	//t_soc = ((buf[0] * 256) + buf[2]) * 19531;
//                                                                     
	t_soc = ((buf[0] * 256) + buf[1]) * 19531;

	//soc = ((t_soc - 13 * 1000000) / (950 - 13)) / 10000;
	soc = (t_soc - 13 *1000000) / ((970-13)* 10000);
	// empty(1.3)  & full(95) ¨¬¢¬A¢´
// taekyeong.kim 20120511 maxim recommend

#endif
	chip->soc = (int)soc;

	if (chip->soc >= 100) {
		chip->soc = 100;
	}


	dev_dbg(&client->dev, "Get soc [%d]\n", chip->soc);

	return ret;
}

static int max17043_update(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	int ret;

	ret = max17043_read_vcell(client);
	if(ret < 0) return ret;

	ret = max17043_get_soc(client);
	//ret = max17043_read_soc(client);
	if(ret < 0) return ret;

	/* convert raw data to usable data */
	chip->voltage = (chip->vcell * 5) >> 2;	// vcell * 1.25 mV
	chip->voltage = chip->voltage * 1000;
	chip->capacity = chip->soc;

#if 0	// taekyeong.kim 20120511 maxim recommend
	chip->capacity = ((chip->capacity - (2)) * 100) / (MAX17043_BATTERY_FULL - 2);	// 2, 2 Adjust 100% Condition
#endif	// taekyeong.kim 20120511 maxim¢¯¢®¨ù¡© recommend

//                                           
//TODO: Need to update
#if defined(CONFIG_MACH_VU10)
	chip->capacity = ((chip->capacity) * 100) / MAX17043_BATTERY_FULL;
#endif
//                                           

	if (chip->capacity >= 100)  {
		chip->capacity = 100;
	}
	else if (chip->capacity < 0) {
		chip->capacity = 0;
	}

	chip->status = MAX17043_WORKING;

// taekyeong.kim 20120421
//	DFUEL("max17043_update Vol = %d, Rsoc = %d, Cap =%d\n", (chip->voltage/1000), chip->soc, chip->capacity );
// taekyeong.kim 20120421

	return 0;
}
static void max17043_work(struct work_struct *work)
{
	struct max17043_chip *chip;
	int charging = 0;

	chip = container_of(work, struct max17043_chip, work.work);

	switch(chip->status) {
		case MAX17043_RESET://1
			max17043_read_version(chip->client);
			max17043_read_config(chip->client);
#if defined(CONFIG_MACH_VU10)
			max17043_set_rcomp(RCOMP_BLT3);
#else
			max17043_set_rcomp(RCOMP_BL44JN);
#endif
		case MAX17043_QUICKSTART://2
			max17043_update(chip->client);
			if(max17043_need_quickstart(chip, charging)) {
				max17043_reset(chip->client);
				schedule_delayed_work(&chip->work,  HZ);
				return;
			}
			max17043_set_athd(MONITOR_LEVEL);
			max17043_clear_interrupt(chip->client);
			need_to_quickstart = 0;
			break;
		case MAX17043_WORKING://3
		default:
			//max17043_update(chip->client);
			break;
	}
	schedule_delayed_work(&chip->work, MAX17043_WORK_DELAY);
	// taekyeong.kim	20120418
	DFUEL("max17043_work chip->status = %d\n", chip->status);
}

static void max17043_alert_work(struct work_struct *work)
{
	printk(KERN_INFO "[Max17043_Alert_Work_Start]\n");
	struct max17043_chip *chip =
		container_of(work, struct max17043_chip, alert_work);;

	if (chip->status == MAX17043_WORKING) {
		cancel_delayed_work_sync(&chip->work);
		schedule_delayed_work(&chip->work, MAX17043_WORK_DELAY);
	}
	max17043_update(chip->client);

	max17043_read_config(chip->client);
	max17043_clear_interrupt(chip->client);

#if defined(CONFIG_HUB_MUIC)
	if(get_muic_mode() == MUIC_NONE) {
		printk(KERN_INFO "[Battery] Low Level Alert (%d)\n",chip->capacity);
	}
#endif
}

static irqreturn_t max17043_interrupt_handler(int irq, void *data)
{
	struct max17043_chip *chip = data;

	schedule_work(&chip->alert_work);
	return IRQ_HANDLED;
}

#if 1	// B-Project Does not use fuel gauge as a battery driver
/* sysfs(power_supply) interface : for Android Battery Service [START] */
static enum power_supply_property max17043_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int max17043_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17043_chip *chip = container_of(psy,
				struct max17043_chip, battery);

	switch (psp) {
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			max17043_update(chip->client); //nedd to update only 1 time.
			val->intval = chip->voltage;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = chip->capacity;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}
/* sysfs interface : for Android Battery Service [END] */
#endif

/* sysfs interface : for AT Commands [START] */
ssize_t max17043_show_soc(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct max17043_chip *chip = dev_get_drvdata(dev);
	int level;

	level = ((chip->soc) >> 8);
	if(level > 100)
		level = 100;
	else if(level < 0)
		level = 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", level);
}
DEVICE_ATTR(soc, 0444, max17043_show_soc, NULL);

ssize_t max17043_show_status(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct max17043_chip *chip = dev_get_drvdata(dev);

	switch(chip->status) {
		case MAX17043_RESET:
			return snprintf(buf, PAGE_SIZE, "reset\n");
		case MAX17043_QUICKSTART:
			return snprintf(buf, PAGE_SIZE, "quickstart\n");
		case MAX17043_WORKING:
			return snprintf(buf, PAGE_SIZE, "working\n");
		default:
			return snprintf(buf, PAGE_SIZE, "ERROR\n");
	}
}

ssize_t max17043_store_status(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	struct max17043_chip *chip = dev_get_drvdata(dev);

	if(strncmp(buf,"reset",5) == 0) {
		cancel_delayed_work(&chip->work);
		//max17043_reset(chip->client);
		schedule_delayed_work(&chip->work, HZ);
	} else if(strncmp(buf,"quickstart",10) == 0) {
		cancel_delayed_work(&chip->work);
		//max17043_quickstart(chip->client);
		schedule_delayed_work(&chip->work, HZ);
	} else if(strncmp(buf,"working",7) == 0) {
		// do nothing
	} else {
		return -1;
	}
	return count;
}
DEVICE_ATTR(state, 0660, max17043_show_status, max17043_store_status);
/* sysfs interface : for AT Commands [END] */

/*                                                                      */
#if defined(CONFIG_MACH_VU10)
ssize_t max17043_store_por(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	u8 org_rcomp_msb, org_rcomp_lsb;
	u8 org_ocv_msb, org_ocv_lsb;
	u8 values[4];

	struct max17043_chip *chip = dev_get_drvdata(dev);

	if(strncmp(buf,"por",3) == 0) {
		cancel_delayed_work(&chip->work);
		printk("FG POR START...\n");
		/* 1 : Unlock Model Access */
		values[0] = 0x4A; values[1] = 0x57;
		max17043_write_data(reference->client, 0x3E, &values[0], 2);

		/* 2 : Read Original RCOMP and OCV Register */
		max17043_read_data(reference->client, MAX17043_RCOMP_MSB, &values[0], 4);
		org_rcomp_msb = values[0];	org_rcomp_lsb = values[1];
		org_ocv_msb = values[2];	org_ocv_lsb = values[3];

		/* 3 : Lock Model Access */
		values[0] = 0x00; values[1] = 0x00;
		max17043_write_data(reference->client, 0x3E, &values[0], 2);

		/* 4 : Write POR Command */
		values[0] = 0x54; values[1] = 0x00;
		max17043_write_data(reference->client, 0xFE, &values[0], 2);

		/* 5 : Delay 300ms */
		msleep(300);

		/* 6 : Unlock Model Access */
		values[0] = 0x4A; values[1] = 0x57;
		max17043_write_data(reference->client, 0x3E, &values[0], 2);

		/* 7 : Restore RCOMP and OCV */
		values[0] = org_rcomp_msb;	values[1] = org_rcomp_lsb;
		values[2] = org_ocv_msb;	values[3] = org_ocv_lsb;
		max17043_write_data(reference->client, MAX17043_RCOMP_MSB, &values[0], 4);

		/* 8 : Lock Model Access */
		values[0] = 0x00; values[1] = 0x00;
		max17043_write_data(reference->client, 0x3E, &values[0], 2);

		/* 9 : Delay 200ms */
		msleep(200);
		printk("FG POR END...\n");
		schedule_delayed_work(&chip->work, HZ);
	} else {
		printk("Invalid POR command...\n");
		return -1;
	}
	return count;
}
DEVICE_ATTR(por, 0660, NULL, max17043_store_por);
#endif
/*                                                                      */

/* SYMBOLS to use outside of this module */
int max17043_get_capacity(void)
{
	if(reference == NULL)	// if fuel gauge is not initialized,
		return 100;			// return Dummy Value
	return reference->capacity;
}
EXPORT_SYMBOL(max17043_get_capacity);
int max17043_get_voltage(void)
{
	if(reference == NULL)	// if fuel gauge is not initialized,
		return 4200;		// return Dummy Value
	return reference->voltage;
}
EXPORT_SYMBOL(max17043_get_voltage);
int max17043_do_calibrate(void)
{
	if(reference == NULL)
		return -1;

	cancel_delayed_work(&reference->work);
	max17043_quickstart(reference->client);
	schedule_delayed_work(&reference->work, HZ);

	return 0;
}
EXPORT_SYMBOL(max17043_do_calibrate);
int max17043_set_rcomp_by_temperature(int temp)
{
	int rcomp;
	if(reference == NULL)
		return -1;	// MAX17043 not initialized
#if defined(CONFIG_MACH_VU10)
	rcomp = RCOMP_BLT3;
#else
	rcomp = RCOMP_BL44JN;
#endif

	// RCOMP compensation refer to temperature
	if(temp < 200)
		rcomp = rcomp + ((200 - temp) / 2);
	else if(temp > 200)
		rcomp = rcomp - ((temp - 200) * 11 / 50);

	if(rcomp < 0x00)
		rcomp = 0x00;
	else if(rcomp > 0xff)
		rcomp = 0xff;

	max17043_set_rcomp(rcomp);
	return 0;
}
EXPORT_SYMBOL(max17043_set_rcomp_by_temperature);
int max17043_set_alert_level(int alert_level)
{
	return max17043_set_athd(alert_level);
}
EXPORT_SYMBOL(max17043_set_alert_level);
/* End SYMBOLS */

static void max17043_init_model(struct i2c_client *client)
{
	u8 org_rcomp_msb, org_rcomp_lsb;
	u8 org_ocv_msb, org_ocv_lsb;
	u8 reg;
	u8 values[32];
	int len;

	/* 1 : Unlock Model Access */
	values[0] = 0x4A; values[1] = 0x57;
	max17043_write_data(client, 0x3E, &values[0], 2);

	/* 2 : Read Original RCOMP and OCV Register */
	len = max17043_read_data(client, MAX17043_RCOMP_MSB, &values[0], 4);
	org_rcomp_msb = values[0];	org_rcomp_lsb = values[1];
	org_ocv_msb = values[2];	org_ocv_lsb = values[3];

	/* 3 : Write OCV Register */
	values[0] = 0xE1; values[1] = 0x50;
	max17043_write_data(client, MAX17043_OCV_MSB, &values[0], 2);

	/* 4 : Write RCOMP Register */
	values[0] = 0xFF; values[1] = 0x00;
	max17043_write_data(client, MAX17043_RCOMP_MSB, &values[0], 2);

	/* 5 : Write the Model - Write 64bytes model */
	len = 16;
	reg = 0x40;
	values[0]=0xAD;  values[1]=0xD0;  values[2]=0xB4;  values[3]=0x70;
	values[4]=0xB7;  values[5]=0x00;  values[6]=0xB7;   values[7]=0x60;
	values[8]=0xB9;   values[9]=0x90;   values[10]=0xB9;   values[11] = 0xC0;
	values[12]=0xBA;   values[13]=0x40;   values[14]=0xBC;   values[15] = 0xA0;

	max17043_write_data(client, reg, &values[0], len);

	reg = 0x50;
	values[0]=0xBD;  values[1]=0x10;  values[2]=0xBD;  values[3]=0xB0;
	values[4]=0xC0;  values[5]=0xA0;  values[6]=0xC3;   values[7]=0xC0;
	values[8]=0xC8;   values[9]=0xF0;   values[10]=0xCD;   values[11]=0x80;
	values[12]=0xD2;   values[13]=0xF0;   values[14]=0xD7;   values[15]=0x50;

	max17043_write_data(client, reg, &values[0], len);

	reg = 0x60;
	values[0]=0x06;  values[1]=0x00;  values[2]=0x00;  values[3]=0x40;
	values[4]=0x7B;  values[5]=0x20;  values[6]=0x14;   values[7]=0x20;
	values[8]=0x00;   values[9]=0x40;   values[10]=0x77;   values[11]=0xC0;
	values[12]=0x05;   values[13]=0x40;   values[14]=0x6A;   values[15]=0xE0;

	max17043_write_data(client, reg, &values[0], len);

	reg = 0x70;
	values[0]=0x44;  values[1]=0x00;  values[2]=0x21;  values[3]=0xC0;
	values[4]=0x19;  values[5]=0xE0;  values[6]=0x10;   values[7]=0xA0;
	values[8]=0x14;   values[9]=0x20;   values[10]=0x0F;   values[11] = 0xE0;
	values[12]=0x0E;   values[13]=0x00;   values[14]=0x0E;   values[15] = 0x00;

	max17043_write_data(client, reg, &values[0], len);

	/* 6 : Delay at least 150mS */
	msleep(150);

	/* 7 : Write OCV Register */
	values[0] = 0xE1; values[1] = 0x50;
	max17043_write_data(client, MAX17043_OCV_MSB, &values[0], 2);

	/* 8 : Delay between 150mS and 600mS */
	msleep(150);

	/* 9 : Read SOC Register */
	len = max17043_read_data(client, MAX17043_SOC_MSB, &values[0], 2);

	if (values[0] >= 0xE9 && values[0] <= 0xEB)
		pr_info("%s: Model Guage was loaded successful!!!(%02x)\n", __func__, values[0]);
	else
		pr_info("%s: Model Guage was not loaded successful!!!T.T(%02x)\n", __func__, values[0]);

	/* 10 : Restore RCOMP and OCV */
	values[0] = org_rcomp_msb;	values[1] = org_rcomp_lsb;
	values[2] = org_ocv_msb;	values[3] = org_ocv_lsb;
	max17043_write_data(client, MAX17043_RCOMP_MSB, &values[0], 4);

	/* 11 : Lock Model Access */
	values[0] = 0x00; values[1] = 0x00;
	max17043_write_data(client, 0x3E, &values[0], 2);
}

static int __devinit max17043_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17043_chip *chip;
	int ret = 0;

	dev_info(&client->dev, "%s..\n", __func__);

	//if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_info(&client->dev, "%s failed\n", __func__);
		return -EIO;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_info(&client->dev, "%s failed\n", __func__);
		return -ENOMEM;
	}

	reference = chip;
#if !defined(CONFIG_MACH_VU10)
	max17043_init_model(client);
#endif
	chip->client 	= client;
	chip->pdata	= client->dev.platform_data;

	i2c_set_clientdata(client, chip);

	chip->battery.name		= "fuelgauge";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17043_get_property;
	chip->battery.properties	= max17043_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17043_battery_props);

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto err_power_supply_register_failed;
	}
#if 0
	chip->vcell = 3360;
	chip->soc = 100 << 8;
	chip->voltage = 4200;
	chip->capacity = 100;
	chip->config = 0x971C;
#endif

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17043_work);
	INIT_WORK(&chip->alert_work, max17043_alert_work);
#if 1
	max17043_read_version(client);
	max17043_read_config(client);
#if defined(CONFIG_MACH_VU10)
	max17043_set_rcomp(RCOMP_BLT3);
#else
	max17043_set_rcomp(RCOMP_BL44JN);
#endif
	max17043_set_athd(MONITOR_LEVEL);
#if defined(CONFIG_MACH_VU10)
	max17043_clear_interrupt(client);
#else
	if(x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_E)
	{
		max17043_clear_interrupt(client);
	}
#endif
#endif

#if 1
	// sysfs path : /sys/devices/platform/i2c_omap.2/i2c-2/2-0036/soc
	ret = device_create_file(&client->dev, &dev_attr_soc);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_soc_failed;
	}

	// sysfs path : /sys/devices/platform/i2c_omap.2/i2c-2/2-0036/state
	ret = device_create_file(&client->dev, &dev_attr_state);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_state_failed;
	}

/*                                                                      */
#if defined(CONFIG_MACH_VU10)
	// sysfs path : /sys/devices/platform/tegra-i2c.4/i2c-4/4-0036/por
	ret = device_create_file(&client->dev, &dev_attr_por);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_por_failed;
	}
#endif
/*                                                                      */

#endif

	max17043_update(client);
	schedule_delayed_work(&chip->work, MAX17043_WORK_DELAY);
#if defined(CONFIG_MACH_VU10)
	tegra_gpio_enable(GAUGE_INT);

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		printk(KERN_DEBUG "[MAX17043] set GAUGE_INT to wakeup source failed.\n");
		goto err_request_wakeup_irq_failed;
	}

	ret = gpio_request(GAUGE_INT, "max17043_alert");
	if (ret < 0) {
		printk(KERN_DEBUG " [MAX17043] GPIO Request Failed\n");
		goto err_gpio_request_failed;
	}
	gpio_direction_input(GAUGE_INT);

//		ret = request_irq(client->irq,	max17043_interrupt_handler,	IRQF_TRIGGER_FALLING,	"MAX17043_Alert", NULL);
	ret = request_threaded_irq(client->irq, NULL, max17043_interrupt_handler, IRQF_ONESHOT | IRQF_TRIGGER_FALLING, client->name, chip);
	if (ret < 0) {
		printk(KERN_DEBUG " [MAX17043] IRQ Request Failed\n");
		goto err_request_irq_failed;
	}
#else
	if(x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_E) // Rev 1.0
	{
		tegra_gpio_enable(GAUGE_INT);

		ret = enable_irq_wake(client->irq);
		if (ret < 0) {
			printk(KERN_DEBUG "[MAX17043] set GAUGE_INT to wakeup source failed.\n");
			goto err_request_wakeup_irq_failed;
		}

		ret = gpio_request(GAUGE_INT, "max17043_alert");
		if (ret < 0) {
			printk(KERN_DEBUG " [MAX17043] GPIO Request Failed\n");
			goto err_gpio_request_failed;
		}
		gpio_direction_input(GAUGE_INT);

//		ret = request_irq(client->irq,	max17043_interrupt_handler,	IRQF_TRIGGER_FALLING,	"MAX17043_Alert", NULL);
		ret = request_threaded_irq(client->irq, NULL, max17043_interrupt_handler, IRQF_ONESHOT | IRQF_TRIGGER_FALLING, client->name, chip);
		if (ret < 0) {
			printk(KERN_DEBUG " [MAX17043] IRQ Request Failed\n");
			goto err_request_irq_failed;
		}
	}
#endif
	return 0;

/*                                                                      */
#if defined(CONFIG_MACH_VU10)
err_create_file_por_failed:
	device_remove_file(&client->dev, &dev_attr_por);
#endif
/*                                                                      */

err_create_file_state_failed:
	device_remove_file(&client->dev, &dev_attr_soc);
err_create_file_soc_failed:
#if defined(CONFIG_MACH_VU10)
	disable_irq_wake(gpio_to_irq(GAUGE_INT));
err_request_wakeup_irq_failed:
	free_irq(gpio_to_irq(GAUGE_INT), chip);
err_request_irq_failed:
	gpio_free(GAUGE_INT);
#else
	if(x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_E) //Rev 1.0
	{
		disable_irq_wake(gpio_to_irq(GAUGE_INT));
err_request_wakeup_irq_failed:
	free_irq(gpio_to_irq(GAUGE_INT), chip);
err_request_irq_failed:
	gpio_free(GAUGE_INT);
	}
#endif
err_gpio_request_failed:
	power_supply_unregister(&chip->battery);
err_power_supply_register_failed:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	chip = NULL;
	reference = NULL;

	return ret;
}

static int __devexit max17043_remove(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_state);
	device_remove_file(&client->dev, &dev_attr_soc);

	power_supply_unregister(&chip->battery);
	cancel_delayed_work_sync(&chip->work);
	flush_work_sync(&chip->alert_work);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	chip = NULL;

	return 0;
}

#ifdef CONFIG_PM
static int max17043_suspend(struct device *dev)
{
#if defined(CONFIG_MACH_VU10)
	int alert_level = 0;
	struct max17043_chip *chip = dev_get_drvdata(dev);

	max17043_read_config(chip->client);
	alert_level = max17043_next_alert_level(chip->soc);
	max17043_set_athd(alert_level);
	cancel_delayed_work_sync(&chip->work);

	printk(KERN_DEBUG " [MAX17043] Max17043_Suspend\n");
#else
	if(x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_E)	  //Rev 1.0
	{
		int alert_level = 0;
	struct max17043_chip *chip = dev_get_drvdata(dev);

		max17043_read_config(chip->client);
		alert_level = max17043_next_alert_level(chip->soc);
		max17043_set_athd(alert_level);
		cancel_delayed_work_sync(&chip->work);

		printk(KERN_DEBUG " [MAX17043] Max17043_Suspend\n");
	}
	else
	{
		struct max17043_chip *chip = dev_get_drvdata(dev);

		cancel_delayed_work_sync(&chip->work);
	}
#endif
	return 0;
}

static int max17043_resume(struct device *dev)
{
	struct max17043_chip *chip = dev_get_drvdata(dev);
	schedule_delayed_work(&chip->work,msecs_to_jiffies(50));
	printk(KERN_DEBUG " [MAX17043] Max17043_Resume\n");
	return 0;
}

static const struct dev_pm_ops max17043_pm_ops = {
	.suspend	= max17043_suspend,
	.resume		= max17043_resume,
};
#else
#define max17043_suspend NULL
#define max17043_resume NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id max17043_id[] = {
	{ "max17043", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17043_id);

static struct i2c_driver max17043_i2c_driver = {
	.driver	= {
		.name	= "max17043",
		.owner	= THIS_MODULE,
#if defined(CONFIG_PM)
		.pm	= &max17043_pm_ops,
#endif
	},
	.probe		= max17043_probe,
	.remove		= __devexit_p(max17043_remove),
	.id_table	= max17043_id,
};

#if 0
/* boot argument from boot loader */
static s32 __init max17043_state(char *str)
{
	switch(str[0]) {
		case 'g':	// fuel gauge value is good
		case 'q':	// did quikcstart.
			need_to_quickstart = 0;
			break;
		case 'b':	// battery not connected when booting
			need_to_quickstart = 1;
			break;
		case 'e':	// quickstart needed. but error occured.
			need_to_quickstart = -1;
			break;
		default:
			// can not enter here
			break;
	}
	return 0;
}
__setup("fuelgauge=", max17043_state);
#endif

static int __init max17043_init(void)
{
	return i2c_add_driver(&max17043_i2c_driver);
}
#if defined(CONFIG_MACH_LGHDK) || defined(CONFIG_MACH_X3)
subsys_initcall(max17043_init);
#else
module_init(max17043_init);
#endif

static void __exit max17043_exit(void)
{
	i2c_del_driver(&max17043_i2c_driver);
}
module_exit(max17043_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("MAX17043 Fuel Gauge");
MODULE_LICENSE("GPL");

