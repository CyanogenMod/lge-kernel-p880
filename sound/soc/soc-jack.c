/*
 * soc-jack.c  --  ALSA SoC jack handling
 *
 * Copyright 2008 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <sound/jack.h>
#include <sound/soc.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <trace/events/asoc.h>
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
#include <linux/hook_test_header.h> //                                                              
#include <linux/wakelock.h> //                                                                                       

//                                             
unsigned int hook_status = HOOK_RELEASED;
headset_type_enum headset_type = X3_NONE;
int hook_detection_time = 20;//100; hyunsub.na for hookkey double click (= mp3 next play)
static struct wake_lock hook_wake_lock; //                                          
#endif
//                                             

/**
 * snd_soc_jack_new - Create a new jack
 * @card:  ASoC card
 * @id:    an identifying string for this jack
 * @type:  a bitmask of enum snd_jack_type values that can be detected by
 *         this jack
 * @jack:  structure to use for the jack
 *
 * Creates a new jack object.
 *
 * Returns zero if successful, or a negative error code on failure.
 * On success jack will be initialised.
 */
int snd_soc_jack_new(struct snd_soc_codec *codec, const char *id, int type,
		     struct snd_soc_jack *jack)
{
	jack->codec = codec;
	INIT_LIST_HEAD(&jack->pins);
	INIT_LIST_HEAD(&jack->jack_zones);
	BLOCKING_INIT_NOTIFIER_HEAD(&jack->notifier);

	return snd_jack_new(codec->card->snd_card, id, type, &jack->jack);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_new);

/**
 * snd_soc_jack_report - Report the current status for a jack
 *
 * @jack:   the jack
 * @status: a bitmask of enum snd_jack_type values that are currently detected.
 * @mask:   a bitmask of enum snd_jack_type values that being reported.
 *
 * If configured using snd_soc_jack_add_pins() then the associated
 * DAPM pins will be enabled or disabled as appropriate and DAPM
 * synchronised.
 *
 * Note: This function uses mutexes and should be called from a
 * context which can sleep (such as a workqueue).
 */
void snd_soc_jack_report(struct snd_soc_jack *jack, int status, int mask)
{
	struct snd_soc_codec *codec;
	struct snd_soc_dapm_context *dapm;
	struct snd_soc_jack_pin *pin;
	int enable;
	int oldstatus;

	trace_snd_soc_jack_report(jack, mask, status);

	if (!jack)
		return;

	codec = jack->codec;
	dapm =  &codec->dapm;

	mutex_lock(&codec->mutex);

	oldstatus = jack->status;

	jack->status &= ~mask;
	jack->status |= status & mask;

	/* The DAPM sync is expensive enough to be worth skipping.
	 * However, empty mask means pin synchronization is desired. */
	if (mask && (jack->status == oldstatus))
		goto out;

	trace_snd_soc_jack_notify(jack, status);

	list_for_each_entry(pin, &jack->pins, list) {
		enable = pin->mask & jack->status;

		if (pin->invert)
			enable = !enable;

		if (enable)
			snd_soc_dapm_enable_pin(dapm, pin->pin);
		else
			snd_soc_dapm_disable_pin(dapm, pin->pin);
	}

	/* Report before the DAPM sync to help users updating micbias status */
	blocking_notifier_call_chain(&jack->notifier, status, jack);

	snd_soc_dapm_sync(dapm);

	snd_jack_report(jack->jack, jack->status);

out:
	mutex_unlock(&codec->mutex);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_report);

/**
 * snd_soc_jack_add_zones - Associate voltage zones with jack
 *
 * @jack:  ASoC jack
 * @count: Number of zones
 * @zone:  Array of zones
 *
 * After this function has been called the zones specified in the
 * array will be associated with the jack.
 */
int snd_soc_jack_add_zones(struct snd_soc_jack *jack, int count,
			  struct snd_soc_jack_zone *zones)
{
	int i;

	for (i = 0; i < count; i++) {
		INIT_LIST_HEAD(&zones[i].list);
		list_add(&(zones[i].list), &jack->jack_zones);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_zones);

/**
 * snd_soc_jack_get_type - Based on the mic bias value, this function returns
 * the type of jack from the zones delcared in the jack type
 *
 * @micbias_voltage:  mic bias voltage at adc channel when jack is plugged in
 *
 * Based on the mic bias value passed, this function helps identify
 * the type of jack from the already delcared jack zones
 */
int snd_soc_jack_get_type(struct snd_soc_jack *jack, int micbias_voltage)
{
	struct snd_soc_jack_zone *zone;

	list_for_each_entry(zone, &jack->jack_zones, list) {
		if (micbias_voltage >= zone->min_mv &&
			micbias_voltage < zone->max_mv)
				return zone->jack_type;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_get_type);

/**
 * snd_soc_jack_add_pins - Associate DAPM pins with an ASoC jack
 *
 * @jack:  ASoC jack
 * @count: Number of pins
 * @pins:  Array of pins
 *
 * After this function has been called the DAPM pins specified in the
 * pins array will have their status updated to reflect the current
 * state of the jack whenever the jack status is updated.
 */
int snd_soc_jack_add_pins(struct snd_soc_jack *jack, int count,
			  struct snd_soc_jack_pin *pins)
{
	int i;

	for (i = 0; i < count; i++) {
		if (!pins[i].pin) {
			printk(KERN_ERR "No name for pin %d\n", i);
			return -EINVAL;
		}
		if (!pins[i].mask) {
			printk(KERN_ERR "No mask for pin %d (%s)\n", i,
			       pins[i].pin);
			return -EINVAL;
		}

		INIT_LIST_HEAD(&pins[i].list);
		list_add(&(pins[i].list), &jack->pins);
	}

	/* Update to reflect the last reported status; canned jack
	 * implementations are likely to set their state before the
	 * card has an opportunity to associate pins.
	 */
	snd_soc_jack_report(jack, 0, 0);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_pins);

/**
 * snd_soc_jack_notifier_register - Register a notifier for jack status
 *
 * @jack:  ASoC jack
 * @nb:    Notifier block to register
 *
 * Register for notification of the current status of the jack.  Note
 * that it is not possible to report additional jack events in the
 * callback from the notifier, this is intended to support
 * applications such as enabling electrical detection only when a
 * mechanical detection event has occurred.
 */
void snd_soc_jack_notifier_register(struct snd_soc_jack *jack,
				    struct notifier_block *nb)
{
	blocking_notifier_chain_register(&jack->notifier, nb);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_notifier_register);

/**
 * snd_soc_jack_notifier_unregister - Unregister a notifier for jack status
 *
 * @jack:  ASoC jack
 * @nb:    Notifier block to unregister
 *
 * Stop notifying for status changes.
 */
void snd_soc_jack_notifier_unregister(struct snd_soc_jack *jack,
				      struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&jack->notifier, nb);
}
EXPORT_SYMBOL_GPL(snd_soc_jack_notifier_unregister);

#ifdef CONFIG_GPIOLIB
/* gpio detect */
static void snd_soc_jack_gpio_detect(struct snd_soc_jack_gpio *gpio)
{
	struct snd_soc_jack *jack = gpio->jack;
	int enable;
	int report;

	enable = gpio_get_value_cansleep(gpio->gpio);
	if (gpio->invert)
		enable = !enable;

	if (enable)
		report = gpio->report;
	else
		report = 0;

	if (gpio->jack_status_check)
		report = gpio->jack_status_check();

	snd_soc_jack_report(jack, report, gpio->report);
}

/* irq handler for gpio pin */
static irqreturn_t gpio_handler(int irq, void *data)
{
	struct snd_soc_jack_gpio *gpio = data;
	struct device *dev = gpio->jack->codec->card->dev;

	trace_snd_soc_jack_irq(gpio->name);

	if (device_may_wakeup(dev))
		pm_wakeup_event(dev, gpio->debounce_time + 50);

	schedule_delayed_work(&gpio->work,
			      msecs_to_jiffies(gpio->debounce_time));

	return IRQ_HANDLED;
}

/* gpio work */
//                                             
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
static void hook_det_work(struct work_struct *work)
{
    unsigned int gpio_headset = 0, gpio_hook = 0;
    
    gpio_headset = gpio_get_value(headset_sw_data->gpio);
    gpio_hook = gpio_get_value(headset_sw_data->hook_gpio);
	
//    printk(KERN_ERR "##(hook_det_work)## xxxxxxx  headset pin = %d, hook pin = %d \n",gpio_headset, gpio_hook);
	//                                                                                             
    msleep(20);
    msleep(20);
    msleep(20);
    msleep(20);
    msleep(20);
    msleep(20);
    msleep(20);
    msleep(20);
    msleep(20);
    msleep(20);
	//                                                                                             
    if(headset_type != X3_HEADSET)
		return;
	
    wake_lock_timeout(&hook_wake_lock, HZ * 5);//                                             
	if(hook_status == HOOK_RELEASED){
		if(gpio_hook == 0){ 
		    hook_status = HOOK_PRESSED;
		    if ( get_test_mode() != 1 ) {
		        input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 1);
			    input_sync(headset_sw_data->ip_dev);
			} else {
			    write_gkpd_value(KEY_HOOK); //                                                  
			}
			printk(KERN_ERR "##(hook_det_work)## HOOK PRESSED\n");
		}
    }
	else{
		if(gpio_hook == 1){
		    hook_status = HOOK_RELEASED;
		    if ( get_test_mode() != 1 ) {
		        input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 0);
			    input_sync(headset_sw_data->ip_dev);
			}
			printk( KERN_ERR "##(hook_det_work)## HOOK RELEASED\n");
		}
	}
}

static irqreturn_t headset_hook_int_handler(int irq, void *dev_id)
{
	static i=0; //20130110 keunhui.park [Audio] prevent pressing hook key while unplugging headset
	struct headset_switch_data	*switch_data =
	    (struct headset_switch_data *)dev_id;

	printk(KERN_ERR "##(Headset_det.c)## headset_hook_int_handler()!! headset type %d\n", headset_type );
	
	if(headset_type == X3_HEADSET)
	{
	//                                                                                             
		schedule_delayed_work(&switch_data->hook_delayed_work[i++],	msecs_to_jiffies(hook_detection_time));
		if (i>3)
			i=0;
	//                                                                                             
	}

	return IRQ_HANDLED;
}
#endif
//                                             

static void gpio_work(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio;
//                                             
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
	int headset_gpio_status;
	int hookkey_gpio_status;
#endif
//                                             

	gpio = container_of(work, struct snd_soc_jack_gpio, work.work);
	snd_soc_jack_gpio_detect(gpio);

//                                             
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
	headset_gpio_status = gpio_get_value (headset_sw_data->gpio);
	hookkey_gpio_status = gpio_get_value (headset_sw_data->hook_gpio);

	if (headset_gpio_status == 0)
	{
		headset_type = X3_NONE;

		if (hookkey_gpio_status == 0)
		{
			headset_type = X3_HEADPHONE;
//			gpio_set_value(headset_sw_data->ear_mic, 1);
		}
		else
		{
			headset_type = X3_HEADSET;
//			gpio_set_value(headset_sw_data->ear_mic, 1);
		}
	}
	else
	{
		headset_type = X3_NONE;
//		gpio_set_value(headset_sw_data->ear_mic, 0);
	//                                                                                             
		if(hook_status == HOOK_PRESSED){
		    hook_status = HOOK_RELEASED;
		    if ( get_test_mode() != 1 ) {
		        input_report_key(headset_sw_data->ip_dev, KEY_HOOK, 0);
			    input_sync(headset_sw_data->ip_dev);
			}
			printk( KERN_ERR "##(hook_det_work)## HOOK RELEASED\n");
			msleep(20);
			msleep(20);
			msleep(20);
			msleep(20);
			msleep(20);
			msleep(20);
			msleep(20);
			msleep(20);
			msleep(20);
			msleep(20);
		}
	//                                                                                             
	}

	switch_set_state(&headset_sw_data->sdev, headset_type);
#endif
//                                             
}

/**
 * snd_soc_jack_add_gpios - Associate GPIO pins with an ASoC jack
 *
 * @jack:  ASoC jack
 * @count: number of pins
 * @gpios: array of gpio pins
 *
 * This function will request gpio, set data direction and request irq
 * for each gpio in the array.
 */
int snd_soc_jack_add_gpios(struct snd_soc_jack *jack, int count,
			struct snd_soc_jack_gpio *gpios)
{
	int i, ret;

	for (i = 0; i < count; i++) {
		if (!gpio_is_valid(gpios[i].gpio)) {
			printk(KERN_ERR "Invalid gpio %d\n",
				gpios[i].gpio);
			ret = -EINVAL;
			goto undo;
		}
		if (!gpios[i].name) {
			printk(KERN_ERR "No name for gpio %d\n",
				gpios[i].gpio);
			ret = -EINVAL;
			goto undo;
		}

		ret = gpio_request(gpios[i].gpio, gpios[i].name);
		if (ret)
			goto undo;

		ret = gpio_direction_input(gpios[i].gpio);
		if (ret)
			goto err;

#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
		tegra_gpio_enable (gpios[i].gpio);	//                                             
#endif

		INIT_DELAYED_WORK(&gpios[i].work, gpio_work);
		gpios[i].jack = jack;

		ret = request_any_context_irq(gpio_to_irq(gpios[i].gpio),
					      gpio_handler,
					      IRQF_TRIGGER_RISING |
					      IRQF_TRIGGER_FALLING,
					      gpios[i].name,
					      &gpios[i]);
		if (ret < 0)
			goto err;

		if (gpios[i].wake) {
			ret = irq_set_irq_wake(gpio_to_irq(gpios[i].gpio), 1);
			if (ret != 0)
				printk(KERN_ERR
				  "Failed to mark GPIO %d as wake source: %d\n",
					gpios[i].gpio, ret);
		}

  //                                                                  
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
    ret = enable_irq_wake(gpio_to_irq(gpios[i].gpio));
    if (ret) {
      printk("%s: enable_irq_wake error\n", __func__);
      free_irq(gpio_to_irq(gpios[i].gpio), NULL);
      return ret;
    }
  //                                                                  
    
//                                             
		ret = gpio_request(headset_sw_data->ear_mic, "ear_mic");
		ret = gpio_direction_output(headset_sw_data->ear_mic, 0);
		if (ret)
			goto err;
		tegra_gpio_enable (headset_sw_data->ear_mic);


		ret = gpio_request(headset_sw_data->hook_gpio, "hook_det");
		ret = gpio_direction_input(headset_sw_data->hook_gpio);
		if (ret)
			goto err;
		tegra_gpio_enable (headset_sw_data->hook_gpio);		
	//                                                                                             
		INIT_DELAYED_WORK(&headset_sw_data->hook_delayed_work[0], hook_det_work);
		INIT_DELAYED_WORK(&headset_sw_data->hook_delayed_work[1], hook_det_work);
		INIT_DELAYED_WORK(&headset_sw_data->hook_delayed_work[2], hook_det_work);
		INIT_DELAYED_WORK(&headset_sw_data->hook_delayed_work[3], hook_det_work);
	//                                                                                             

		headset_sw_data->hook_irq = gpio_to_irq(headset_sw_data->hook_gpio);
		
		ret = request_irq(headset_sw_data->hook_irq, headset_hook_int_handler,
			(IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING), "hook_det", headset_sw_data);
		if (ret)
			goto err;

    //                                                                    
    ret = enable_irq_wake(headset_sw_data->hook_irq);
    if (ret) {
      printk("%s: enable_irq_wake error\n", __func__);
      free_irq(headset_sw_data->hook_irq, NULL);
      return ret;
    }
    //                                                                    

		gpio_set_value(headset_sw_data->ear_mic, 1);

		schedule_delayed_work(&gpios[i].work,0);
#endif
//                                             

#ifdef CONFIG_GPIO_SYSFS
		/* Expose GPIO value over sysfs for diagnostic purposes */
		gpio_export(gpios[i].gpio, false);
#endif

		/* Update initial jack status */
		snd_soc_jack_gpio_detect(&gpios[i]);
	}

  //                                                                          
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
  wake_lock_init(&hook_wake_lock, WAKE_LOCK_SUSPEND, "hook_wake_lock");
#endif
  //                                            
	return 0;

err:
	gpio_free(gpios[i].gpio);
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
	gpio_free(headset_sw_data->hook_gpio);
	gpio_free(headset_sw_data->ear_mic);
#endif
undo:
	snd_soc_jack_free_gpios(jack, i, gpios);

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_add_gpios);
//                                                                 
#if defined(CONFIG_MACH_X3) || defined(CONFIG_MACH_LX) || defined(CONFIG_MACH_VU10)
int snd_soc_jack_check(struct snd_soc_jack_gpio *gpios)
{
	tegra_gpio_enable (gpios[0].gpio);	
	snd_soc_jack_gpio_detect(&gpios[0]);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_jack_check);
#endif


/**
 * snd_soc_jack_free_gpios - Release GPIO pins' resources of an ASoC jack
 *
 * @jack:  ASoC jack
 * @count: number of pins
 * @gpios: array of gpio pins
 *
 * Release gpio and irq resources for gpio pins associated with an ASoC jack.
 */
void snd_soc_jack_free_gpios(struct snd_soc_jack *jack, int count,
			struct snd_soc_jack_gpio *gpios)
{
	int i;

	for (i = 0; i < count; i++) {
#ifdef CONFIG_GPIO_SYSFS
		gpio_unexport(gpios[i].gpio);
#endif
		free_irq(gpio_to_irq(gpios[i].gpio), &gpios[i]);
		cancel_delayed_work_sync(&gpios[i].work);
		gpio_free(gpios[i].gpio);
		gpios[i].jack = NULL;
	}
}
EXPORT_SYMBOL_GPL(snd_soc_jack_free_gpios);
#endif	/* CONFIG_GPIOLIB */
