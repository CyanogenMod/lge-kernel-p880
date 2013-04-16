
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/timer.h>
#define KP_LEDS_GPIO 139

u8 pwrkey_led_status = 0;

struct timer_list	 blink_timer;
static void power_led_store(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	printk("%s ***  naveen value:%d \n",__func__,value);

	if(led_cdev->br_maintain_trigger == 1){
		return;
	}

	if (pwrkey_led_status > 0) {
		return;
	}

	if (value == 0){
		gpio_set_value(KP_LEDS_GPIO, 0);
	}
	else {
		gpio_set_value(KP_LEDS_GPIO, 1);
	}
}


	struct led_classdev power_led_class_dev ={
	.name = "power-key-led",
	.brightness_set = power_led_store,
	.max_brightness = LED_FULL,
	.brightness = LED_OFF,

	};

static void led_timer_function(unsigned long data)
{

	unsigned long brightness;
	unsigned long delay;

	brightness = gpio_get_value(KP_LEDS_GPIO);
	printk("############Brightness : %ld \n" , brightness);
	if (!brightness) {
		/* Time to switch the LED on. */
		brightness = LED_FULL;
		delay = 300;
	} else {
		/* Store the current brightness value to be able
		 * to restore it when the delay_off period is over.
		 */
		brightness = LED_OFF;
		delay = 200;
	}

	gpio_set_value(KP_LEDS_GPIO, brightness);
	mod_timer(&blink_timer, jiffies + msecs_to_jiffies(delay));
}


bool power_key_let_set_state(struct device *dev , uint led_set){
//	int ret = 0;
//	unsigned long delay_on;
//	unsigned long delay_off;

//	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	printk("[naveen] %s() %d\n", __func__, led_set);

	if (pwrkey_led_status > 0)
		return 1;

	switch (led_set){
	 case 0:/* Turn off LED */
		gpio_set_value(KP_LEDS_GPIO, 0);
		break;
	 case 1:/* Always ON LED */
		gpio_set_value(KP_LEDS_GPIO, 1);
		break;
	 case 2:/* Blinking LED */
		mod_timer(&blink_timer, jiffies + 1);
		break;
	case 3:
		/* Stop blinking */
		del_timer_sync(&blink_timer);

		break;
	default :
		return 1;
	}
	return 0; //Success to set LED
}
static ssize_t key_led_state_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int level;
	level = simple_strtoul(buf, NULL, 10);
	power_key_let_set_state(dev ,level);
	return count;
}

DEVICE_ATTR(led_set_state, 0660, 0, key_led_state_store);

EXPORT_SYMBOL(power_key_let_set_state);

bool set_pwrkey_led_state(uint led_set) {
	switch (led_set){

	case 0:/* Turn off LED */
		pwrkey_led_status = 0;
		gpio_set_value(KP_LEDS_GPIO, 0);
		break;
	case 1:/* Always ON LED */
		pwrkey_led_status = 1;
		gpio_set_value(KP_LEDS_GPIO, 1);
		break;
	case 2:/* Blinking LED */
		pwrkey_led_status = 2;
		mod_timer(&blink_timer, jiffies + 1);
		break;
	case 3:
		/* Stop blinking */
		del_timer_sync(&blink_timer);
		break;
	default :
		return 0;
	}
	return 1; //Success to set LED
}
EXPORT_SYMBOL(set_pwrkey_led_state);


static int __devinit power_led_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = gpio_request(KP_LEDS_GPIO, "power_key_led");
	if(ret){
		printk(KERN_ERR "[kp_led]: naveen request gpio %d failed!\n", KP_LEDS_GPIO);
		return ret;
	}

	tegra_gpio_enable(KP_LEDS_GPIO);
	ret = gpio_direction_output(KP_LEDS_GPIO, 0);
	if(ret){
		printk(KERN_ERR"[kp_led]: naveen set gpio %d direction out error!\n", KP_LEDS_GPIO);
		return ret;
	}



	ret = led_classdev_register(&pdev->dev,
				    &power_led_class_dev);
	if (ret < 0) {
		pr_err("[kp_led]: %s: naveen Register led class failed\n", __func__);

		return ret;
	}


	ret = device_create_file(&pdev->dev, &dev_attr_led_set_state);
	if (ret < 0) {
		printk("attr set_led_state failed to create \n");
	}

	init_timer(&blink_timer);
	blink_timer.function = led_timer_function;
	blink_timer.data = (unsigned long)1;
	printk("naveen power_led_probe success \n");

	set_pwrkey_led_state(pwrkey_led_status);

	return ret;
}

static int power_led_remove(struct platform_device *pdev)
{
	tegra_gpio_disable(KP_LEDS_GPIO);
	if (gpio_is_valid(KP_LEDS_GPIO)) {
		gpio_free(KP_LEDS_GPIO);
	}
	device_remove_file(&pdev->dev, &dev_attr_led_set_state);
	led_classdev_unregister(&power_led_class_dev);
	return 0;
}

#ifdef CONFIG_PM
static int power_key_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	printk("[naveen] power__key_led suspend \n");
	led_classdev_suspend(&power_led_class_dev);

	return 0;
}

static int power_key_led_resume(struct platform_device *dev)
{
	printk("[naveen] power__key_led resume \n");
	led_classdev_resume(&power_led_class_dev);
	return 0;
}
#else
#define power_key_led_suspend NULL
#define power_key_led_resume NULL
#endif

static struct platform_driver power_led_driver = {
	.probe = power_led_probe,
	.remove = power_led_remove,
	.suspend	= power_key_led_suspend,
	.resume		= power_key_led_resume,
	.driver = {
		.name = "power-key-led",
		.owner = THIS_MODULE,
	},
};

static int __init power_led_init(void)
{
	return platform_driver_register(&power_led_driver);
}

static void __exit power_led_exit(void)
{
	platform_driver_unregister(&power_led_driver);
}

module_init(power_led_init);
module_exit(power_led_exit);

MODULE_DESCRIPTION("AP30 power Lighting driver");
MODULE_LICENSE("GPL");
