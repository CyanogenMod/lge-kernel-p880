#ifndef __lm3559_flash_led_H
#define __lm3559_flash_led_H

#include <linux/ioctl.h>

//#define LM3559_MAX_TORCH_LEVEL	11
//#define LM3559_MAX_FLASH_LEVEL	20

#define LM3559_FLASH_LEVEL		        0
#define LM3559_TORCH_LEVEL		        1

#define LM3559_POWER_OFF				      1
#define LM3559_POWER_STANDBY					2
#define LM3559_POWER_ON					      3

struct lm3559_param {
	int param;
  int value;
	//__s32 sizeofvalue;
	//void *p_value;
};

#define LM3559_IOCTL_P0WER_CONT   _IOW('o', 13, int)
#define LM3559_IOCTL_FLASH_TORCH  _IOW('o', 14, struct lm3559_param)

struct lm3559_platform_data {
	//unsigned cfg; /* use the NVC_CFG_ defines */
	//unsigned num; /* see implementation notes in driver */
	//unsigned sync; /* see implementation notes in driver */
	//const char *dev_name; /* see implementation notes in driver */
	//struct lm3559_pin_state (*pinstate); /* see notes in driver */
	//unsigned max_amp_torch; /* maximum torch value allowed */
	//unsigned max_amp_flash; /* maximum flash value allowed */
	unsigned gpio_act; /* GPIO connected to the ACT signal */
};


#endif
