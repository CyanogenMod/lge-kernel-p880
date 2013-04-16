/*
*/

#ifndef __M24C08_MAIN_H__
#define __M24C08_MAIN_H__

#include <linux/ioctl.h> /* For IOCTL macros */

#define M24C08_IOCTL_GET_E2PROM_DATA		_IOW('o', 1, struct m24c08_register_info)
#define M24C08_IOCTL_PUT_E2PROM_DATA		_IOW('o', 2, struct m24c08_register_info)

struct m24c08_register_info {
	u8 e2prom_data[1024];
	u16 reg_addr;
	u16 length;
} __attribute__ ((packed));

struct m24c08_info {
	struct i2c_client *i2c_client;
	struct m24c08_register_info reg_info;
} __attribute__ ((packed));


#ifdef __KERNEL__
struct m24c08_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif
