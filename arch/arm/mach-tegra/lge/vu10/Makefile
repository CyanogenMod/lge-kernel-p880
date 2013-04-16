WVGA_FLAG=0

HD_FLAG=0

ifeq ($(CONFIG_MACH_HD7_HITACHI),y)
HD_FLAG=1
endif

ifeq ($(CONFIG_MACH_HD5_LGD),y)
HD_FLAG=1
endif

#ifeq ($(HW_REV),rev_a)
#EXTRA_CFLAGS += -DLGE_SENSOR=1
#EXTRA_CFLAGS += -DLGE_SENSOR_FUSION=1
#EXTRA_CFLAGS += -DLGE_SENSOR_ACCELEROMETER=1
#EXTRA_CFLAGS += -DLGE_SENSOR_GYROSCOPE=1
#EXTRA_CFLAGS += -DLGE_SENSOR_DCOMPASS=1
#else
EXTRA_CFLAGS += -DMPU_SENSORS_MPU6050B1=1
#endif

# 4 VU
obj-$(CONFIG_MACH_VU10)				+= board-x3.o
obj-$(CONFIG_MACH_VU10)				+= board-x3-kbc.o

obj-$(CONFIG_MACH_VU10)				+= board-x3-panel.o

obj-$(CONFIG_MACH_VU10)				+= board-x3-power.o
obj-$(CONFIG_MACH_VU10)				+= board-x3-sdhci.o
obj-$(CONFIG_MACH_VU10)				+= board-x3-sensors.o
obj-$(CONFIG_MACH_VU10)				+= board-x3-memory.o
obj-$(CONFIG_MACH_VU10)				+= board-x3-usb.o
obj-$(CONFIG_MACH_VU10)				+= board-x3-bt.o
obj-$(CONFIG_MACH_VU10)				+= board-x3-audio.o
obj-$(CONFIG_MACH_VU10)				+= board-x3-misc.o
obj-$(CONFIG_MACH_VU10)				+= board-x3-input.o
obj-$(CONFIG_MACH_VU10)				+= board-x3-i2c-dev.o

#ifeq ($(CONFIG_MACH_VU10),y)
#obj-$(CONFIG_MACH_SOLOMON_GPIO_SWITCH_SPI)	+= board-x3-gpio-spi-solomon.o
#obj-$(CONFIG_MACH_TOSHIBA_GPIO_SWITCH_SPI)	+= board-x3-gpio-spi-toshiba.o
#endif

# LGE_CHANGE 20111104 gps_gpio dongseon.kim@lge.com
obj-$(CONFIG_MACH_VU10)				+= board-x3-gps.o
# obj-$(CONFIG_MACH_VU10)				+= gps_gpio.o
# LGE_CHANGE 20111104 gps_gpio dongseon.kim@lge.com
obj-$(CONFIG_MACH_VU10)				+= board-x3-baseband.o

obj-$(CONFIG_MACH_VU10)				+= board-x3-pinmux.o

obj-$(CONFIG_REBOOT_MONITOR)		+= board-x3-reboot.o

obj-$(CONFIG_MACH_VU10)		+= board-x3-nv.o
