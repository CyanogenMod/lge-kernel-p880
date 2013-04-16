WVGA_FLAG=0

HD_FLAG=0

ifeq ($(CONFIG_MACH_LX_HD7_HITACHI),y)
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

# 4 lx
obj-$(CONFIG_MACH_LX)				+= board-lx.o
obj-$(CONFIG_MACH_LX)				+= board-lx-kbc.o

obj-$(CONFIG_MACH_LX)				+= board-lx-panel.o

obj-$(CONFIG_MACH_LX)				+= board-lx-power.o
obj-$(CONFIG_MACH_LX)				+= board-lx-sdhci.o
obj-$(CONFIG_MACH_LX)				+= board-lx-sensors.o
obj-$(CONFIG_MACH_LX)				+= board-lx-memory.o
obj-$(CONFIG_MACH_LX)				+= board-lx-usb.o
obj-$(CONFIG_MACH_LX)				+= board-lx-bt.o
obj-$(CONFIG_MACH_LX)				+= board-lx-audio.o
obj-$(CONFIG_MACH_LX)				+= board-lx-misc.o
obj-$(CONFIG_MACH_LX)				+= board-lx-input.o
obj-$(CONFIG_MACH_LX)				+= board-lx-i2c-dev.o

#ifeq ($(CONFIG_MACH_LX),y)
#obj-$(CONFIG_MACH_SOLOMON_GPIO_SWITCH_SPI)	+= board-lx-gpio-spi-solomon.o
#obj-$(CONFIG_MACH_TOSHIBA_GPIO_SWITCH_SPI)	+= board-lx-gpio-spi-toshiba.o
#endif

# LGE_CHANGE 20111104 gps_gpio dongseon.kim@lge.com
obj-$(CONFIG_MACH_LX)				+= board-lx-gps.o
# obj-$(CONFIG_MACH_LX)				+= gps_gpio.o
# LGE_CHANGE 20111104 gps_gpio dongseon.kim@lge.com
obj-$(CONFIG_MACH_LX)				+= board-lx-baseband.o

ifeq ($(HW_REV),rev_a)
obj-y			+= board-lx-pinmux_rev_a.o
else
ifeq ($(HW_REV),rev_b)
obj-y			+= board-lx-pinmux_rev_b.o
else
ifeq ($(HW_REV),rev_c)
obj-y			+= board-lx-pinmux_rev_c.o
else
ifeq ($(HW_REV),rev_d)
obj-y			+= board-lx-pinmux_rev_d.o
else
ifeq ($(HW_REV),rev_e)
obj-y			+= board-lx-pinmux_rev_e.o
else
$(error $(HW_REV) not assigned)
endif
endif
endif
endif
endif


obj-$(CONFIG_REBOOT_MONITOR)		+= board-lx-reboot.o

obj-$(CONFIG_MACH_LX)		+= board-lx-nv.o
