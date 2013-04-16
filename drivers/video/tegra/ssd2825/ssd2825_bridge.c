/*
 * arch/arm/mach-tegra/panel-w1.c
 */

#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/earlysuspend.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include "ssd2825_bridge.h"
//                                                                                             
#if defined(CONFIG_MACH_VU10)
#include "solomon_spi_table_vu10.h"
#include "../../../../arch/arm/mach-tegra/lge/x3/include/lge/board-x3.h"	//                                                       
#else
#include "solomon_spi_table.h"
#endif
//                                                                                             
#include <linux/gpio.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/err.h>
#include "../../../../arch/arm/mach-tegra/clock.h"
#include "../../../../arch/arm/mach-tegra/gpio-names.h"
#define CONFIG_ESD_REG_CHECK
#ifdef CONFIG_ESD_REG_CHECK
#include <linux/workqueue.h>
#define ESD_REG_CHECK msecs_to_jiffies(2000)
typedef struct lcd_reg_cmd lcd_reg_set;
#endif
#if defined(CONFIG_SPI_SOLOMON_BRIDGE)
typedef struct spi_cmd_data16 spi_data;
#define SPI_WRITE(value) spi_write9(value);
#elif defined(CONFIG_SPI_TOSHIBA_BRIDGE)
typedef struct spi_cmd_data32 spi_data;
#define SPI_WRITE(value) spi_write32(value);
#endif

#define SEQUENCE_SIZE(array) sizeof(array)/sizeof(spi_data)

static u16 gpio_bridge_en;
static u16 gpio_lcd_en;

//                                                                                                       
#if defined(CONFIG_MACH_VU10)
static u16 gpio_lcd_en_3v;
#endif
//                                                                                                       

static u16 gpio_lcd_reset_n;
static u16 gpio_bridge_reset_n;
static DEFINE_MUTEX(bridge_init_mutex);

static struct spi_device *bridge_spi;
static struct early_suspend ssd2825_bridge_early_suspend;
#define FALSE 0
#define TRUE 1
static int x3_bridge_on = FALSE;
static int ssd2825_disable_end = TRUE;
static int ssd2825_shutdown = FALSE;
static int ssd2825_device_id;
static int ssd2825_mipi_lp;
static int ssd2825_mipi_hs;

extern struct device* lm3533_bl_dev(void);
extern int lm3533_is_ready(void);
extern int lm3533_bl_on_off(struct device *dev, int on_off);
extern int lm3533_bl_onoff_ForHM(struct device *dev,int on_off);
#ifdef CONFIG_ESD_REG_CHECK
struct work_register{
	struct delayed_work work_reg_check;
};
struct work_register *work_instance = NULL;
static int ssd2825_bridge_lcd_reg_read_esd(void);
static int ssd2825_bridge_spi_read2(u8 value);
#endif
static int ssd2825_HM_mode = FALSE;

/*                                 */
extern int dc_set_gamma_rgb(int window_n, int red,int green,int blue);
struct lcd_gamma_rgb cmdlineRGBvalue;
static int __init dc_get_gamma_cmdline(char *str)
{
	int output[3]={0,};

	sscanf(str,"%d,%d,%d ",&output[0],&output[1],&output[2]);

	cmdlineRGBvalue.red = (unsigned char)output[0];
	cmdlineRGBvalue.green = (unsigned char)output[1];
	cmdlineRGBvalue.blue= (unsigned char)output[2];

	if(((output[0]==0) && (output[1]==0) && (output[2]==0)) ||
		((output[0]==0xFF) && (output[1]==0xFF) && (output[2]==0xFF))){
		cmdlineRGBvalue.table_type = GAMMA_NV_DISABLED;
	}
	else if(((output[0]>255) || (output[1]>255) || (output[2]>255)) ||
		 ((output[0]<0) || (output[1]<0) || (output[2]<0))){
		cmdlineRGBvalue.table_type = GAMMA_NV_DISABLED;
	}
	else{
		cmdlineRGBvalue.table_type = GAMMA_NV_ENABLED;
	}

	printk(" %s R:%d,G:%d,B:%d,table_type:%d \n",__func__,
		output[0],output[1],output[2],cmdlineRGBvalue.table_type);
	return 1;
}
__setup("RGB=", dc_get_gamma_cmdline);
/*                                 */

static int lm353x_bl_on(void)
{
	int ret =0;
	struct device *dev = lm3533_bl_dev();
	if(dev != NULL) ret=lm3533_bl_on_off(dev, 1);

	return ret;
}

static int lm353x_bl_off(void)
{
	int ret =0;
	struct device *dev = lm3533_bl_dev();
	if(dev != NULL) ret=lm3533_bl_on_off(dev, 0);

	return ret;
}

static int lm353x_bl_on_ForHM(void)
{
	int ret =0;
	struct device *dev = lm3533_bl_dev();
	if(dev != NULL) ret=lm3533_bl_onoff_ForHM(dev, 1);

	return ret;
}

static int lm353x_bl_off_ForHM(void)
{
	int ret =0;
	struct device *dev = lm3533_bl_dev();
	if(dev != NULL) ret=lm3533_bl_onoff_ForHM(dev, 0);

	return ret;
}

/*                                 */
static struct clk *clk_s3;
extern struct clk *clk_get_sys(const char *dev_id, const char *con_id);
extern struct clk *clk_get_sys(const char *dev_id, const char *con_id);
extern int clk_set_parent(struct clk *c, struct clk *parent);
extern int clk_set_rate(struct clk *c, unsigned long rate);
extern int clk_enable(struct clk *c);
extern void clk_disable(struct clk *c);
int first_clk_disable_before_suspend = 1;
void tegra_set_clk_out_parent(int enable)
{
	int ret1=0, ret2=0, ret3=0, ret4=0;
	struct clk *clk_s1 = clk_get_sys(NULL,"pll_p");
	struct clk *clk_s2 = clk_get_sys("extern3",NULL);
	ret1=clk_set_parent(clk_s2, clk_s1);
	ret2=clk_set_rate(clk_s2, 24000000); //24Mhz
	clk_s3 = clk_get_sys("clk_out_3","extern3");
	ret3=clk_set_parent(clk_s3, clk_s2);
	if(enable){
		ret4=clk_enable(clk_s3);
	}
	else{
		clk_disable(clk_s3);
	}
	printk(" clk_s3->auto_dvfs:%d, clk_s3->refcnt:%d \n",clk_s3->auto_dvfs,clk_s3->refcnt);
	printk("%s ---ret1:%d, ret2:%d, ret3:%d, ret4:%d, enable:%d \n", __func__,ret1,ret2,ret3,ret4,enable);
}
/*                                 */

static int spi_write9(u16 value)
{
	struct spi_message m;
	struct spi_transfer xfer;

	u8 w[2];
	int ret;

	spi_message_init(&m);

	memset(&xfer, 0, sizeof(xfer));

	w[1] = (value & 0xFF00) >> 8;
	w[0] = (value & 0x00FF);
	xfer.tx_buf = w;

	xfer.bits_per_word = 9;
	xfer.len = 2;
	spi_message_add_tail(&xfer, &m);

	ret = spi_sync(bridge_spi, &m);

	if (ret < 0)
		dev_warn(&bridge_spi->dev, "failed to write to value (%d)\n", value);
	return ret;
}

static int spi_read_bytes(u8 value, u16 *data)
{
	u8	tx_buf[2];
	u8	rx_buf[2];
	u16	rxtmp0, rxtmp1;
	int	rc;
	struct spi_message  m;
	struct spi_transfer xfer[2];

	memset(&xfer, 0, sizeof(xfer));

	tx_buf[1] = (value & 0xFF00) >> 8;
	tx_buf[0] = (value & 0x00FF);
	xfer[0].tx_buf = tx_buf;
	xfer[0].bits_per_word = 9;
	xfer[0].len = 2;

	xfer[1].rx_buf = rx_buf;
	xfer[1].bits_per_word = 16;
	xfer[1].len = 2;

	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);

	rc = spi_sync(bridge_spi, &m);
	rxtmp0=(u16)rx_buf[0];
	rxtmp1=(u16)rx_buf[1];
	*data=(rxtmp1 | (rxtmp0<<8));
	if (rc)
		printk(KERN_ERR "spi_sync_read failed %d\n", rc);
	return rc;
}

static int spi_read_byte16(u8 value, u16 *data)
{
	u8	tx_buf[2];
	u8	rx_buf[2];
	int	rc;
	struct spi_message  m;
	struct spi_transfer xfer[2];

	memset(&xfer, 0, sizeof(xfer));

	tx_buf[1] = (value & 0xFF00) >> 8;
	tx_buf[0] = (value & 0x00FF);
	xfer[0].tx_buf = tx_buf;
	xfer[0].bits_per_word = 9;
	xfer[0].len = 2;

	xfer[1].rx_buf = rx_buf;
	xfer[1].bits_per_word = 16;
	xfer[1].len = 2;

	spi_message_init(&m);
	spi_message_add_tail(&xfer[0], &m);
	spi_message_add_tail(&xfer[1], &m);

	rc = spi_sync(bridge_spi, &m);
	*data=(rx_buf[1] | (((u16)rx_buf[0])<<8));
	printk(KERN_INFO "rx_buf[0]:0x%x,rx_buf[1]:0x%x \n", rx_buf[0],rx_buf[1]);
	if (rc)
		printk(KERN_ERR "spi_sync_read failed %d\n", rc);
	return rc;
}


#ifdef CONFIG_ESD_REG_CHECK
/* To ESD test: check register 0xC4, if the value is wrong, re-initialze registers  */
/*                                                    */
static void reg_check(struct work_struct *work)
{
	u16 readdata1,readdata2;
	u8 data1,data2;
	//printk(KERN_INFO "%s ***** x3_bridge_on : %d \n", __func__, x3_bridge_on);
	if(!ssd2825_shutdown){
	if(x3_bridge_on && ssd2825_disable_end && !ssd2825_HM_mode){
		readdata1=ssd2825_bridge_spi_read2(0xB7);
		readdata2=ssd2825_bridge_lcd_reg_read_esd();
		data1=(u8)(readdata1&0x00FF);
		data2=(u8)(readdata2&0x00FF);
		//printk(KERN_INFO "register check 0xB7:0x%x , 0x0A:0x%x \n", data1, data2);
		if ((lm3533_is_ready() && (!((data1==0xC9)||(data1==0x49)||(data1==0x9)))) || (data2 != 0x1C))
		{
			printk(KERN_INFO "[ssd2825]register check 0xB7:0x%x , 0x0A:0x%x \n", data1, data2);
			printk(KERN_INFO "[ssd2825]re-initalize register");
			ssd2825_bridge_disable();
			mdelay(100);
			ssd2825_bridge_enable();
			schedule_delayed_work(&work_instance->work_reg_check, ESD_REG_CHECK);
		}
	}
	}
}
#endif
static int ssd2825_bridge_HitachiLcd_spi_read(int cmd)
{
	int cnt, read_cnt,read_cnt2,read_cnt3,read_cnt4,read_cnt5;
	spi_data *sequence,*sequence2,*sequence3,*sequence4,*sequence5;
	u16 readdata1;

	sequence =solomon_reg_read_set;
	sequence2 = solomon_reg_read_set2;
	sequence3 = solomon_reg_read_set3;
	read_cnt=SEQUENCE_SIZE(solomon_reg_read_set);
	read_cnt2=SEQUENCE_SIZE(solomon_reg_read_set2);
	read_cnt3=SEQUENCE_SIZE(solomon_reg_read_set3);
	printk("%s *** cmd:%d \n",__func__, cmd);

	switch (cmd){
		case 2 :
			sequence4 = solomon_reg_read_set4;
			read_cnt4=SEQUENCE_SIZE(solomon_reg_read_set4);
			break;
		case 3 :
			sequence4 = solomon_reg_read_set5;
			read_cnt4=SEQUENCE_SIZE(solomon_reg_read_set5);
			break;
		case 4 :
			sequence4 = solomon_reg_read_set6;
			read_cnt4=SEQUENCE_SIZE(solomon_reg_read_set6);
			sequence5 = solomon_reg_read_set9;
			read_cnt5=SEQUENCE_SIZE(solomon_reg_read_set9);
			break;
		case 5 :
			sequence4 = solomon_reg_read_set7;
			read_cnt4=SEQUENCE_SIZE(solomon_reg_read_set7);
			sequence5 = solomon_reg_read_set9;
			read_cnt5=SEQUENCE_SIZE(solomon_reg_read_set9);
			break;
		default :
			printk("[ssd2825]wrong cmd : return fail\n");
			return -1;
	}

	for ( cnt = 0 ; cnt < read_cnt4 ; cnt++ )
	{
		SPI_WRITE(sequence4->value)
		if( sequence4->delay ) mdelay( sequence4->delay );
		sequence4++;
	}
	spi_read_bytes(0xFA,&readdata1);

	for ( cnt = 0 ; cnt < read_cnt3 ; cnt++ )
	{
		SPI_WRITE(sequence3->value)
		if( sequence3->delay ) mdelay( sequence3->delay );
		sequence3++;
	}
	spi_read_bytes(0xFA,&readdata1);

	for ( cnt = 0 ; cnt < read_cnt2 ; cnt++ )
	{
		SPI_WRITE(sequence2->value)
		if( sequence2->delay ) mdelay( sequence2->delay );
		sequence2++;
	}

	if((cmd==2) || (cmd==3)){
		ssd2825_mipi_lp=0;
		ssd2825_mipi_hs=0;
		spi_read_byte16(0xFA,&readdata1);
		if(cmd==2) ssd2825_mipi_lp=(0x00FF&readdata1);
		else if(cmd==3) ssd2825_mipi_hs=(0x00FF&readdata1);
	}
	else if((cmd==4) || (cmd==5)){
		spi_read_byte16(0xFA,&readdata1);
		spi_read_byte16(0xFA,&readdata1);
		spi_read_byte16(0xFA,&readdata1);
		spi_read_byte16(0xFA,&readdata1);
		spi_read_byte16(0xFA,&readdata1);
	}

	if((cmd==4) || (cmd==5)){
		for ( cnt = 0 ; cnt < read_cnt5 ; cnt++ )
		{
			SPI_WRITE(sequence5->value)
			if( sequence5->delay ) mdelay( sequence5->delay );
			sequence5++;
		}
	}
//	printk("solomon_reg_read_set9 write success \n");

	for ( cnt = 0 ; cnt < read_cnt ; cnt++ )
	{
		SPI_WRITE(sequence->value)
		if( sequence->delay ) mdelay( sequence->delay );
		sequence++;
	}
	printk("solomon_reg_read_set write success *** spi read finished \n");
	return 0;
}

static int ssd2825_bridge_spi_read(void)
{
	int cnt, read_cnt;
	u16 readdata1;
	spi_data *sequence;
	sequence =solomon_reg_read_set8;
	read_cnt=SEQUENCE_SIZE(solomon_reg_read_set8);
	printk("%s *** read_cnt:%d \n",__func__,read_cnt);
	for ( cnt = 0 ; cnt < read_cnt ; cnt++ )
	{
		SPI_WRITE(sequence->value)
		if( sequence->delay ) mdelay( sequence->delay );
		sequence++;
	}
	spi_read_byte16(0xFA,&readdata1);
	ssd2825_device_id = readdata1;
	printk("ssd2825_bridge_spi_read success *** spi read finished \n");
	return readdata1;
}

static int ssd2825_bridge_spi_read2(u8 value)
{
	u16 regcmd;
	u16 readdata1;
	regcmd= (0x00FF & value);
	//printk("%s *** regcmd:%d \n",__func__,regcmd);
	spi_write9(0x00D4);
	spi_write9(0x01FA);
	spi_write9(0x0100);
	spi_write9(regcmd);
	spi_read_bytes(0xFA,&readdata1);
	//printk("ssd2825_bridge_spi_read2 success *** spi read finished \n");
	return readdata1;
}

static int ssd2825_bridge_lcd_reg_read_esd(void)
{
	int cnt, read_cnt4_1;
	u16 readdata1;
	spi_data *sequence4_1;

		sequence4_1 = solomon_reg_read_set4_1;
		read_cnt4_1=SEQUENCE_SIZE(solomon_reg_read_set4_1);
		for ( cnt = 0 ; cnt < read_cnt4_1 ; cnt++ )
		{
			SPI_WRITE(sequence4_1->value)
			if( sequence4_1->delay ) mdelay( sequence4_1->delay );
			sequence4_1++;
		}
		spi_write9(0x010A);
		spi_write9(0x0100);
		mdelay(20);
		spi_write9(0x00C6);
		spi_read_bytes(0xFA,&readdata1);
		mdelay(10);
		spi_write9(0x00FF);
		spi_read_bytes(0xFA,&readdata1);

	return readdata1;
}

struct rgb_bridge_gpio {
	char *name;
	int gpio;
};

static int rgb_bridge_gpio_init;

static struct rgb_bridge_gpio rgb_bridge_gpios_for_spi[] = {
	{ "LGD_RGB_SDI", TEGRA_GPIO_PZ2 },
	{ "LCD_RGB_SDO", TEGRA_GPIO_PN5 },
	{ "LCD_RGB_SCL", TEGRA_GPIO_PZ4 },
	{ "LCD_RGB_CS", TEGRA_GPIO_PN4 },
};

void ssd2825_bridge_enable_spi_pins_to_nomal(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(rgb_bridge_gpios_for_spi); i++) {
		tegra_pinmux_set_tristate(
			gpio_to_pingroup[rgb_bridge_gpios_for_spi[i].gpio],
			TEGRA_TRI_NORMAL);
		tegra_gpio_disable(rgb_bridge_gpios_for_spi[i].gpio);
	}
}

void ssd2825_bridge_disable_spi_pins_to_tristate(void)
{
	int i;

	if (!rgb_bridge_gpio_init && ARRAY_SIZE(rgb_bridge_gpios_for_spi) > 0) {
		for (i = 0; i < ARRAY_SIZE(rgb_bridge_gpios_for_spi); i++)
			gpio_request(rgb_bridge_gpios_for_spi[i].gpio,
						 rgb_bridge_gpios_for_spi[i].name);
		rgb_bridge_gpio_init = 1;
	}

	for (i = 0; i < ARRAY_SIZE(rgb_bridge_gpios_for_spi); i++) {
		tegra_pinmux_set_tristate(
			gpio_to_pingroup[rgb_bridge_gpios_for_spi[i].gpio],
			TEGRA_TRI_TRISTATE);
		gpio_direction_input(rgb_bridge_gpios_for_spi[i].gpio);
		tegra_gpio_enable(rgb_bridge_gpios_for_spi[i].gpio);
	}
}

int ssd2825_bridge_enable(void)
{
	int cnt, ret;
	spi_data *sequence;
	int total_cnt =0;
	printk(KERN_INFO "%s ***** x3_bridge_on : %d \n", __func__, x3_bridge_on);
	mutex_lock(&bridge_init_mutex);
	 if(!x3_bridge_on){
		ssd2825_bridge_enable_spi_pins_to_nomal();
#if defined(CONFIG_SPI_SOLOMON_BRIDGE)
		total_cnt = SEQUENCE_SIZE(solomon_init_sequence_set);
		sequence = solomon_init_sequence_set;
#elif defined(CONFIG_SPI_TOSHIBA_BRIDGE)
		total_cnt = SEQUENCE_SIZE(toshiba_init_sequence_set_updated);//toshiba_init_sequence_set);
		sequence = toshiba_init_sequence_set_updated;//toshiba_init_sequence_set;
#endif
		/* Power Sequence */
		gpio_set_value(gpio_bridge_en, 1);
		mdelay(1);
//                                                                                                       
#if defined(CONFIG_MACH_VU10)
		if (x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_B){
			gpio_set_value(gpio_lcd_en, 1);
			mdelay(2);
			gpio_set_value(gpio_lcd_en_3v, 1);
		}
		else{
			gpio_set_value(gpio_lcd_en, 1);
		}
#else
		gpio_set_value(gpio_lcd_en, 1);
#endif
//                                                                                                       

		gpio_set_value(gpio_bridge_reset_n, 0);
		mdelay(5);
		gpio_set_value(gpio_bridge_reset_n, 1);
		gpio_set_value(gpio_lcd_reset_n, 0);
		mdelay(1);
		gpio_set_value(gpio_lcd_reset_n, 1);
		mdelay(5);

/*                                 */
		clk_enable(clk_s3);
/*                                 */

		/* LCD_RESET_N */
		tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_LCD_CS1_N, TEGRA_PUPD_PULL_UP);

		/* Init Sequence */
		for ( cnt = 0 ; cnt < total_cnt ; cnt++ )
		{
			SPI_WRITE(sequence->value)
			/*printk(" rgb_bridge_enable > bridge spi driver : value %8x \n", sequence->value);*/

			if( sequence->delay )
				mdelay( sequence->delay );

			sequence++;
		}
		ret=lm353x_bl_on();
		if(ret==2){
			printk(KERN_INFO "lm353x_bl_on success *** state: 0 -> 1 \n");
		}

		//mdelay(10);
		x3_bridge_on = TRUE;
	}
	mutex_unlock(&bridge_init_mutex);
	printk(KERN_INFO "%s ended \n", __func__);
	return 0;
}

int ssd2825_bridge_disable(void)
{
	int cnt, ret;
	int total_cnt=0;
	spi_data  *sequence;
	printk(KERN_INFO "%s ***** x3_bridge_on : %d \n", __func__, x3_bridge_on);
	mutex_lock(&bridge_init_mutex);
	if(x3_bridge_on) {
		ret=lm353x_bl_off();
		printk(KERN_INFO "lm353x_bl_off success *** state: %d -> 0 \n", ret);
#if defined(CONFIG_SPI_SOLOMON_BRIDGE)
		total_cnt = SEQUENCE_SIZE(solomon_power_off_set);
		sequence = solomon_power_off_set;
#elif defined(CONFIG_SPI_TOSHIBA_BRIDGE)
		total_cnt = SEQUENCE_SIZE(toshiba_power_off_set);
		sequence = toshiba_power_off_set;
#endif

		for ( cnt = 0 ; cnt < total_cnt ; cnt++ )
		{
			SPI_WRITE(sequence->value)
				/*printk(" rgb_bridge_disable > bridge spi"
				  " driver : value %8x \n", sequence->value);*/
				if( sequence->delay )
					mdelay( sequence->delay );

			sequence++;
		}
		mdelay(10);

//                                                                                         
		gpio_set_value(gpio_lcd_reset_n, 0);
		mdelay(5);
//                                                                                                       
#if defined(CONFIG_MACH_VU10)
		if (x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_B){
			gpio_set_value(gpio_lcd_en_3v, 0);
			mdelay(2);
			gpio_set_value(gpio_lcd_en, 0);
		}
		else{
			gpio_set_value(gpio_lcd_en, 0);
		}
#else
		gpio_set_value(gpio_lcd_en, 0);
#endif
//                                                                                                       
		gpio_set_value(gpio_bridge_reset_n, 0);
		mdelay(5);
		gpio_set_value(gpio_bridge_en, 0);
//                                                                                         

		/* LCD_RESET_N */
		tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_LCD_CS1_N, TEGRA_PUPD_NORMAL);
/*                                 */
		if (first_clk_disable_before_suspend == 1) {
		    first_clk_disable_before_suspend = 0;
		    tegra_set_clk_out_parent(false);
		}
		else clk_disable(clk_s3);
/*                                 */
		ssd2825_bridge_disable_spi_pins_to_tristate();
		x3_bridge_on = FALSE;
		ssd2825_disable_end = TRUE;
	}
	mutex_unlock(&bridge_init_mutex);
	printk(KERN_INFO "%s ended \n", __func__);
	return 0;
}

static ssize_t ssd2825_bridge_show_device_id(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int r;
	printk("%s \n",__func__);
	//printk("%s *** device_id : 0x%x \n",__func__,device_id);
	ssd2825_bridge_spi_read();
	r = snprintf(buf, PAGE_SIZE,
			"ssd2825 bridge device_id : 0x%x \n",
			ssd2825_device_id);
	return r;
}

static ssize_t ssd2825_bridge_show_mipi_lp(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int r,ret;
	printk("%s \n",__func__);
	ssd2825_HM_mode = TRUE;
	/*Hitachi HD LCD command 0Ch : return 70h*/
	printk("MIPI DCS packet read in LP mode \n");
	ret=lm353x_bl_off_ForHM();
	if(ret==1){
		ssd2825_bridge_HitachiLcd_spi_read(2);
		mdelay(500);
		lm353x_bl_on_ForHM();
	}
	else{
		printk("Error !!! Please turn on the backlight LED and LCD with Power key \n");
	}
	r = snprintf(buf, PAGE_SIZE,
			"ssd2825 bridge mipi_lp : 0x%x \n",
			ssd2825_mipi_lp);
	ssd2825_HM_mode = FALSE;
	return r;
}

static ssize_t ssd2825_bridge_show_mipi_hs(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int r,ret;
	printk("%s \n",__func__);
	ssd2825_HM_mode = TRUE;
	/*Hitachi HD LCD command 0Ch : return 70h*/
	printk("MIPI DCS packet read in LP mode \n");
	ret=lm353x_bl_off_ForHM();
	if(ret==1){
		ssd2825_bridge_HitachiLcd_spi_read(3);
		mdelay(500);
		lm353x_bl_on_ForHM();
	}
	else{
		printk("Error !!! Please turn on the backlight LED and LCD with Power key \n");
	}
	r = snprintf(buf, PAGE_SIZE,
			"ssd2825 bridge mipi_hs : 0x%x \n",
			ssd2825_mipi_hs);
	ssd2825_HM_mode = FALSE;
	return r;
}

static ssize_t ssd2825_bridge_store_reg_dump(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	int readdata,i,sendcmd;
	sscanf(buf, "%d", &sendcmd);
	if(sendcmd==1)
	{
		/*ssd2825 all reg dump*/
		printk("\n %s *** cmd( data ) *** \n",__func__);
		for(i=0;i<sizeof(ssd2825_reg_set);i++){
			readdata=ssd2825_bridge_spi_read2(ssd2825_reg_set[i]);
			printk(" 0x%x( 0x%x ) , ",ssd2825_reg_set[i],readdata);
			if(i%10==9) printk("\n");
		}
		printk("\n");
	}

	return count;
}

static void ssd2825_bridge_reg_dump_test(void)
{
	int readdata,i;
	/*ssd2825 all reg dump*/
	printk("\n %s *** cmd(data) \n",__func__);
	for(i=0;i<sizeof(ssd2825_reg_set);i++){
		readdata=ssd2825_bridge_spi_read2(ssd2825_reg_set[i]);
		printk(" 0x%x( 0x%x ) , ",ssd2825_reg_set[i],readdata);
		if(i%10==9) printk("\n");
	}
	printk("\n");
}

static ssize_t ssd2825_bridge_reg_read(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int cmd,ret;

	sscanf(buf, "%d", &cmd);

	switch (cmd){
		case 1:	/* SSD2825 Bridge IC register vendor IC SPI read*/
			printk("SPI I/F Test for SSD2825 \n");
			ssd2825_bridge_spi_read();
			break;
		case 2:	/* Hitachi LCD register SPI read via MIPI DCS packet read in LP mode*/
			printk("MIPI DCS packet read in LP mode \n");
			ret=lm353x_bl_off_ForHM();
			if(ret==1){
				ssd2825_bridge_HitachiLcd_spi_read(2);
				mdelay(500);
				lm353x_bl_on_ForHM();
			}
			else{
				printk("Error !!! Please turn on the backlight LED and LCD with Power key \n");
			}
			break;
		case 3:	/* Hitachi LCD register SPI read via MIPI DCS packet read in HS mode*/
			printk("MIPI DCS packet read in HS mode \n");
			ret=lm353x_bl_off_ForHM();
			if(ret==1){
				ssd2825_bridge_HitachiLcd_spi_read(3);
				mdelay(500);
				lm353x_bl_on_ForHM();
			}
			else{
				printk("Error !!! Please turn on the backlight LED and LCD with Power key \n");
			}
			break;
		case 4:	/* Hitachi LCD register SPI read via MIPI Generic packet read in LP mode*/
			printk("MIPI Generic packet read in LP mode \n");
			ret=lm353x_bl_off_ForHM();
			if(ret==1){
				ssd2825_bridge_HitachiLcd_spi_read(4);
				mdelay(500);
				lm353x_bl_on_ForHM();
			}
			else{
				printk("Error !!! Please turn on the backlight LED and LCD with Power key \n");
			}
			break;
		case 5:	/* Hitachi LCD register SPI read via MIPI Generic packet read in HS mode*/
			printk("MIPI Generic packet read in HS mode \n");
			ret=lm353x_bl_off_ForHM();
			if(ret==1){
				ssd2825_bridge_HitachiLcd_spi_read(5);
				mdelay(500);
				lm353x_bl_on_ForHM();
			}
			else{
				printk("Error !!! Please turn on the backlight LED and LCD with Power key \n");
			}
			break;
		case 6:	/* SSD2825 Bridge IC register dump test*/
			printk("SSD2825 Bridge IC register dump test \n");
			ssd2825_bridge_reg_dump_test();
			break;
		default :
			break;
	}
	return count;
}

static ssize_t ssd2825_bridge_reg_read2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int cmd;

	sscanf(buf, "%d", &cmd);

	/* SSD2825 Bridge IC normal register SPI read*/
	//printk("SSD2825 Bridge IC normal register SPI read *** cmd 0x%x \n",cmd);
	ssd2825_bridge_spi_read2(cmd);

	return count;
}

static ssize_t display_gamma_tuning_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int red,green,blue;
	sscanf(buf, "%d,%d,%d",&red,&green,&blue);
	printk("RED:%d GREEN:%d BLUE:%d\n",red,green,blue);
	dc_set_gamma_rgb(0,red,green,blue);
	if(cmdlineRGBvalue.table_type==GAMMA_NV_ENABLED){
		cmdlineRGBvalue.table_type=GAMMA_NV_ENABLED_SEND;
	}
	else if(cmdlineRGBvalue.table_type==GAMMA_NV_SAVED){
		cmdlineRGBvalue.table_type=GAMMA_NV_SAVED_SEND;
	}
	else{
		cmdlineRGBvalue.table_type=GAMMA_NV_SEND;
	}

	return size;
}

static ssize_t display_gamma_saved_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int red,green,blue;
	sscanf(buf, "%d,%d,%d",&red,&green,&blue);
	printk("RED:%d GREEN:%d BLUE:%d\n",red,green,blue);
	dc_set_gamma_rgb(0,red,green,blue);
	cmdlineRGBvalue.table_type=GAMMA_NV_SAVED;
	cmdlineRGBvalue.red=red;
	cmdlineRGBvalue.green=green;
	cmdlineRGBvalue.blue=blue;

	return size;
}
DEVICE_ATTR(gamma_tuning, 0660, NULL, display_gamma_tuning_store);
DEVICE_ATTR(gamma_saved, 0660, NULL, display_gamma_saved_store);
DEVICE_ATTR(device_id, 0660, ssd2825_bridge_show_device_id, NULL);
DEVICE_ATTR(mipi_lp, 0660, ssd2825_bridge_show_mipi_lp, NULL);
DEVICE_ATTR(mipi_hs, 0660, ssd2825_bridge_show_mipi_hs, NULL);
DEVICE_ATTR(reg_dump, 0660, NULL, ssd2825_bridge_store_reg_dump);
DEVICE_ATTR(reg_read, 0660, NULL, ssd2825_bridge_reg_read);
DEVICE_ATTR(reg_read2, 0660, NULL, ssd2825_bridge_reg_read2);

void ssd2825_bridge_spi_suspend(struct early_suspend * h)
{
/*                                 *//*2012/01/19*/
	ssd2825_disable_end = FALSE;
	printk(KERN_INFO "%s start \n", __func__);
#ifdef CONFIG_ESD_REG_CHECK
	cancel_delayed_work_sync(&work_instance->work_reg_check);
#endif
	ssd2825_bridge_disable();
	if(cmdlineRGBvalue.table_type==GAMMA_NV_ENABLED_SEND)
		cmdlineRGBvalue.table_type=GAMMA_NV_ENABLED;
	else if(cmdlineRGBvalue.table_type==GAMMA_NV_SAVED_SEND)
		cmdlineRGBvalue.table_type=GAMMA_NV_SAVED;
	else if(cmdlineRGBvalue.table_type==GAMMA_NV_SEND)
		cmdlineRGBvalue.table_type=GAMMA_NV_RETURNED;
	printk(KERN_INFO "%s end \n", __func__);
	//return 0;
}

void ssd2825_bridge_spi_shutdown(struct spi_device *spi)
{
	int ret;
	printk(KERN_INFO "%s \n", __func__);
	ssd2825_disable_end = FALSE;
	ret=lm353x_bl_off();
	printk(KERN_INFO "lm353x_bl_off *** state: %d -> 0 \n", ret);
#ifdef CONFIG_ESD_REG_CHECK
	cancel_delayed_work_sync(&work_instance->work_reg_check);
#endif
	ssd2825_bridge_disable();
}

void ssd2825_bridge_spi_resume(struct early_suspend * h)
{
	printk(KERN_INFO "%s start \n", __func__);
#ifdef CONFIG_ESD_REG_CHECK
	schedule_delayed_work(&work_instance->work_reg_check, msecs_to_jiffies(100));
#endif
	printk(KERN_INFO "%s end \n", __func__);
	//return 0;
}

static int ssd2825_bridge_reboot_notify(struct notifier_block *nb,
                                unsigned long event, void *data)
{
	printk("%s\n", __func__);

	switch (event) {
		case SYS_RESTART:
		case SYS_HALT:
		case SYS_POWER_OFF:
		{
			ssd2825_shutdown = TRUE;
			ssd2825_disable_end = FALSE;
#ifdef CONFIG_ESD_REG_CHECK
			cancel_delayed_work(&work_instance->work_reg_check);
#endif
			//ret=lm353x_bl_off();
			//printk(KERN_INFO "lm353x_bl_off success *** state: %d -> 0 \n", ret);

			ssd2825_bridge_disable();
		}
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}


static struct notifier_block ssd2825_bridge_reboot_nb = {
	.notifier_call = ssd2825_bridge_reboot_notify,
};

static int ssd2825_bridge_spi_probe(struct spi_device *spi)
{
	struct bridge_platform_data *pdata;
	int ret,err;
	pdata = spi->dev.platform_data;

	if(!pdata){
		printk("ssd2825_bridge_spi_probe platform data null pointer \n");
		return 0;
	}

	/* request platform data */
	spi->bits_per_word = pdata->bits_per_word;

	/*printk(" bridge spi driver : pdata->bits_per_word --> %d \n",pdata->bits_per_word);*/
	spi->mode = pdata->mode;
	spi->max_speed_hz = pdata->max_speed_hz;

	gpio_lcd_reset_n		= pdata->lcd_reset_n;
//                                                                                                       
#if defined(CONFIG_MACH_VU10)
		if (x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_B){
			gpio_lcd_en		= pdata->lcd_en;
			gpio_lcd_en_3v  = pdata->lcd_en_3v;
		}
		else{
			gpio_lcd_en		= pdata->lcd_en;
		}
#else
	gpio_lcd_en 		= pdata->lcd_en;
#endif
	gpio_bridge_en		= pdata->bridge_en;
	gpio_bridge_reset_n	= pdata->bridge_reset_n;

	/* spi set up */
	ret = spi_setup(spi);

#ifdef CONFIG_ESD_REG_CHECK
	work_instance = kzalloc(sizeof(*work_instance), GFP_KERNEL);
	INIT_DELAYED_WORK(&work_instance->work_reg_check, reg_check);
	schedule_delayed_work(&work_instance->work_reg_check, ESD_REG_CHECK);
#endif
	printk(KERN_INFO " %s \n", __func__);

	if (ret < 0) {
		dev_err(&spi->dev, "%s spi_setup failed: %d\n",  __func__, ret);
		return ret;
	}
	printk(KERN_INFO "%s: probe\n", __func__);

	/* gpio request */
	ret = gpio_request(gpio_lcd_reset_n, "panel_reset");
	if (ret < 0){
		printk(KERN_INFO "%s: gpio_lcd_reset_n gpio_request failed with %d \n", __func__, ret);
		goto gpio_lcd_reset_error;
	}
	ret = gpio_request(gpio_bridge_reset_n, "bridge_reset");
	if (ret < 0){
		printk(KERN_INFO "%s: gpio_bridge_reset_n gpio_request failed with %d \n", __func__, ret);
		goto gpio_bridge_reset_error;
	}

	ret = gpio_request(gpio_bridge_en, "panel_bridge");
	if (ret < 0){
		printk(KERN_INFO "%s: gpio_bridge_en gpio_request failed with %d \n", __func__, ret);
		goto gpio_bridge_en_error;
	}

//                                                                                                       
#if defined(CONFIG_MACH_VU10)
	if (x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_B){
		ret = gpio_request(gpio_lcd_en, "lcd_en");
		if (ret < 0){
			printk(KERN_INFO "%s: lcd_en gpio_request failed with %d \n", __func__, ret);
			goto gpio_lcd_en_error;
		}

		ret = gpio_request(gpio_lcd_en_3v, "lcd_en_3v");
		if (ret < 0){
			printk(KERN_INFO "%s: lcd_en gpio_request failed with %d \n", __func__, ret);
			goto gpio_lcd_en_error;
		}
	}
	else{
		ret = gpio_request(gpio_lcd_en, "lcd_en");
		if (ret < 0){
			printk(KERN_INFO "%s: lcd_en gpio_request failed with %d \n", __func__, ret);
			goto gpio_lcd_en_error;
		}
	}
#else
	ret = gpio_request(gpio_lcd_en, "lcd_en");
	if (ret < 0){
		printk(KERN_INFO "%s: lcd_en gpio_request failed with %d \n", __func__, ret);
		goto gpio_lcd_en_error;
	}
#endif
//                                                                                                       

	/* gpio direction & enable */

	ret = gpio_direction_output(gpio_bridge_en, 1);
	tegra_gpio_enable(gpio_bridge_en);

//                                                                                                       
#if defined(CONFIG_MACH_VU10)
	if (x3_get_hw_rev_pcb_version() > hw_rev_pcb_type_B){
		ret = gpio_direction_output(gpio_lcd_en, 1);
		tegra_gpio_enable(gpio_lcd_en);

		ret = gpio_direction_output(gpio_lcd_en_3v, 1);
		tegra_gpio_enable(gpio_lcd_en_3v);
	}
	else{
		ret = gpio_direction_output(gpio_lcd_en, 1);
		tegra_gpio_enable(gpio_lcd_en);
	}
#else
	ret = gpio_direction_output(gpio_lcd_en, 1);
	tegra_gpio_enable(gpio_lcd_en);
#endif
//                                                                                                       

	mdelay(10);
	ret = gpio_direction_output(gpio_bridge_reset_n, 1);
	ret = gpio_direction_output(gpio_lcd_reset_n, 1);
	tegra_gpio_enable(gpio_bridge_reset_n);
	tegra_gpio_enable(gpio_lcd_reset_n);
	x3_bridge_on = TRUE;

	err = device_create_file(&spi->dev, &dev_attr_device_id);
	err = device_create_file(&spi->dev, &dev_attr_mipi_lp);
	err = device_create_file(&spi->dev, &dev_attr_mipi_hs);
	err = device_create_file(&spi->dev, &dev_attr_reg_dump);
	err = device_create_file(&spi->dev, &dev_attr_reg_read);
	err = device_create_file(&spi->dev, &dev_attr_reg_read2);
	err = device_create_file(&spi->dev, &dev_attr_gamma_tuning);
	err = device_create_file(&spi->dev, &dev_attr_gamma_saved);

	bridge_spi = spi;

	ssd2825_bridge_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB-10;
	ssd2825_bridge_early_suspend.suspend = ssd2825_bridge_spi_suspend;
	ssd2825_bridge_early_suspend.resume = ssd2825_bridge_spi_resume;
	register_early_suspend(&ssd2825_bridge_early_suspend);

	register_reboot_notifier(&ssd2825_bridge_reboot_nb);

	printk(KERN_INFO "rgb_bridge_spi_probe success\n");
	goto probe_success;

/* probe error */
gpio_lcd_en_error:
	gpio_free(gpio_lcd_en);
gpio_bridge_en_error:
	gpio_free(gpio_bridge_en);
gpio_lcd_reset_error:
	gpio_free(gpio_lcd_reset_n);
gpio_bridge_reset_error:
	gpio_free(gpio_bridge_reset_n);
/* probe success */
probe_success:
	return 0;
}

static int __devexit ssd2825_bridge_spi_remove(struct spi_device *spi)
{
	device_remove_file(&spi->dev, &dev_attr_device_id);
	device_remove_file(&spi->dev, &dev_attr_mipi_lp);
	device_remove_file(&spi->dev, &dev_attr_mipi_hs);
	device_remove_file(&spi->dev, &dev_attr_reg_dump);
	device_remove_file(&spi->dev, &dev_attr_reg_read);
	device_remove_file(&spi->dev, &dev_attr_reg_read2);
	device_remove_file(&spi->dev, &dev_attr_gamma_tuning);
	device_remove_file(&spi->dev, &dev_attr_gamma_saved);
	return 0;
}

static struct spi_driver ssd2825_bridge_spi_driver = {
	.driver = {
		.name	= STR_RGB_BRIDGE,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe = ssd2825_bridge_spi_probe,
	.remove = __devexit_p(ssd2825_bridge_spi_remove),
	.shutdown = ssd2825_bridge_spi_shutdown,
};

static int __init ssd2825_bridge_init(void)
{
	return spi_register_driver(&ssd2825_bridge_spi_driver);
}

static void __exit ssd2825_bridge_exit(void)
{
	spi_unregister_driver(&ssd2825_bridge_spi_driver);
}
subsys_initcall(ssd2825_bridge_init);// chages for early probe
module_exit(ssd2825_bridge_exit);

MODULE_DESCRIPTION("LCD Driver");
MODULE_LICENSE("GPL");
