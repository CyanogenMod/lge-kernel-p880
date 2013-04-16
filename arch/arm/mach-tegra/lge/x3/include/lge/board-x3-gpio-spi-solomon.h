
#define LGIT_IEF
#define LGIT_CABC
#define BKCHUNG_CABC_TEST

static unsigned int shift_bit[9] = { 
	(1 << 8),
	(1 << 7),	
	(1 << 6),
	(1 << 5),
	(1 << 4),
	(1 << 3),
	(1 << 2),
	(1 << 1),
	(1 << 0)	
};

struct spi_cmd_data {
	int dtype;
	int wait;
	int dlen;
	char *payload;
};


#define DEVICE_ID_REGISTER					0xB0
#define RGB_INTERFACE_CTRL_REG_1			0xB1
#define RGB_INTERFACE_CTRL_REG_2			0xB2
#define RGB_INTERFACE_CTRL_REG_3			0xB3
#define RGB_INTERFACE_CTRL_REG_4			0xB4
#define RGB_INTERFACE_CTRL_REG_5			0xB5
#define RGB_INTERFACE_CTRL_REG_6			0xB6
#define CONFIGURATION_REGISTER_0			0xB7
#define VC_CONTROL_REGISTER_0				0XB8
#define PLL_CONTROL_REGISTER				0xB9
#define PLL_CONFIGURATION_REGISTER			0xBA
#define CLOCK_CONTROL_REGISTER_0			0xBB
#define PACKET_SIZE_CONTROL_REGISTER_1_0 	0xBC
#define PACKET_SIZE_CONTROL_REGISTER_2_0 	0xBD
#define PACKET_SIZE_CONTROL_REGISTER_3_0 	0xBE
#define PACKET_DROP_REGISTER_0				0xBF
#define LINE_CTRL_REGISTER_0				0xC4
#define LANE_CONFIGURATION_REGISTER			0xDE
#define TEST_REGISTER_0						0xD6
#define ANALOG_CONTROL_REGISTER_1				0xD8 //add

#define DCS_READ_NUM_ERRORS		0x05
#define DCS_READ_POWER_MODE		0x0a
#define DCS_READ_MADCTL			0x0b
#define DCS_READ_PIXEL_FORMAT	0x0c
#define DCS_RDDSDR				0x0f
#define DCS_COLUMN_ADDR			0x2a
#define DCS_PAGE_ADDR			0x2b
#define DCS_MEMORY_WRITE		0x2c
#define DCS_TEAR_OFF			0x34
#define DCS_TEAR_ON				0x35
#define DCS_MEM_ACC_CTRL		0x36
#define DCS_PIXEL_FORMAT		0x3a

#if defined(LGIT_CABC)

#define DCS_BRIGHTNESS			0x51
#define DCS_CTRL_DISPLAY		0x53
#define DCS_WRITE_CABC			0x55
#define DCS_READ_CABC			0x56
//add
#define	DCS_WRITE_CABC_MIN_BRIGHTNESS	0x5e
#define DCS_BACKLIGHT_CONTROL		0xc8

#endif

#define DCS_GET_ID				0xf8



#define DCS_SLEEP_IN			0x10
#define DCS_SLEEP_OUT			0x11
#define DCS_DISPLAY_OFF			0x28
#define DCS_DISPLAY_ON			0x29
#define DCS_DEEP_STANDBY_IN		0xC1



static char ssd2825_lp_mode_test01[2] = {0x01, 0x03};
static char ssd2825_lp_mode_test02[2] = {0x49, 0x00};
static char ssd2825_lp_mode_test03[2] = {0x01, 0x00};
static char ssd2825_lp_mode_test04[2] = {0x11, 0x00};

static struct spi_cmd_data ssd2825_lp_mode_test[] = {
	{  CONFIGURATION_REGISTER_0, 			0, sizeof(ssd2825_lp_mode_test01),  ssd2825_lp_mode_test01},
	{  VC_CONTROL_REGISTER_0, 				0, sizeof(ssd2825_lp_mode_test02),  ssd2825_lp_mode_test02},
	{  PACKET_SIZE_CONTROL_REGISTER_1_0, 	0, sizeof(ssd2825_lp_mode_test03),  ssd2825_lp_mode_test03},
	{  PACKET_DROP_REGISTER_0, 				0, sizeof(ssd2825_lp_mode_test04),  ssd2825_lp_mode_test04},

};



static char ssd2825_0[2] = {0x04, 0x04};
static char ssd2825_1[2] = {0x3a, 0x0e};
static char ssd2825_2[2] = {0x08, 0x04};
static char ssd2825_3[2] = {0xd0, 0x02};
static char ssd2825_4[2] = {0x00, 0x05};

static char ssd2825_5[2] = {0x03, 0x00};

static char ssd2825_6[2] = {0x03, 0x00};
static char ssd2825_7[2] = {0x04, 0x00};
static char ssd2825_8[2] = {0x00, 0x00};

static char ssd2825_9[2] = {0x01, 0x00};
static char ssd2825_10[2] = {0x14, 0x80};
static char ssd2825_11[2] = {0x08, 0x00};
static char ssd2825_12[2] = {0x01, 0x00};
static char ssd2825_13[2] = {0x00, 0x00};

static char ssd2825_14[2] = {0x1c, 0x02}; // add


static struct spi_cmd_data init_ssd2825_sequence[] = {
	{  RGB_INTERFACE_CTRL_REG_1, 	0, sizeof(ssd2825_0),  ssd2825_0},
	{  RGB_INTERFACE_CTRL_REG_2, 	0, sizeof(ssd2825_1),  ssd2825_1},
	{  RGB_INTERFACE_CTRL_REG_3, 	0, sizeof(ssd2825_2),  ssd2825_2},
	{  RGB_INTERFACE_CTRL_REG_4, 	0, sizeof(ssd2825_3),  ssd2825_3},
	{  RGB_INTERFACE_CTRL_REG_5, 	0, sizeof(ssd2825_4),  ssd2825_4},
	{  RGB_INTERFACE_CTRL_REG_6, 	0, sizeof(ssd2825_5),  ssd2825_5},
	{  LANE_CONFIGURATION_REGISTER, 0, sizeof(ssd2825_6),  ssd2825_6},
	{  TEST_REGISTER_0,			 	0, sizeof(ssd2825_7),  ssd2825_7},
	{  PLL_CONTROL_REGISTER,		0, sizeof(ssd2825_8),  ssd2825_8}, 
	{  LINE_CTRL_REGISTER_0, 		0, sizeof(ssd2825_9),  ssd2825_9},
	{  PLL_CONFIGURATION_REGISTER,	0, sizeof(ssd2825_10),  ssd2825_10},
	{  CLOCK_CONTROL_REGISTER_0, 	0, sizeof(ssd2825_11),  ssd2825_11},
	{  PLL_CONTROL_REGISTER, 		0, sizeof(ssd2825_12),  ssd2825_12},
	{  VC_CONTROL_REGISTER_0, 		0, sizeof(ssd2825_13),  ssd2825_13},
	{  ANALOG_CONTROL_REGISTER_1, 		0, sizeof(ssd2825_14),  ssd2825_14}, //add
};


static char read_01[2] = {0xfa, 0x00};

static struct spi_cmd_data ssd2825_chip_read_sequence[] = {
	{  0xd4, 	2, sizeof(read_01),  read_01},
	{  0xb0, 	2, 0,  0},
	{  0xfa,	0, 0,  0},
};

static char lgd_status_test01[2] = {0x82, 0x03};

static char lgd_status_01[2] = {0x01, 0x00};
static char lgd_status_02[1] = {0x0a,0x00};

static struct spi_cmd_data lgd_status_read_sequence[] = {
	{  0xb7, 				2, sizeof(lgd_status_test01),  lgd_status_test01},
	{  0xbc, 				2, sizeof(lgd_status_01),  lgd_status_01},/*packet size*/
	{  0xbf,				0, sizeof(lgd_status_02),  lgd_status_02},
	
};

static char ssd2825_pll [2] 			= {0x01, 0x00};
static char ssd2825_config_reg_0 [2] 	= {0x02, 0x03}; 
static char ssd2825_vc_ctrl_reg_0 [2] 	= {0x00, 0x00};


static char ssd2825_pkt_sz_ctrl_reg_01 [2] = {0x01, 0x00};
static char ssd2825_pkt_sz_ctrl_reg_02 [2] = {0x02, 0x00};
static char ssd2825_pkt_sz_ctrl_reg_03 [2] = {0x03, 0x00};
static char ssd2825_pkt_sz_ctrl_reg_04 [2]	= {0x04, 0x00};
static char ssd2825_pkt_sz_ctrl_reg_05 [2]	= {0x05, 0x00};
static char ssd2825_pkt_sz_ctrl_reg_06 [2]	= {0x06, 0x00};
static char ssd2825_pkt_sz_ctrl_reg_09 [2] = {0x09, 0x00};
static char ssd2825_pkt_sz_ctrl_reg_0a [2] = {0x0A, 0x00};



static char dsi_config    [6] = {0xE0, 0x43, 0x00, 0x80, 0x00, 0x00};
static char display_mode1 [6] = {0xB5, 0x14, 0x20, 0x40, 0x00, 0x00};
static char display_mode2 [6] = {0xB6, 0x01, 0x16, 0x0F, 0x16, 0x13};


static char p_gamma_r_setting[10] = {0xD0, 0x00, 0x30, 0x56, 0x01, 0x12, 0x04, 0x34, 0x10, 0x01};
static char n_gamma_r_setting[10] = {0xD1, 0x02, 0x51, 0x56, 0x06, 0x08, 0x00, 0x47, 0x12, 0x02};
static char p_gamma_g_setting[10] = {0xD2, 0x00, 0x30, 0x56, 0x01, 0x12, 0x04, 0x34, 0x10, 0x01};
static char n_gamma_g_setting[10] = {0xD3, 0x02, 0x51, 0x56, 0x06, 0x08, 0x00, 0x47, 0x12, 0x02};
static char p_gamma_b_setting[10] = {0xD4, 0x00, 0x30, 0x56, 0x01, 0x12, 0x04, 0x34, 0x10, 0x01};
static char n_gamma_b_setting[10] = {0xD5, 0x02, 0x51, 0x56, 0x06, 0x08, 0x00, 0x47, 0x12, 0x02};

static char osc_setting[3] =     {0xC0, 0x01, 0x08};


static char power_setting_test01[2] ={0xc1,0x00};
static char power_setting_test02[2] ={0xc2,0x02};
static char power_setting_test03[2] ={0xc2,0x06};
static char power_setting_test04[2] ={0xc2,0x4e};

static char power_setting3[10] = {0xC3, 0x00, 0x08, 0x00, 0x00, 0x00, 0x67, 0x88, 0x32,0x02};//{0xC3, 0x01, 0x08, 0x00, 0x00, 0x00, 0x67, 0x88, 0x32,0x02};
static char power_setting4[6] =  {0xC4, 0x22, 0x24, 0x19, 0x19, 0x41};
static char otp2_setting[2] =    {0XF9, 0x00};



#if defined(LGIT_IEF)
static char ief_set0[2] = {0x70, 0x07};
static char ief_set1[5] = {0x71, 0x00, 0x00, 0x01, 0x01};
static char ief_set2[3] = {0x72, 0x01, 0x0F};
static char ief_set3[4] = {0x73, 0x34, 0x55, 0x00};
static char ief_set4[4] = {0x74, 0x04, 0x01, 0x07};
static char ief_set5[4] = {0x75, 0x03, 0x0F, 0x07};
static char ief_set6[4] = {0x76, 0x07, 0x00, 0x05}; 
#endif

#if defined(LGIT_CABC)

#if defined(BKCHUNG_CABC_TEST)
static char cabc_set0[2] = {0x51, 0xFF};  // adjust the brightness value of the display ::: FF --> MAX reverse !!!!!!!!!!!!!!!
static char cabc_set1[2] = {0x5E, 0xC0};  // set minimum brightness < 11101000 > <C0> 192  original <E8>
static char cabc_set2[2] = {0x53, 0x2C};  // control ambient light, brightness and gamma settings --> < 00101100 >
						// < x | x | Brightness Control | x | Dimming | Backlight ON/OFF | x | x >
static char cabc_set3[2] = {0x55, 0x01};  // image content based adaptive brightness
//static char cabc_set4[5] = {0xC8, 0xFF, 0xE3, 0x0F, 0xFF}; // setting last 2  >> 0x01 0x11 change to 0x0f 0xff for dimming test
static char cabc_set4[5] = {0xC8, 0xFF, 0x0F, 0xE3, 0x22}; // setting last 2  >> 0x01 0x11 change to 0x0f 0xff for dimming test

#else
static char cabc_set0[2] = {0x51, 0xFF};  // adjust the brightness value of the display ::: FF --> MAX reverse !!!!!!!!!!!!!!!
static char cabc_set1[2] = {0x5E, 0xC0};  // set minimum brightness < 11101000 > <C0> 192  original <E8>
static char cabc_set2[2] = {0x53, 0x2C};  // control ambient light, brightness and gamma settings --> < 00101100 >
						// < x | x | Brightness Control | x | Dimming | Backlight ON/OFF | x | x >
static char cabc_set3[2] = {0x55, 0x01};  // image content based adaptive brightness
//static char cabc_set4[5] = {0xC8, 0xFF, 0xE3, 0x0F, 0xFF}; // setting last 2  >> 0x01 0x11 change to 0x0f 0xff for dimming test
static char cabc_set4[5] = {0xC8, 0x11, 0x01, 0xE3, 0x22}; // setting last 2  >> 0x01 0x11 change to 0x0f 0xff for dimming test
#endif

#endif

static char exit_sleep[2] =  {DCS_SLEEP_OUT,0x00};
static char display_on[2] =  {DCS_DISPLAY_ON,0x00};
static char enter_sleep[2] = {DCS_SLEEP_IN,0x00};
static char display_off[2] = {DCS_DISPLAY_OFF,0x00};
static char deep_standby[2] = {DCS_DEEP_STANDBY_IN,0x01};



static char ssd2825_sleep_0 [2] 			= {0x42, 0x03};
									
static char ssd2825_zero_pkt_2 	[2] 		= {0x00, 0x00};




static char lgit_power_on_test01[3] = {0xc6 ,0x23, 0x40};
static char lgit_power_on_test02[3] = {0xc6 ,0x20, 0x40};
static char lgit_power_on_test03[3] = {0xc6 ,0x27, 0x40};


//                                    
#if defined(CONFIG_MACH_X3_HD_HITACHI)

static char set_address_mode [2] 	= {0x36, 0x00};
/* ----- set_address_mode -----
	* [D7] Page Address Order   	: 0 >> Top to Bottom. 	1 >> Bottom to Top
	* [D6] Column Address Order 	: 0 >> Left to Right. 	1 >> Right to Left.
	* [D3] RGB Order 				: 0 >> RGB  			1 >> BGR
*/

static char set_pixel_format [2] 	= {0x3A, 0x70}; 
/* ----- set_pixel_format -----
    * 18 bits/pixel : 0x60
	* 24 bits/pixel : 0x70
*/



static struct spi_cmd_data hitachi_power_on_set[] = {
	{PLL_CONTROL_REGISTER,	 			0,	sizeof(ssd2825_pll),				ssd2825_pll},
	{CONFIGURATION_REGISTER_0,			0,	sizeof(ssd2825_config_reg_0), 		ssd2825_config_reg_0},
	{VC_CONTROL_REGISTER_0,				0,	sizeof(ssd2825_vc_ctrl_reg_0), 		ssd2825_vc_ctrl_reg_0},
	{PACKET_SIZE_CONTROL_REGISTER_1_0, 	0, 	sizeof(ssd2825_pkt_sz_ctrl_reg_02),	ssd2825_pkt_sz_ctrl_reg_02},
	{PACKET_DROP_REGISTER_0,	 		0, 	sizeof(set_address_mode),			set_address_mode},
	{PACKET_DROP_REGISTER_0, 			0, 	sizeof(set_pixel_format),			set_pixel_format},
};

#elif defined(CONFIG_MACH_X3_HD_LGD)


static struct spi_cmd_data lgit_power_on_set[] = {
	{PLL_CONTROL_REGISTER,	 			0,	sizeof(ssd2825_pll),			ssd2825_pll},
	{CONFIGURATION_REGISTER_0,			0,	sizeof(ssd2825_config_reg_0), 	ssd2825_config_reg_0},
	{VC_CONTROL_REGISTER_0,				0,	sizeof(ssd2825_vc_ctrl_reg_0), 	ssd2825_vc_ctrl_reg_0},
	{PACKET_SIZE_CONTROL_REGISTER_1_0, 	0, sizeof(ssd2825_pkt_sz_ctrl_reg_06),	ssd2825_pkt_sz_ctrl_reg_06},
	{PACKET_DROP_REGISTER_0,	 		0, sizeof(dsi_config    ),			dsi_config   },
	{PACKET_DROP_REGISTER_0, 			0, sizeof(display_mode1 ),			display_mode1},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(display_mode2 ),			display_mode2},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_0a),ssd2825_pkt_sz_ctrl_reg_0a},
	{PACKET_DROP_REGISTER_0,			0, sizeof(p_gamma_r_setting),		p_gamma_r_setting},
	{PACKET_DROP_REGISTER_0,			0, sizeof(n_gamma_r_setting),		n_gamma_r_setting},	
	{PACKET_DROP_REGISTER_0,			0, sizeof(p_gamma_g_setting),		p_gamma_g_setting},
	{PACKET_DROP_REGISTER_0,			0, sizeof(n_gamma_g_setting),		n_gamma_g_setting},
	{PACKET_DROP_REGISTER_0,			0, sizeof(p_gamma_b_setting),		p_gamma_b_setting},
	{PACKET_DROP_REGISTER_0,			0, sizeof(n_gamma_b_setting),		n_gamma_b_setting},

#if defined(LGIT_IEF)
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_02),ssd2825_pkt_sz_ctrl_reg_02},
	{PACKET_DROP_REGISTER_0,			0, sizeof(ief_set0),				ief_set0},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_05),ssd2825_pkt_sz_ctrl_reg_05},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ief_set1),				ief_set1},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_03),ssd2825_pkt_sz_ctrl_reg_03},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ief_set2),				ief_set2},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_04),ssd2825_pkt_sz_ctrl_reg_04},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ief_set3),				ief_set3},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ief_set4),				ief_set4},
	{PACKET_DROP_REGISTER_0,			0, sizeof(ief_set5),				ief_set5},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ief_set6),				ief_set6},
#endif

#if defined(LGIT_CABC)
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_02),ssd2825_pkt_sz_ctrl_reg_02},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(cabc_set0),				cabc_set0},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(cabc_set1),				cabc_set1},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(cabc_set2),				cabc_set2},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(cabc_set3),				cabc_set3},

	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_05),ssd2825_pkt_sz_ctrl_reg_05},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(cabc_set4),				cabc_set4},
#endif
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_03),ssd2825_pkt_sz_ctrl_reg_03},
	{PACKET_DROP_REGISTER_0,			0, sizeof(osc_setting),				osc_setting},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_0a),ssd2825_pkt_sz_ctrl_reg_0a},
	{PACKET_DROP_REGISTER_0,			0, sizeof(power_setting3),			power_setting3},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_06),ssd2825_pkt_sz_ctrl_reg_06},
	{PACKET_DROP_REGISTER_0,			0, sizeof(power_setting4),			power_setting4},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_02),ssd2825_pkt_sz_ctrl_reg_02},
	{PACKET_DROP_REGISTER_0,			10, sizeof(otp2_setting),			otp2_setting},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_pkt_sz_ctrl_reg_02),ssd2825_pkt_sz_ctrl_reg_02},
	{PACKET_DROP_REGISTER_0,			20, sizeof(power_setting_test01),			power_setting_test01},
	{PACKET_DROP_REGISTER_0,			20, sizeof(power_setting_test02),			power_setting_test02},
	{PACKET_DROP_REGISTER_0,			20, sizeof(power_setting_test03),			power_setting_test03},
	{PACKET_DROP_REGISTER_0,			20, sizeof(power_setting_test04),			power_setting_test04},
};
#endif
//                                  


static char ssd2825_leave_sleep_0_pll 			[2] 			= {0x42, 0x03};
static char ssd2825_leave_sleep_1_pll 			[2] 			= {0x00, 0x00};
static char ssd2825_leave_sleep_2_pll 			[2] 			= {0x02, 0x00};

static char test_111 [2] ={0x6a,0x95};
static char test_112 [2] ={0x10,0x80};
static char test_113 [2] ={0x0, 0x0};
static char ief_set_test[2] = {0x11, 0x00};

static char power_setting3_test[9] = {0x01, 0x08, 0x00, 0x00, 0x00, 0x67, 0x88, 0x32,0x02};//{0xC3, 0x01, 0x0A, 0x00, 0x00, 0x00, 0x77, 0x88, 0x34, 0x02};


static struct spi_cmd_data LGD_leave_sleep_sequence[] = {
	{CONFIGURATION_REGISTER_0,			0, sizeof(ssd2825_leave_sleep_0_pll),			ssd2825_leave_sleep_0_pll},

	{VC_CONTROL_REGISTER_0,				0, sizeof(ssd2825_leave_sleep_1_pll),			ssd2825_leave_sleep_1_pll},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_leave_sleep_1_pll),			ssd2825_leave_sleep_1_pll},
	{DCS_SLEEP_OUT,						0, 0,											0},
	{DCS_DISPLAY_ON,					0, 0,											0},
};

static char ssd2825_leave_hs_sleep_0_pll 			[2] 			= {0x43, 0x03};
static char ssd2825_leave_hs_sleep_1_pll 			[2] 			= {0x00, 0x00};

static struct spi_cmd_data LGD_leave_hs_sleep_sequence[] = {
	{CONFIGURATION_REGISTER_0,			0, sizeof(ssd2825_leave_hs_sleep_0_pll),			ssd2825_leave_sleep_0_pll},
	{VC_CONTROL_REGISTER_0,				0, sizeof(ssd2825_leave_hs_sleep_1_pll),			ssd2825_leave_sleep_1_pll},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_leave_hs_sleep_1_pll),			ssd2825_leave_sleep_1_pll},
	{DCS_SLEEP_IN, 					0, 0,											0},
	{DCS_DISPLAY_OFF,					0, 0,											0},
};

static char ssd2825_hs_0_pll 			[2] 			= {0x03, 0x03};
static char ssd2825_hs_1_pll 			[2] 			= {0x0a, 0x00};
static char ssd2825_hs_cmd				[2] 			= {0xfe, 0xfe};


static struct spi_cmd_data LGD_hs_sequence[] = {
	{CONFIGURATION_REGISTER_0,			0, sizeof(ssd2825_hs_0_pll),			ssd2825_hs_0_pll},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,	0, sizeof(ssd2825_hs_1_pll),			ssd2825_hs_1_pll},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ssd2825_hs_cmd),				ssd2825_hs_cmd},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ssd2825_hs_cmd),				ssd2825_hs_cmd},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ssd2825_hs_cmd),				ssd2825_hs_cmd},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ssd2825_hs_cmd),				ssd2825_hs_cmd},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ssd2825_hs_cmd),				ssd2825_hs_cmd},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ssd2825_hs_cmd),				ssd2825_hs_cmd},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ssd2825_hs_cmd),				ssd2825_hs_cmd},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ssd2825_hs_cmd),				ssd2825_hs_cmd},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ssd2825_hs_cmd),				ssd2825_hs_cmd},
	{PACKET_DROP_REGISTER_0, 			0, sizeof(ssd2825_hs_cmd),				ssd2825_hs_cmd},
};

static char ssd2825_video_pll 			[2] 			= {0x14, 0x80};
static char ssd2825_video_clk_ctrl_reg [2] 				= {0x3f, 0x00};
static char ssd2825_video_pll_ctrl_reg [2] 				= {0x01, 0x00};
static char ssd2825_video_vc_ctrl_reg 	[2] 			= {0x00, 0x00};
static char ssd2825_video_config_reg 	[2] 			= {0x09, 0x03};


static struct spi_cmd_data ssd2825_video_sequence[] = {
	{PLL_CONFIGURATION_REGISTER,	0, sizeof(ssd2825_video_pll),			ssd2825_video_pll},
	{CLOCK_CONTROL_REGISTER_0,		0, sizeof(ssd2825_video_clk_ctrl_reg),	ssd2825_video_clk_ctrl_reg},
	{PLL_CONTROL_REGISTER,			0, sizeof(ssd2825_video_pll_ctrl_reg),	ssd2825_video_pll_ctrl_reg},
	{VC_CONTROL_REGISTER_0,			0, sizeof(ssd2825_video_vc_ctrl_reg),	ssd2825_video_vc_ctrl_reg},
	{CONFIGURATION_REGISTER_0,		0, sizeof(ssd2825_video_config_reg),	ssd2825_video_config_reg},
};



static struct spi_cmd_data lgit_power_off_set[] = {
	{CONFIGURATION_REGISTER_0, 				0,	sizeof(ssd2825_sleep_0),	ssd2825_sleep_0},
	{VC_CONTROL_REGISTER_0,					0,	sizeof(ssd2825_zero_pkt_2),		ssd2825_zero_pkt_2},
	{PACKET_SIZE_CONTROL_REGISTER_1_0,		0, 	sizeof(ssd2825_pkt_sz_ctrl_reg_02),ssd2825_pkt_sz_ctrl_reg_02},
	{PACKET_DROP_REGISTER_0, 				100, sizeof(display_off), 		display_off}, 
	{PACKET_DROP_REGISTER_0, 				150, sizeof(enter_sleep), 		enter_sleep}, 
	{PACKET_DROP_REGISTER_0, 				0, sizeof(deep_standby), 		deep_standby},
};



	
