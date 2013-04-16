
#define LGIT_IEF

static unsigned int shift_bit32[32] = { 
	(1 << 31),
	(1 << 30),	
	(1 << 29),
	(1 << 28),
	(1 << 27),
	(1 << 26),
	(1 << 25),
	(1 << 24),
	(1 << 23),
	(1 << 22),	
	(1 << 21),
	(1 << 20),
	(1 << 19),
	(1 << 18),
	(1 << 17),
	(1 << 16),
	(1 << 15),
	(1 << 14),	
	(1 << 13),
	(1 << 12),
	(1 << 11),
	(1 << 10),
	(1 << 9),
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
	unsigned int dtype;
	int wait;
	int dlen;
	unsigned int *payload;
};


/*-------------------------
* 		Register
--------------------------*/


/* TC358768XBG Software Reset */
#define T_RESET_SYSCTL		 			0x00020000

/* TC358768XBG PLL,Clock Setting */
#define T_PLL_CONTROL_REGISTER 			0x00160000
#define T_PLL_OSC_ENABLE				0x00180000
#define T_PLL_CLK_OUT_ENABLE			0x00180000


/* TC358768XBG DPI Input Control */
#define	T_DPI_FIFO_CONTROL_REGISTER 	0x00060000
#define T_TX_FORMAT_SETTING				0x00080000
#define	T_TX_PIXEL_STREAM_PACKET_DTYPE	0x00500000

/* TC358768XBG D-PHY Setting */
#define	T_PHY_CLK_LANE_ENABLE0			0x01400000
#define	T_PHY_CLK_LANE_ENABLE1			0x01420000
#define	T_PHY_DATA_LANE0_ENABLE0		0x01440000
#define	T_PHY_DATA_LANE0_ENABLE1		0x01460000
#define	T_PHY_DATA_LANE1_ENABLE0		0x01480000
#define	T_PHY_DATA_LANE1_ENABLE1		0x014A0000
#define	T_PHY_DATA_LANE2_ENABLE0		0x014C0000
#define	T_PHY_DATA_LANE2_ENABLE1		0x014E0000
#define	T_PHY_DATA_LANE3_ENABLE0		0x01500000
#define	T_PHY_DATA_LANE3_ENABLE1		0x01520000

/* TC358768XBG DSI-TX PPI Control */
#define T_TXPPI_LINEINITCNT0			0x02100000
#define T_TXPPI_LINEINITCNT1			0x02120000
#define T_TXPPI_LPTXTIMECNT0			0x02140000
#define T_TXPPI_LPTXTIMECNT1			0x02160000
#define T_TXPPI_TCLK_HEADERCNT0			0x02180000
#define T_TXPPI_TCLK_HEADERCNT1			0x021A0000
#define T_TXPPI_TCLK_TRAILCNT0			0x021C0000
#define T_TXPPI_TCLK_TRAILCNT1			0x021E0000
#define T_TXPPI_THS_HEADERCNT0			0x02200000
#define T_TXPPI_THS_HEADERCNT1			0x02220000
#define T_TXPPI_TWAKEUPCNT0				0x02240000
#define T_TXPPI_TWAKEUPCNT1				0x02260000
#define T_TXPPI_TCLK_POSTCNT0			0x02280000
#define T_TXPPI_TCLK_POSTCNT1			0x022A0000
#define T_TXPPI_THS_TRAILCNT0			0x022C0000
#define T_TXPPI_THS_TRAILCNT1			0x022E0000
#define T_TXPPI_HSTXVREGCNT0			0x02300000
#define T_TXPPI_HSTXVREGCNT1			0x02320000
#define T_TXPPI_HSTXVREGEN_ENABLE0		0x02340000
#define T_TXPPI_HSTXVREGEN_ENABLE1		0x02360000


#define T_TXPPI_BTACNTRL1_0				0x023C0000
#define T_TXPPI_BTACNTRL1_1				0x023E0000
#define T_TXPPI_STARTCNTRL0				0x02040000
#define T_TXPPI_STARTCNTRL1				0x02060000

/* TC358768XBG DSI-TX Timing Control */
#define	T_DSI_SYNC_PULSE_AND_MOD		0x06200000
#define T_DSI_VCONTROL_REGISTER1		0x06220000
#define T_DSI_VCONTROL_REGISTER2		0x06240000
#define T_DSI_VCONTROL_REGISTER3		0x06260000
#define T_DSI_HCONTROL_REGISTER1		0x06280000
#define T_DSI_HCONTROL_REGISTER2		0x062A0000
#define T_DSI_HCONTROL_REGISTER3		0x062C0000
#define T_DSI_START0					0x05180000
#define T_DSI_START1					0x051A0000
#define T_DSI_LANE_SETTING				0x05000000
#define T_DSI_BIT_SETTING				0x05020000
#define	T_DSI_CONFIG_CONTROL_REGISTER	0x00040000

/* DSI clock mode */
#define	T_DSICLK_ENABLE0				0x02380000
#define	T_DSICLK_ENABLE1				0x023A0000

/* Set to HS mode */
#define T_SETHS_LANE_SETTING			0x05000000
#define T_SETHS_BIT_SETTING				0x05020000


/*
*		DCS WRITE
*/
#define DCS_LONG_WRITE				0x06020000 
#define DCS_SHORT_WRITE				0x06020000
#define DSC_WC_SIZE					0x06040000
#define DSC_WC_DATA_0_1				0x06100000
#define DSC_WC_DATA_2_3				0x06120000
#define DSC_WC_DATA_4_5				0x06140000
#define DSC_WC_DATA_6_7				0x06160000
#define DSC_WC_DATA_8_9				0x06180000
#define DSC_WC_START_TX				0x06000000


/* Set to HS mode */
#define T_SETHS_LANE_SETTING			0x05000000
#define T_SETHS_BIT_SETTING				0x05020000


/*--------------------------
* 		DATA
----------------------------*/

/* TC358768XBG Software Reset */
static unsigned int reset[1] = {0x00000001};
static unsigned int reset_release[1] = {0x00000000};

/* TC358768XBG PLL,Clock Setting */
static unsigned int pll_register_val0[1] = {0x0000212B};
static unsigned int pll_register_val1[1] = {0x00000813}; // bareng 8/15 -->813
static unsigned int pll_register_val2[1] = {0x00000813}; // bareng 8/15 -->NOT USED

/* TC358768XBG DPI Input Control */
static unsigned int dpi_register_val0[1] = {0x00000082}; // spec : 01C0 // nVidia : 0082
static unsigned int dpi_register_val1[1] = {0x00000037}; // 37 : RGB888 47: RGB666
static unsigned int dpi_register_val2[1] = {0x0000003E};

/* TC358768XBG D-PHY Setting */
static unsigned int d_phy_register_val[1] = {0x00000000};

/* TC358768XBG DSI-TX PPI Control */
static unsigned int dsi_tx_register_lineinitcnt_val0[1] 	= {0x00000FA0};
static unsigned int dsi_tx_register_lineinitcnt_val1[1] 	= {0x00000000};
static unsigned int dsi_tx_register_lptxtimecnt_val0[1] 	= {0x0000000A};
static unsigned int dsi_tx_register_lptxtimecnt_val1[1] 	= {0x00000000};
static unsigned int dsi_tx_register_tclk_headercnt_val0[1]	= {0x00000503};// changes by TOSHIBA 0815 from 0x00000303 to 0x00000503
static unsigned int dsi_tx_register_tclk_headercnt_val1[1] 	= {0x00000000};
static unsigned int dsi_tx_register_tclk_trailcnt_val0[1] 	= {0x00000000};
static unsigned int dsi_tx_register_tclk_trailcnt_val1[1] 	= {0x00000000};
static unsigned int dsi_tx_register_ths_headercnt_val0[1] 	= {0x00000104};
static unsigned int dsi_tx_register_ths_headercnt_val1[1] 	= {0x00000000};
static unsigned int dsi_tx_register_twakeupcnt_val0[1] 		= {0x00003A98};
static unsigned int dsi_tx_register_twakeupcnt_val1[1] 		= {0x00000000};
static unsigned int dsi_tx_register_tclk_posclk_val0[1] 	= {0x00000010};
static unsigned int dsi_tx_register_tclk_posclk_val1[1] 	= {0x00000000};
static unsigned int dsi_tx_register_ths_trailcnt_val0[1] 	= {0x00000001};
static unsigned int dsi_tx_register_ths_trailcnt_val1[1] 	= {0x00000000};
static unsigned int dsi_tx_register_hstxvregcnt_val0[1] 	= {0x00000005};
static unsigned int dsi_tx_register_hstxvregcnt_val1[1] 	= {0x00000000};
static unsigned int dsi_tx_register_hstxvregcnt_en_val0[1] 	= {0x0000001F};
static unsigned int dsi_tx_register_hstxvregcnt_en_val1[1] 	= {0x00000000};
static unsigned int dsi_tx_register_dsi_clk_en0[1] 			= {0x00000000}; // changes by NVIDIA 0816 from 0x00000001 to 0x00000000
static unsigned int dsi_tx_register_dsi_clk_en1[1] = {0x00000000};
static unsigned int dsi_tx_register_btactrl_val0[1] 		= {0x00000004};
static unsigned int dsi_tx_register_btactrl_val1[1] 		= {0x00000003};
static unsigned int dsi_tx_register_startctrl_val0[1] 		= {0x00000001};
static unsigned int dsi_tx_register_startctrl_val1[1] 		= {0x00000000};

/* TC358768XBG DSI-TX Timing Control */
static unsigned int dsi_tx_timing_sync[1]				={0x00000001};
static unsigned int dsi_tx_timing_vsync0[1]			={0x00000012}; // changes by TOSHIBA 0815 from 0x00000004 to 0x00000012
static unsigned int dsi_tx_timing_vsync1[1]			={0x0000000E};
static unsigned int dsi_tx_timing_vsync2[1]			={0x00000500};
static unsigned int dsi_tx_timing_hsync0[1]			={0x000000F0}; // changes by TOSHIBA 0815 from 0x000000F4 to 0x000000F0
static unsigned int dsi_tx_timing_hsync1[1]			={0x00000119};
static unsigned int dsi_tx_timing_hsync2[1]			={0x00000870};
static unsigned int dsi_tx_timing_dsi_start0[1]		={0x00000001};
static unsigned int dsi_tx_timing_dsi_start1[1]		={0x00000000};
static unsigned int dsi_tx_timing_lane_setting[1]		={0x00000001};
static unsigned int dsi_tx_timing_bit_set[1]			={0x0000A300};
static unsigned int dsi_tx_timing_lp_insert[1]		={0x00008000};
static unsigned int dsi_tx_timing_bit_clear[1]		={0x0000C300};
static unsigned int dsi_tx_timing_conf_ctrl[1]		={0x00000044};

/* DSI clock mode */
static unsigned int dsi_clk_mod0[1]					={0x00000001};
static unsigned int dsi_clk_mod1[1]					={0x00000000};

/* Set to HS mode */
static unsigned int dsi_hs_lane_set0[1]				={0x00000086};
static unsigned int dsi_hs_bit_set0[1]				={0x0000A300};
static unsigned int dsi_hs_lane_set1[1]				={0x00008000};
static unsigned int dsi_hs_bit_set1[1]				={0x0000C300};
static unsigned int dsi_hs_end_of_sequence[1]			={0x00000064};


/*
*		DCS WRITE
*/

static unsigned int dcs_long_write_val[1]		={0x00004039};
static unsigned int dcs_short_write_val[1]	={0x00001015};
static unsigned int dcs_short_write_val_sleepanddisp[1]	={0x00001005};


static unsigned int dcs_wc_size0[1]			={0x00000000};
static unsigned int dcs_wc_size2[1]			={0x00000002};
static unsigned int dcs_wc_size3[1]			={0x00000003};
static unsigned int dcs_wc_size4[1]			={0x00000004};
static unsigned int dcs_wc_size5[1]			={0x00000005}; 
static unsigned int dcs_wc_size6[1]			={0x00000006};
static unsigned int dcs_wc_size8[1]			={0x00000008};
static unsigned int dcs_wc_sizea[1]			={0x0000000A};

static unsigned int dcs_wc_start_tx[1]		={0x00000001};


static unsigned int lgd_power_on_set0[5] = { 0x000043E0 , DSC_WC_DATA_2_3 , 0x00008000 , DSC_WC_DATA_4_5 , 0x00000000 }; 
static unsigned int lgd_power_on_set1[5] = { 0x000014B5 , DSC_WC_DATA_2_3 , 0x00004020 , DSC_WC_DATA_4_5 , 0x00000000 }; 
static unsigned int lgd_power_on_set2[5] = { 0x000001B6 , DSC_WC_DATA_2_3 , 0x00000F16 , DSC_WC_DATA_4_5 , 0x00001316 }; 
static unsigned int lgd_power_on_set3[7] = { 0x000000D0 , DSC_WC_DATA_2_3 , 0x00005630 , DSC_WC_DATA_4_5 , 0x00001201 , DSC_WC_DATA_6_7 , 0x00003404 }; 
static unsigned int lgd_power_on_set4[1] = { 0x00000110 };
static unsigned int lgd_power_on_set5[7] = { 0x000002D1 , DSC_WC_DATA_2_3 , 0x00005651 , DSC_WC_DATA_4_5 , 0x00000806 , DSC_WC_DATA_6_7 , 0x00004700 }; 
static unsigned int lgd_power_on_set6[1] = { 0x00000212 }; 
static unsigned int lgd_power_on_set7[7] = { 0x000000D2 , DSC_WC_DATA_2_3 , 0x00005630 , DSC_WC_DATA_4_5 , 0x00001201 , DSC_WC_DATA_6_7 , 0x00003404 }; 
static unsigned int lgd_power_on_set8[1] = { 0x00000110 };
static unsigned int lgd_power_on_set9[7] = { 0x000002D3 , DSC_WC_DATA_2_3 , 0x00005651 , DSC_WC_DATA_4_5 , 0x00000806 , DSC_WC_DATA_6_7 , 0x00004700 }; 
static unsigned int lgd_power_on_set10[1] = { 0x00000212 };
static unsigned int lgd_power_on_set11[7] = { 0x000000D4 , DSC_WC_DATA_2_3 , 0x00005630 , DSC_WC_DATA_4_5 , 0x00001201 , DSC_WC_DATA_6_7 , 0x00003404 }; 
static unsigned int lgd_power_on_set12[1] = { 0x00000110 };
static unsigned int lgd_power_on_set13[7] = { 0x000002D5 , DSC_WC_DATA_2_3 , 0x00005651 , DSC_WC_DATA_4_5 , 0x00000806 , DSC_WC_DATA_6_7 , 0x00004700 }; 
static unsigned int lgd_power_on_set14[1] = { 0x00000212 }; 
static unsigned int lgd_power_on_set15[1] = { 0x00000770 }; // step 10
static unsigned int lgd_power_on_set16[5] = { 0x00000071 , DSC_WC_DATA_2_3 , 0x00000100 , DSC_WC_DATA_4_5 , 0x00000001 }; // step 11 
static unsigned int lgd_power_on_set17[3] = { 0x00000172 , DSC_WC_DATA_2_3 , 0x0000000F }; // step 12 size 3
static unsigned int lgd_power_on_set18[3] = { 0x00003473 , DSC_WC_DATA_2_3 , 0x00000055 }; // step 13 size 4
static unsigned int lgd_power_on_set19[3] = { 0x00000474 , DSC_WC_DATA_2_3 , 0x00000107 }; // step 14 size 4
static unsigned int lgd_power_on_set20[3] = { 0x00000375 , DSC_WC_DATA_2_3 , 0x0000070F }; // step 15 size 4
static unsigned int lgd_power_on_set21[3] = { 0x00000776 , DSC_WC_DATA_2_3 , 0x00000500 }; // step 16 size 4




static unsigned int lgd_power_supply_set0[3] = { 0x000001C0 , DSC_WC_DATA_2_3 , 0x00000008 };
static unsigned int lgd_power_supply_set1[7] = { 0x000000C3 , DSC_WC_DATA_2_3 , 0x00000008 , DSC_WC_DATA_4_5 , 0x00000000 , DSC_WC_DATA_6_7 , 0x00008867 };
static unsigned int lgd_power_supply_set2[1] = { 0x00000232 };
static unsigned int lgd_power_supply_set3[5] = { 0x000022C4 , DSC_WC_DATA_2_3 , 0x00001924 , DSC_WC_DATA_4_5 , 0x00004119 };
static unsigned int lgd_power_supply_set4[1] = { 0x000000F9 };
static unsigned int lgd_power_supply_set5[1] = { 0x000000C1 };
static unsigned int lgd_power_supply_set6[1] = { 0x000002C2 };
static unsigned int lgd_power_supply_set7[1] = { 0x000006C2 };
static unsigned int lgd_power_supply_set8[1] = { 0x00004EC2 };



static unsigned int lgd_sleep_out_set[1] = { 0x00000011 };
static unsigned int lgd_disp_on_set[1] = { 0x00000029 };


/* End of panel enable set */
static unsigned int dsi_hs_lane_set[1]				={0x00000086};
static unsigned int dsi_hs_bit_set[1]					={0x0000A300};

static unsigned int end_of_panel_enable_set0[1]				={0x00000087}; // spec : 86 // nVidia : 87
static unsigned int end_of_panel_enable_set1[1]					={0x0000A300};
static unsigned int end_of_panel_enable_set2[1]				={0x00008000};
static unsigned int end_of_panel_enable_set3[1]					={0x0000C300};
static unsigned int end_of_panel_enable_set4[1]				={0x00000044}; //Changes by TOSHIBA 0815 from 0x00000064 to 0x00000044


// lane setting 
// case  0000 0000 0000 0001

// case  0000 0000 1000 0110 

static unsigned int test_set0[1]				={0x00000087}; // spec : 86 // nVidia : 87
static unsigned int test_set1[1]					={0x0000A300};
static unsigned int test_set2[1]				={0x00000086}; 
static unsigned int test_set3[1]					={0x0000A300};
static unsigned int test_set4[1]					={0x00000044};



/*--------------------------------------------------------------------------*/
#if 0

static struct spi_cmd_data init_toshiba_bridge_sequence[] = {
	{  T_RESET_SYSCTL, 					1, 1,  		reset },
	{  T_RESET_SYSCTL, 					0, 1,  		reset_release},
	{  T_PLL_CONTROL_REGISTER, 			0, 1, 	pll_register_val0},
	{  T_PLL_OSC_ENABLE, 				0, 1,  	pll_register_val1},
	{  T_PLL_CLK_OUT_ENABLE, 			0, 1,  	pll_register_val2},
	{  T_DPI_FIFO_CONTROL_REGISTER, 	0, 1,  	dpi_register_val0},
	{  T_TX_FORMAT_SETTING, 			0, 1,  	dpi_register_val1},
	{  T_TX_PIXEL_STREAM_PACKET_DTYPE,	0, 1,  	dpi_register_val2},	
	{  T_PHY_CLK_LANE_ENABLE0,			0, 1,  d_phy_register_val}, 
	{  T_PHY_CLK_LANE_ENABLE1, 			0, 1,  d_phy_register_val},
	{  T_PHY_DATA_LANE0_ENABLE0,		0, 1,  d_phy_register_val},
	{  T_PHY_DATA_LANE0_ENABLE1, 		0, 1,  d_phy_register_val},
	{  T_PHY_DATA_LANE1_ENABLE0, 		0, 1,  d_phy_register_val},
	{  T_PHY_DATA_LANE1_ENABLE1, 		0, 1,  d_phy_register_val},
	{  T_PHY_DATA_LANE2_ENABLE0,		0, 1,  d_phy_register_val}, 
	{  T_PHY_DATA_LANE2_ENABLE1, 		0, 1,  d_phy_register_val},
	{  T_PHY_DATA_LANE3_ENABLE0,		0, 1,  d_phy_register_val},
	{  T_PHY_DATA_LANE3_ENABLE1, 		0, 1,  d_phy_register_val},
	{  T_TXPPI_LINEINITCNT0, 			0, 1,  dsi_tx_register_lineinitcnt_val0},
	{  T_TXPPI_LINEINITCNT1, 			0, 1,  dsi_tx_register_lineinitcnt_val1},
	{  T_TXPPI_LPTXTIMECNT0, 			0, 1,  dsi_tx_register_lptxtimecnt_val0},
	{  T_TXPPI_LPTXTIMECNT1, 			0, 1,  dsi_tx_register_lptxtimecnt_val1},
	{  T_TXPPI_TCLK_HEADERCNT0, 		0, 1,  dsi_tx_register_tclk_headercnt_val0},
	{  T_TXPPI_TCLK_HEADERCNT1, 		0, 1,  dsi_tx_register_tclk_headercnt_val1},
	{  T_TXPPI_TCLK_TRAILCNT0, 			0, 1,  dsi_tx_register_tclk_trailcnt_val0},
	{  T_TXPPI_TCLK_TRAILCNT1, 			0, 1,  dsi_tx_register_tclk_trailcnt_val1},
	{  T_TXPPI_THS_HEADERCNT0, 			0, 1,  dsi_tx_register_ths_headercnt_val0},
	{  T_TXPPI_THS_HEADERCNT1, 			0, 1,  dsi_tx_register_ths_headercnt_val1},
	{  T_TXPPI_TWAKEUPCNT0, 			0, 1,  dsi_tx_register_twakeupcnt_val0},
	{  T_TXPPI_TWAKEUPCNT1, 			0, 1,  dsi_tx_register_twakeupcnt_val1},
	{  T_TXPPI_TCLK_POSTCNT0, 			0, 1,  dsi_tx_register_tclk_posclk_val0},
	{  T_TXPPI_TCLK_POSTCNT1, 			0, 1,  dsi_tx_register_tclk_posclk_val1},
	{  T_TXPPI_THS_TRAILCNT0, 			0, 1,  dsi_tx_register_ths_trailcnt_val0},
	{  T_TXPPI_THS_TRAILCNT1, 			0, 1,  dsi_tx_register_ths_trailcnt_val1},
	{  T_TXPPI_HSTXVREGCNT0, 			0, 1,  dsi_tx_register_hstxvregcnt_val0},
	{  T_TXPPI_HSTXVREGCNT1, 			0, 1,  dsi_tx_register_hstxvregcnt_val1},
	{  T_TXPPI_HSTXVREGEN_ENABLE0, 		0, 1,  dsi_tx_register_hstxvregcnt_en_val0},
	{  T_TXPPI_HSTXVREGEN_ENABLE1, 		0, 1,  dsi_tx_register_hstxvregcnt_en_val1},
	{  T_DSICLK_ENABLE0, 				0, 1,  dsi_tx_register_dsi_clk_en0},
	{  T_DSICLK_ENABLE1, 				0, 1,  dsi_tx_register_dsi_clk_en1},
	{  T_TXPPI_BTACNTRL1_0, 			0, 1,  dsi_tx_register_btactrl_val0},
	{  T_TXPPI_BTACNTRL1_1, 			0, 1,  dsi_tx_register_btactrl_val1},
	{  T_TXPPI_STARTCNTRL0, 			0, 1,  dsi_tx_register_startctrl_val0},
	{  T_TXPPI_STARTCNTRL1, 			0, 1,  dsi_tx_register_startctrl_val1},
	{  T_DSI_SYNC_PULSE_AND_MOD, 		0, 1,  dsi_tx_timing_sync}, // dis
	{  T_DSI_VCONTROL_REGISTER1, 		0, 1,  dsi_tx_timing_vsync0},
	{  T_DSI_VCONTROL_REGISTER2, 		0, 1,  dsi_tx_timing_vsync1},
	{  T_DSI_VCONTROL_REGISTER3, 		0, 1,  dsi_tx_timing_vsync2},
	{  T_DSI_HCONTROL_REGISTER1, 		0, 1,  dsi_tx_timing_hsync0},
	{  T_DSI_HCONTROL_REGISTER2, 		0, 1,  dsi_tx_register_hstxvregcnt_val0},
	{  T_DSI_HCONTROL_REGISTER3, 		0, 1,  dsi_tx_timing_hsync2},
	{  T_DSI_START0, 					0, 1,  dsi_tx_timing_dsi_start0},
	{  T_DSI_START1, 					0, 1,  dsi_tx_timing_dsi_start1},
	{  T_DSI_LANE_SETTING, 				0, 1,  dsi_tx_timing_lane_setting},
	{  T_DSI_BIT_SETTING, 				0, 1,  dsi_tx_timing_bit_set},
	{  T_DSI_LANE_SETTING, 				0, 1,  dsi_tx_register_btactrl_val0},
	{  T_DSI_BIT_SETTING, 				0, 1,  dsi_tx_register_btactrl_val1},
	{  T_DSI_CONFIG_CONTROL_REGISTER, 	0, 1,  dsi_tx_timing_conf_ctrl},
};

#else


static struct spi_cmd_data init_toshiba_bridge_sequence[] = {
	{  T_RESET_SYSCTL, 					1, 1,  		reset },   			/* step 1 */
	{  T_RESET_SYSCTL, 					0, 1,  		reset_release},		/* step 2 */
	{  T_PLL_CONTROL_REGISTER, 			0, 1, 	pll_register_val0},		/* step 3 */
	{  T_PLL_OSC_ENABLE, 				0, 1,  	pll_register_val1},		/* step 4 */
//	{  T_PLL_CLK_OUT_ENABLE, 			0, 1,  	pll_register_val2}, 
	{  T_DPI_FIFO_CONTROL_REGISTER, 	0, 1,  	dpi_register_val0},		/* step 5 */
	{  T_TX_FORMAT_SETTING, 			0, 1,  	dpi_register_val1},		/* step 6 */
	{  T_TX_PIXEL_STREAM_PACKET_DTYPE,	0, 1,  	dpi_register_val2},		/* step 7 */
	{  T_PHY_CLK_LANE_ENABLE0,			0, 1,  d_phy_register_val}, 	/* step 8 */
	{  T_PHY_CLK_LANE_ENABLE1, 			0, 1,  d_phy_register_val},		
	{  T_PHY_DATA_LANE0_ENABLE0,		0, 1,  d_phy_register_val},		/* step 9 */
	{  T_PHY_DATA_LANE0_ENABLE1, 		0, 1,  d_phy_register_val},
	{  T_PHY_DATA_LANE1_ENABLE0, 		0, 1,  d_phy_register_val},		/* step 10 */
	{  T_PHY_DATA_LANE1_ENABLE1, 		0, 1,  d_phy_register_val},
	{  T_PHY_DATA_LANE2_ENABLE0,		0, 1,  d_phy_register_val}, 	/* step 11 */
	{  T_PHY_DATA_LANE2_ENABLE1, 		0, 1,  d_phy_register_val},
	{  T_PHY_DATA_LANE3_ENABLE0,		0, 1,  d_phy_register_val},		/* step 12 */
	{  T_PHY_DATA_LANE3_ENABLE1, 		0, 1,  d_phy_register_val},
//	{  T_TXPPI_LINEINITCNT0, 			0, sizeof(dsi_tx_register_lineinitcnt_val0),  dsi_tx_register_lineinitcnt_val0},
//	{  T_TXPPI_LINEINITCNT1, 			0, sizeof(dsi_tx_register_lineinitcnt_val1),  dsi_tx_register_lineinitcnt_val1},
	{  T_TXPPI_LPTXTIMECNT0, 			0, 1,  dsi_tx_register_lptxtimecnt_val0},	/* step 13 LSB */ 
	{  T_TXPPI_LPTXTIMECNT1, 			0, 1,  dsi_tx_register_lptxtimecnt_val1},	/*         MSB */
	{  T_TXPPI_TCLK_HEADERCNT0, 		0, 1,  dsi_tx_register_tclk_headercnt_val0},	/* step 14 */
	{  T_TXPPI_TCLK_HEADERCNT1, 		0, 1,  dsi_tx_register_tclk_headercnt_val1},	
//	{  T_TXPPI_TCLK_TRAILCNT0, 			0, sizeof(dsi_tx_register_tclk_trailcnt_val0),  dsi_tx_register_tclk_trailcnt_val0},
//	{  T_TXPPI_TCLK_TRAILCNT1, 			0, sizeof(dsi_tx_register_tclk_trailcnt_val1),  dsi_tx_register_tclk_trailcnt_val1},
	{  T_TXPPI_THS_HEADERCNT0, 			0, 1,  dsi_tx_register_ths_headercnt_val0},
	{  T_TXPPI_THS_HEADERCNT1, 			0, 1,  dsi_tx_register_ths_headercnt_val1}, // changed by NVIDIA 0816 from disable to enable
//	{  T_TXPPI_TWAKEUPCNT0, 			0, sizeof(dsi_tx_register_twakeupcnt_val0),  dsi_tx_register_twakeupcnt_val0},
//	{  T_TXPPI_TWAKEUPCNT1, 			0, sizeof(dsi_tx_register_twakeupcnt_val1),  dsi_tx_register_twakeupcnt_val1},
	{  T_TXPPI_TCLK_POSTCNT0, 			0, 1,  dsi_tx_register_tclk_posclk_val0},		/* step 15 */
	{  T_TXPPI_TCLK_POSTCNT1, 			0, 1,  dsi_tx_register_tclk_posclk_val1},
//	{  T_TXPPI_THS_TRAILCNT0, 			0, sizeof(dsi_tx_register_ths_trailcnt_val0),  dsi_tx_register_ths_trailcnt_val0},
//	{  T_TXPPI_THS_TRAILCNT1, 			0, sizeof(dsi_tx_register_ths_trailcnt_val1),  dsi_tx_register_ths_trailcnt_val1},
//	{  T_TXPPI_HSTXVREGCNT0, 			0, sizeof(dsi_tx_register_hstxvregcnt_val0),  dsi_tx_register_hstxvregcnt_val0},
//	{  T_TXPPI_HSTXVREGCNT1, 			0, sizeof(dsi_tx_register_hstxvregcnt_val1),  dsi_tx_register_hstxvregcnt_val1},
	{  T_TXPPI_HSTXVREGEN_ENABLE0, 		0, 1,  dsi_tx_register_hstxvregcnt_en_val0},	/* step 16 */
	{  T_TXPPI_HSTXVREGEN_ENABLE1, 		0, 1,  dsi_tx_register_hstxvregcnt_en_val1},
	{  T_DSICLK_ENABLE0, 				0, 1,  dsi_tx_register_dsi_clk_en0},			/* step 17 */
	{  T_DSICLK_ENABLE1, 				0, 1,  dsi_tx_register_dsi_clk_en1},
//	{  T_TXPPI_BTACNTRL1_0, 			0, sizeof(dsi_tx_register_btactrl_val0),  dsi_tx_register_btactrl_val0},
//	{  T_TXPPI_BTACNTRL1_1, 			0, sizeof(dsi_tx_register_btactrl_val1),  dsi_tx_register_btactrl_val1},
	{  T_TXPPI_STARTCNTRL0, 			0, 1,  dsi_tx_register_startctrl_val0},			/* step 18 */
	{  T_TXPPI_STARTCNTRL1, 			0, 1,  dsi_tx_register_startctrl_val1},

	/* DSI Tx Control*/
	{  T_DSI_SYNC_PULSE_AND_MOD, 		0, 1,  dsi_tx_timing_sync}, // dis				/* step 19 */
	{  T_DSI_VCONTROL_REGISTER1, 		0, 1,  dsi_tx_timing_vsync0},					/* step 20 */
	{  T_DSI_VCONTROL_REGISTER2, 		0, 1,  dsi_tx_timing_vsync1},					/* step 21 */ // changes by TOSHIBA 0815 from enable to comment 
	{  T_DSI_VCONTROL_REGISTER3, 		0, 1,  dsi_tx_timing_vsync2},					/* step 22 */
	{  T_DSI_HCONTROL_REGISTER1, 		0, 1,  dsi_tx_timing_hsync0},					/* step 23 */
	{  T_DSI_HCONTROL_REGISTER2, 		0, 1,  dsi_tx_timing_hsync1},/* step 24  SKIP by TOSHIBA*/
	{  T_DSI_HCONTROL_REGISTER3, 		0, 1,  dsi_tx_timing_hsync2},					/* step 25 */

	/* Set LP Mode */

	{  T_DSI_START0, 					0, 1,  dsi_tx_timing_dsi_start0},				/* step 26 */
	{  T_DSI_START1, 					0, 1,  dsi_tx_timing_dsi_start1},

	{  T_DSI_LANE_SETTING, 				0, 1,  dsi_tx_timing_lane_setting},				
	{  T_DSI_BIT_SETTING, 				0, 1,  dsi_tx_timing_bit_set},
//	{  T_DSI_LANE_SETTING, 				0, sizeof(dsi_tx_timing_lp_insert),  dsi_tx_register_btactrl_val0},
//	{  T_DSI_BIT_SETTING, 				0, sizeof(dsi_tx_timing_bit_clear),  dsi_tx_register_btactrl_val1},
//	{  T_DSI_CONFIG_CONTROL_REGISTER, 	0, sizeof(dsi_tx_timing_conf_ctrl),  dsi_tx_timing_conf_ctrl},
};

#endif

static struct spi_cmd_data toshiba_lgd_power_on_set[] = {
	{  DCS_LONG_WRITE, 					0, 1,  				dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,  				dcs_wc_size6},
	{  DSC_WC_DATA_0_1, 				0, 5,  				lgd_power_on_set0},
	{  DSC_WC_START_TX, 				0, 1,  				dcs_wc_start_tx},
	// step 2
	{  DCS_LONG_WRITE, 					0, 1,  				dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,  				dcs_wc_size6},
	{  DSC_WC_DATA_0_1, 				0, 5,  				lgd_power_on_set1},
	{  DSC_WC_START_TX, 				0, 1,  				dcs_wc_start_tx},
	// step 3
	{  DCS_LONG_WRITE, 					0, 1,  				dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,  				dcs_wc_size6},
	{  DSC_WC_DATA_0_1, 				0, 5,  				lgd_power_on_set2},
	{  DSC_WC_START_TX, 				0, 1,  				dcs_wc_start_tx},
	// step 4
	{  DCS_LONG_WRITE, 					0, 1,  				dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,  				dcs_wc_size8},
	{  DSC_WC_DATA_0_1, 				0, 7,  				lgd_power_on_set3},
	{  DSC_WC_START_TX, 				0, 1,  				dcs_wc_start_tx},
	// setp 4 split
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size2},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_on_set4},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// step 5
	{  DCS_LONG_WRITE, 					0, 1,  				dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,  				dcs_wc_size8},
	{  DSC_WC_DATA_0_1, 				0, 7,  				lgd_power_on_set5},
	{  DSC_WC_START_TX, 				0, 1,  				dcs_wc_start_tx},
	// step 5 split
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size2},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_on_set6},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// step 6 
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size8},
	{  DSC_WC_DATA_0_1, 				0, 7,				lgd_power_on_set7},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// step 6 split
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size2},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_on_set8},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// step 7 
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size8},
	{  DSC_WC_DATA_0_1, 				0, 7,				lgd_power_on_set9},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// step 7 split
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size2},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_on_set10},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// step 8 
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size8},
	{  DSC_WC_DATA_0_1, 				0, 7,				lgd_power_on_set11},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// step 8 split
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size2},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_on_set12},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// step 9 
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size8},
	{  DSC_WC_DATA_0_1, 				0, 7,				lgd_power_on_set13},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// step 9 split
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size2},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_on_set14},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},

	// step 10
	{  DCS_SHORT_WRITE,					0, 1,					dcs_short_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size0},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_on_set15},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	
	// step 11
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size5},
	{  DSC_WC_DATA_0_1, 				0, 5,				lgd_power_on_set16},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	
	// step 12
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size3},
	{  DSC_WC_DATA_0_1, 				0, 3,				lgd_power_on_set17},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	
	// step 13
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size4},
	{  DSC_WC_DATA_0_1, 				0, 3,				lgd_power_on_set18},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	
	// step 14
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size4},
	{  DSC_WC_DATA_0_1, 				0, 3,				lgd_power_on_set19},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	
	// step 15
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size4},
	{  DSC_WC_DATA_0_1, 				0, 3,				lgd_power_on_set20},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	
	// step 16
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size4},
	{  DSC_WC_DATA_0_1, 				0, 3,				lgd_power_on_set21},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	
};


static char set_address_mode [1] 	= {0x0036};
/* ----- set_address_mode -----
	* [D7] Page Address Order   	: 0 >> Top to Bottom. 	1 >> Bottom to Top
	* [D6] Column Address Order 	: 0 >> Left to Right. 	1 >> Right to Left.
	* [D3] RGB Order 				: 0 >> RGB  			1 >> BGR
*/

static char set_pixel_format [1] 	= {0x603A}; 
/* ----- set_pixel_format -----
    * 18 bits/pixel : 0x60
	* 24 bits/pixel : 0x70
*/



static struct spi_cmd_data toshiba_hitachi_power_on_set[] = {
	{  DCS_SHORT_WRITE,					0, 1,					dcs_short_write_val_sleepanddisp},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size0},
	{  DSC_WC_DATA_0_1, 				0, 1,				set_address_mode},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	
	{  DCS_SHORT_WRITE,					0, 1,					dcs_short_write_val_sleepanddisp},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size0},
	{  DSC_WC_DATA_0_1, 				0, 1,				set_pixel_format},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	
};


static struct spi_cmd_data toshiba_lgd_power_supply_set[] = {
	// step 1
	{  DCS_LONG_WRITE, 					0, 1,  				dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,  				dcs_wc_size3},
	{  DSC_WC_DATA_0_1, 				0, 3,  				lgd_power_supply_set0},
	{  DSC_WC_START_TX, 				0, 1,  				dcs_wc_start_tx},
	// step 2
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size8},
	{  DSC_WC_DATA_0_1, 				0, 7,				lgd_power_supply_set1},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// setp 2 split
	{  DCS_LONG_WRITE,					0, 1,					dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size2},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_supply_set2},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
	// step 3
	{  DCS_LONG_WRITE, 					0, 1,  				dcs_long_write_val},
	{  DSC_WC_SIZE, 					0, 1,  				dcs_wc_size6},
	{  DSC_WC_DATA_0_1, 				0, 5,  				lgd_power_supply_set3},
	{  DSC_WC_START_TX, 				0, 1,  				dcs_wc_start_tx},
	// step 4
	{  DCS_SHORT_WRITE,					0, 1,					dcs_short_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size0},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_supply_set4},
	{  DSC_WC_START_TX, 				10, 1, 				dcs_wc_start_tx},
	// Manual Power step 1
	{  DCS_SHORT_WRITE,					0, 1,					dcs_short_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size0},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_supply_set5},
	{  DSC_WC_START_TX, 				20, 1, 				dcs_wc_start_tx},
	// Manual Power step 2
	{  DCS_SHORT_WRITE,					0, 1,					dcs_short_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size0},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_supply_set6},
	{  DSC_WC_START_TX, 				20, 1, 				dcs_wc_start_tx},
	// Manual Power step 3
	{  DCS_SHORT_WRITE,					0, 1,					dcs_short_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size0},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_supply_set7},
	{  DSC_WC_START_TX, 				20, 1, 				dcs_wc_start_tx},
	// Manual Power step 4
	{  DCS_SHORT_WRITE,					0, 1,					dcs_short_write_val},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size0},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_power_supply_set8},
	{  DSC_WC_START_TX, 				20, 1, 				dcs_wc_start_tx},

};


static struct spi_cmd_data toshiba_lgd_sleep_out_set[] = {
	{  DCS_SHORT_WRITE,					0, 1,					dcs_short_write_val_sleepanddisp},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size0},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_sleep_out_set},
	{  DSC_WC_START_TX, 				150, 1, 				dcs_wc_start_tx},
	
	{  DCS_SHORT_WRITE,					0, 1,					dcs_short_write_val_sleepanddisp},
	{  DSC_WC_SIZE, 					0, 1,				dcs_wc_size0},
	{  DSC_WC_DATA_0_1, 				0, 1,				lgd_disp_on_set},
	{  DSC_WC_START_TX, 				0, 1, 				dcs_wc_start_tx},
};

#if 1
/* nVidia */
static struct spi_cmd_data toshiba_end_of_init_sequence[] = {
	{  T_SETHS_LANE_SETTING,					0, 1,					end_of_panel_enable_set0},
	{  T_SETHS_BIT_SETTING, 					1, 1,				end_of_panel_enable_set1},
	{  T_SETHS_LANE_SETTING,					0, 1, 				end_of_panel_enable_set2},
	{  T_SETHS_BIT_SETTING, 					0, 1,				end_of_panel_enable_set3},	
	{  T_DSI_CONFIG_CONTROL_REGISTER,					100, 1,					end_of_panel_enable_set4},
};

#else
/* spec */
static struct spi_cmd_data toshiba_end_of_init_sequence[] = {

	{  T_SETHS_LANE_SETTING,					0, 1,					test_set0},
	{  T_SETHS_BIT_SETTING, 					0, 1,				test_set1},
	{  T_SETHS_LANE_SETTING,					0, 1, 				test_set2},
	{  T_SETHS_BIT_SETTING, 					0, 1,				test_set3},	
	{  T_DSI_CONFIG_CONTROL_REGISTER,					100, 1,					test_set4},
};

// 86 // 1C0 // send only 2 --> boot white on
#endif

