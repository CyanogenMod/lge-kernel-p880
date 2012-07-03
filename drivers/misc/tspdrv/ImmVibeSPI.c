/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS 1
#define ISA1200_I2C_ADDRESS 0x49 /*0x92 when SADD is high*/
#define SCTRL         (0)     /* 0x0F, System(LDO) Register Group 0*/
#define HCTRL0     (0x30)     /* 0x09 */ /* Haptic Motor Driver Control Register Group 0*/
#define HCTRL1     (0x31)     /* 0x4B */ /* Haptic Motor Driver Control Register Group 1*/
#define HCTRL2     (0x32)     /* 0x00*/ /* Haptic Motor Driver Control Register Group 2*/
#define HCTRL3     (0x33)     /* 0x13 */ /* Haptic Motor Driver Control Register Group 3*/
#define HCTRL4     (0x34)     /* 0x00 */ /* Haptic Motor Driver Control Register Group 4*/
#define HCTRL5     (0x35)     /* 0x6B */ /* Haptic Motor Driver Control Register Group 5*/
#define HCTRL6     (0x36)     /* 0xD6 */ /* Haptic Motor Driver Control Register Group 6*/
#define HCTRL7     (0x37)     /* 0x00 */ /* Haptic Motor Driver Control Register Group 7*/
#define HCTRL8     (0x38)     /* 0x00 */ /* Haptic Motor Driver Control Register Group 8*/
#define HCTRL9     (0x39)     /* 0x40 */ /* Haptic Motor Driver Control Register Group 9*/
#define HCTRLA     (0x3A)     /* 0x2C */ /* Haptic Motor Driver Control Register Group A*/
#define HCTRLB     (0x3B)     /* 0x6B */ /* Haptic Motor Driver Control Register Group B*/
#define HCTRLC     (0x3C)     /* 0xD6 */ /* Haptic Motor Driver Control Register Group C*/
#define HCTRLD     (0x3D)     /* 0x19 */ /* Haptic Motor Driver Control Register Group D*/

#define LDO_VOLTAGE_23V 0x08
#define LDO_VOLTAGE_24V 0x09
#define LDO_VOLTAGE_25V 0x0A
#define LDO_VOLTAGE_26V 0x0B
#define LDO_VOLTAGE_27V 0x0C
#define LDO_VOLTAGE_28V 0x0D
#define LDO_VOLTAGE_29V 0x0E
#define LDO_VOLTAGE_30V 0x0F
#define LDO_VOLTAGE_31V 0x00
#define LDO_VOLTAGE_32V 0x01
#define LDO_VOLTAGE_33V 0x02
#define LDO_VOLTAGE_34V 0x03
#define LDO_VOLTAGE_35V 0x04
#define LDO_VOLTAGE_36V 0x05
#define LDO_VOLTAGE_37V 0x06
#define LDO_VOLTAGE_38V 0x07

static bool g_bAmpEnabled = false;
static int status = 0;

//#define ISA1200_HEN_ENABLE //activate if LEN/HEN is seperate
#define RETRY_CNT 0

#define SYS_API_LEN_HIGH tspdrv_control_vibrator(1);	// todo - define to something
#define SYS_API_LEN_LOW tspdrv_control_vibrator(0);	// todo - define to something
#define SYS_API_HEN_HIGH tspdrv_control_vibrator(1);	// todo - define to something
#define SYS_API_HEN_LOW tspdrv_control_vibrator(0);	// todo - define to something
#define SYS_API_VDDP_ON // todo - define to something
#define SYS_API_VDDP_OFF // todo - define to something
#define SLEEP(_ms_time) msleep(_ms_time)// todo - define to something

#define DEBUG_MSG //printk	// todo - define to something

#define PWM_PERIOD_DEFAULT              44000 //20.3KHz
//#define PWM_DUTY_DEFAULT              (PWM_PERIOD_DEFAULT >> 1) //50%
#define PWM_DUTY_DEFAULT              (PWM_PERIOD_DEFAULT *.75 ) //75%

VibeUInt32 g_nPWM_Freq = PWM_PERIOD_DEFAULT;

#define PWM_CLK_ENABLE tspdrv_control_pwm(1, PWM_DUTY_DEFAULT, PWM_PERIOD_DEFAULT);	// todo - define to something
#define PWM_CLK_DISABLE tspdrv_control_pwm(0, 0, 0);// todo - define to something


#define LDO_VOLTAGE_DEFAULT LDO_VOLTAGE_23V
VibeUInt32 g_nLDO_Voltage = LDO_VOLTAGE_DEFAULT;


IMMVIBESPIAPI VibeStatus SYS_API__I2C__Write( _addr, _data)
{
	return tspdrv_i2c_write_byte_data(_addr, _data);
}

/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
    int cnt = 0;
    unsigned char I2C_data[1];
    int ret = VIBE_S_SUCCESS;

    if (g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable.\n"));

        g_bAmpEnabled = false;


#ifdef ISA1200_HEN_ENABLE
	    SYS_API_HEN_LOW;
#endif	

	    I2C_data[0] = 0x08; // Haptic drive disable
	    do
	    {
	        ret = SYS_API__I2C__Write(HCTRL0,  I2C_data[0]);
	        cnt++;
	    }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
	    if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpDisable] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", HCTRL0, ret);	

	    PWM_CLK_DISABLE;

    }

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force0)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    int cnt = 0;	
    unsigned char I2C_data[1];
    int ret = VIBE_S_SUCCESS;
	int fd = 0;

	mm_segment_t oldfs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open("/sdcard/vib.txt", O_RDONLY , 0755);


    if (!g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpEnable.\n"));
        g_bAmpEnabled = true;
		

	    PWM_CLK_ENABLE;

#ifdef ISA1200_HEN_ENABLE
	    SYS_API_HEN_HIGH;
#endif

		if(fd >= 0) //ELT Test Mode
		{
			DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] Case : ELT Test Vibration\n");
			if(status == 0){
				DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] ELT : Register Setting \n");
				I2C_data[0] = LDO_VOLTAGE_30V; // LDO Voltage : 3.0V
				do
				{
					ret = SYS_API__I2C__Write(SCTRL,  I2C_data[0]);
					cnt++;
				}while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
				if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);
				          
				I2C_data[0] = 0x93; //224Hz
				do{
				    ret = SYS_API__I2C__Write(HCTRL4,  I2C_data[0]);
				    cnt++;
				}while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
				if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);
				      
				I2C_data[0] = tspdrv_i2c_read_byte_data(SCTRL);
				DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] ELT HCTRL0 written data : 0x%x\n", I2C_data[0]);

				I2C_data[0] = tspdrv_i2c_read_byte_data(HCTRL4);
				DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] ELT HCTRL1 written data : 0x%x\n", I2C_data[0]);
				
				status = 1;
			}
			else{ //Normal Mode
				DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] ELT : just vibrate \n");

			}
		}
		else{
			DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] Case : Normal Vibration\n");
            if(status == 1){
                DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] Normal : Register Setting \n");
                I2C_data[0] = g_nLDO_Voltage; // LDO Voltage : 2.7V
                do
                {
                    ret = SYS_API__I2C__Write(SCTRL,  I2C_data[0]);
                    cnt++;
                }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
                if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);

                I2C_data[0] = 0x94; //230Hz
                do{
                    ret = SYS_API__I2C__Write(HCTRL4,  I2C_data[0]);
                    cnt++;
                }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
                if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);

                I2C_data[0] = tspdrv_i2c_read_byte_data(SCTRL);
                DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] Normal HCTRL0 written data : 0x%x\n", I2C_data[0]);

                I2C_data[0] = tspdrv_i2c_read_byte_data(HCTRL4);
                DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] Normal HCTRL1 written data : 0x%x\n", I2C_data[0]);

				status =0;
            }
            else{
                DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] Normal : just vibrate \n");

            }


		}

		
		

	    I2C_data[0] = 0x88; // Haptic Drive Enable + PWM Input mode
	    do
	    {
	        ret = SYS_API__I2C__Write(HCTRL0, I2C_data[0]);
	        cnt++;
	    }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
	    if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_AmpEnable] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);	
#if 0
		I2C_data[0] = tspdrv_i2c_read_byte_data(HCTRL0);
		DEBUG_MSG("HCTRL0 written data : 0x%x\n", I2C_data[0]);

		I2C_data[0] = tspdrv_i2c_read_byte_data(HCTRL1);
		DEBUG_MSG("HCTRL1 written data : 0x%x\n", I2C_data[0]);

		I2C_data[0] = tspdrv_i2c_read_byte_data(HCTRL2);
		DEBUG_MSG("HCTRL2 written data : 0x%x\n", I2C_data[0]);

		I2C_data[0] = tspdrv_i2c_read_byte_data(HCTRL3);
		DEBUG_MSG("HCTRL3 written data : 0x%x\n", I2C_data[0]);

		I2C_data[0] = tspdrv_i2c_read_byte_data(HCTRL4);
		DEBUG_MSG("HCTRL4 written data : 0x%x\n", I2C_data[0]);
#endif
    }

	sys_close(fd);
	set_fs(oldfs);

    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM frequencies, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
    int cnt = 0;	
    unsigned char I2C_data[1];
    int ret = VIBE_S_SUCCESS;

    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Initialize.\n"));

    SYS_API_VDDP_ON;
    SYS_API_LEN_HIGH;

    SLEEP(20 /*ms*/);

    I2C_data[0] = g_nLDO_Voltage; // LDO Voltage
    do
    {
        ret = SYS_API__I2C__Write(SCTRL,  I2C_data[0]);
        cnt++;
    }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
    if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);	

    I2C_data[0] = 0x08; // Haptic Drive Disable + PWM Input mode
    do
    {
        ret = SYS_API__I2C__Write(HCTRL0,  I2C_data[0]);
        cnt++;
    }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
    if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);	

    I2C_data[0] = 0x40; // EXT clock + DAC inversion + LRA
    do
    {
        ret = SYS_API__I2C__Write(HCTRL1,  I2C_data[0]);
        cnt++;
    }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
    if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);	

    I2C_data[0] = 0x00; // Disable Software Reset
    do
    {
        ret = SYS_API__I2C__Write(HCTRL2,  I2C_data[0]);
        cnt++;
    }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
    if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);	

    I2C_data[0] = 0x13; // Disable Software Reset
    do
    {
        ret = SYS_API__I2C__Write(HCTRL3,  I2C_data[0]);
        cnt++;
    }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
    if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);	


    I2C_data[0] = 0x93; //20.3KHz -> 229.xHz
    do
    {
        ret = SYS_API__I2C__Write(HCTRL4,  I2C_data[0]);
        cnt++;
    }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
    if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);	

    I2C_data[0] = 0x00;
    do
    {
        ret = SYS_API__I2C__Write(HCTRL5,  I2C_data[0]);
        cnt++;
    }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
    if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);	

    I2C_data[0] = 0x00;
    do
    {
        ret = SYS_API__I2C__Write(HCTRL6,  I2C_data[0]);
        cnt++;
    }while(VIBE_S_SUCCESS != ret && cnt < RETRY_CNT);
    if( VIBE_S_SUCCESS != ret) DEBUG_MSG("[ImmVibeSPI_ForceOut_Initialize] I2C_Write Error,  Slave Address = [%d], ret = [%d]\n", I2C_data[0], ret);	

    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Terminate.\n"));
    SYS_API_LEN_LOW;
#ifdef ISA1200_HEN_ENABLE        
    SYS_API_HEN_LOW;
#endif	
    SYS_API_VDDP_OFF;
    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM_MAG duty cycle
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    VibeInt8 nForce;
    int ret = VIBE_S_SUCCESS;
	int duty_ns;

    switch (nOutputSignalBitDepth)
    {
        case 8:
            /* pForceOutputBuffer is expected to contain 1 byte */
            if (nBufferSizeInBytes != 1)
			{
				DbgOut((KERN_ERR "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes ));
				return VIBE_E_FAIL;
            }
            nForce = pForceOutputBuffer[0];
            break;
        case 16:
            /* pForceOutputBuffer is expected to contain 2 byte */
            if (nBufferSizeInBytes != 2) return VIBE_E_FAIL;

            /* Map 16-bit value to 8-bit */
            nForce = ((VibeInt16*)pForceOutputBuffer)[0] >> 8;
            break;
        default:
            /* Unexpected bit depth */
            return VIBE_E_FAIL;
    }

    if(nForce == 0)
    {
        duty_ns = PWM_DUTY_DEFAULT;
    }
    else
    {
        duty_ns = ((nForce + 128) * g_nPWM_Freq) >> 8;
    }
//	DEBUG_MSG("****** nForce : %d , duty_ns : %d ****\n", nForce, duty_ns);
	tspdrv_control_pwm(1, duty_ns, g_nPWM_Freq);
    return VIBE_S_SUCCESS;
}
/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    return VIBE_S_SUCCESS;
}	
