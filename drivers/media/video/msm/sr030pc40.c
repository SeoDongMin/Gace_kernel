/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>

#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/io.h> 

//#include "msm_camera_gpio.h"
#include "sr030pc40.h"


#define SENSOR_DEBUG 0


#define CAM_MAIN_PWR		123
#define CAM_MAIN_STBY	96
#define CAM_MAIN_RST	0
#define CAM_SUB_PWR		124
#define CAM_SUB_STBY	1
#define CAM_SUB_RST		31

#define CAM_GPIO_SCL	2
#define CAM_GPIO_SDA	3

//#define CONFIG_LOAD_FILE

static char sr030pc40_sensor_init_done = 1;

//Gopeace LeeSangmin DJ26 add
static unsigned short sr030pc40_sensor_version = 0xff;

struct sr030pc40_work {
	struct work_struct work;
};

static struct  sr030pc40_work *sr030pc40_sensorw;
static struct  i2c_client *sr030pc40_client;

struct sr030pc40_ctrl 
{
	const struct msm_camera_sensor_info *sensordata;

};

//Gopeace LeeSangmin Add
struct sr030pc40_data_save
{
	char mEffect;
	char mBrightness;
	char mContrast;
	char mSaturation;
	char mSharpness;
	char mWhiteBalance;
	char mISO;
	char mAutoExposure;
	//char mScene;
	char mDTP;
	char mInit;
	char mPrettyEffect;
	char mVtMode;
	char mFPS;
};

static struct sr030pc40_ctrl *sr030pc40_ctrl;
static struct sr030pc40_data_save sr030pc40_data;

static DECLARE_WAIT_QUEUE_HEAD(sr030pc40_wait_queue);
//DECLARE_MUTEX(sr030pc40_sem);
static int16_t sr030pc40_effect = CAMERA_EFFECT_OFF;

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
#if 0 //For rough control
extern int cpufreq_direct_set_policy(unsigned int cpu, const char *buf);
#endif 
/*=============================================================*/

#ifdef CONFIG_LOAD_FILE
static int sr030pc40_regs_table_write(char *name);
#endif


/**
 * sr030pc40_i2c_read: Read (I2C) multiple bytes to the camera sensor 
 * @client: pointer to i2c_client
 * @cmd: command register
 * @data: data to be read
 *
 * Returns 0 on success, <0 on error
 */
static inline int sr030pc40_sensor_read( unsigned char subaddr, unsigned char *data)
{
	int err = 0;
	unsigned char buf[1] = {0};
	struct i2c_msg msg = {sr030pc40_client->addr, 0, 1, buf};

	if (!sr030pc40_client->adapter)
	{
		dev_err(&sr030pc40_client->dev, "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	} 

	buf[0] = subaddr;//>> 8;
	//buf[1] = subaddr & 0xff;
	
	err = i2c_transfer(sr030pc40_client->adapter, &msg, 1);
	if (unlikely(err < 0))
	{
		dev_err(&sr030pc40_client->dev, "%s: %d register read fail\n", __func__, __LINE__);	
		return -EIO;
	}

	msg.flags = I2C_M_RD;

	err = i2c_transfer(sr030pc40_client->adapter, &msg, 1);
	if (unlikely(err < 0))
	{
		dev_err(&sr030pc40_client->dev, "%s: %d register read fail\n", __func__, __LINE__);	
		return -EIO;
	}

	/*
	 * Data comes in Little Endian in parallel mode; So there
	 * is no need for byte swapping here
	 */
    *data = *(unsigned char *)(&buf);
		
	return err;
}


static int sr030pc40_sensor_write( unsigned char subaddr, unsigned char val)
{
	unsigned char buf[2];
	struct i2c_msg msg = { sr030pc40_client->addr, 0, 2, buf };

	//printk("[PGH] on write func sr030pc40_client->addr : %x\n", sr030pc40_client->addr);
	//printk("[PGH] on write func sr030pc40_client->adapter->nr : %d\n", sr030pc40_client->adapter->nr);
    
	//CAMDRV_DEBUG("[PGH] on write func subaddr:%x, val:%x\n", subaddr, val);
	
	buf[0] = subaddr;
	buf[1] = val;
	//buf[2] = (val >> 8);
	//buf[3] = (val & 0xFF);

	return i2c_transfer(sr030pc40_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

static int sr030pc40_sensor_write_tunning_data( unsigned short code)
{
	unsigned char subaddr, val;

    subaddr  = (unsigned char)((code & 0xff00) >> 8);
    val = (unsigned char)((code & 0x00ff) >> 0);

	return sr030pc40_sensor_write( subaddr, val);
}


static int sr030pc40_sensor_write_list( const unsigned short *regs, int size, char *name)
{
	int i = 0, ret = 0;

    printk("sr030pc40_sensor_write_list name=%s, size=%d \n", name, size);

#ifdef CONFIG_LOAD_FILE
	ret = sr030pc40_regs_table_write( name);
#else
#if 0
	if( !sr030pc40_client->adapter)
	{
		dev_err( &sr030pc40_client->dev, "%s: %d can't search i2c client adapter\n", __func__, __LINE__);
		return -EIO;
	} 
#endif
	for( i = 0; i < size; i++) 
	{
		//printk("sr030pc40_sensor_write_tunning_data reg[%d] : 0x%x\n", i, regs[i]);
		if( sr030pc40_sensor_write_tunning_data( regs[i]))
		{
		    printk( "<=PCAM=> sensor_write_list fail...-_-\n");
		    return -1;
		}
	}
#endif
	return 0;
}

void sr030pc40_effect_control( char value)
{
	switch( value)
	{
		case EXT_CFG_EFFECT_NORMAL :
		{
			CAMDRV_DEBUG( "EXT_CFG_EFFECT_NORMAL");
			sr030pc40_sensor_write_list( sr030pc40_Effect_Normal, SR030PC40_EFFECT_NORMAL_REGS,\
			 "sr030pc40_Effect_Normal"); 
		}
		break;		

		case EXT_CFG_EFFECT_NEGATIVE :
		{
			CAMDRV_DEBUG( "EXT_CFG_EFFECT_NEGATIVE");
			sr030pc40_sensor_write_list( sr030pc40_Effect_Negative, SR030PC40_EFFECT_NEGATIVE_REGS,\
			 "sr030pc40_Effect_Negative"); 
		}
		break;	
		
		case EXT_CFG_EFFECT_MONO :
		{
			CAMDRV_DEBUG("EXT_CFG_EFFECT_MONO");
			sr030pc40_sensor_write_list( sr030pc40_Effect_Black_White, SR030PC40_EFFECT_BLACK_WHITE_REGS,\
			 "sr030pc40_Effect_Black_White"); 
		}
		break;	

		case EXT_CFG_EFFECT_SEPIA :
		{
			CAMDRV_DEBUG("EXT_CFG_EFFECT_SEPIA");
			sr030pc40_sensor_write_list( sr030pc40_Effect_Sepia, SR030PC40_EFFECT_SEPIA_REGS,\
			 "sr030pc40_Effect_Sepia"); 
		}
		break;	
/*
		case EXT_CFG_EFFECT_GREEN :
		{
			CAMDRV_DEBUG("EXT_CFG_EFFECT_GREEN");
			sr030pc40_sensor_write_list( sr030pc40_Effect_Green, SR030PC40_EFFECT_GREEN_REGS,\
			 "sr030pc40_Effect_Green"); 
		}
		break;	

		case EXT_CFG_EFFECT_AQUA :
		{
			CAMDRV_DEBUG("EXT_CFG_EFFECT_AQUA");
			sr030pc40_sensor_write_list( sr030pc40_Effect_Aqua, SR030PC40_EFFECT_AQUA_REGS,\
			 "sr030pc40_Effect_Aqua"); 
		}
		break;	
*/
		default :
		{
			printk("<=PCAM=> Unexpected Effect mode : %d\n",  value);
		}
		break;
				
	}
}


void sr030pc40_pretty_control( char value)
{
	switch( value)
	{
		case EXT_CFG_PRETTY_LEVEL_0:
		{
			CAMDRV_DEBUG( "EXT_CFG_PRETTY_LEVEL_0");
			sr030pc40_sensor_write_list( sr030pc40_Pretty_Level_0, SR030PC40_PRETTY_LEVEL_0_REGS,\
			 "sr030pc40_Pretty_Level_0"); 
		}
		break;		

		case EXT_CFG_PRETTY_LEVEL_1:
		{
			CAMDRV_DEBUG( "EXT_CFG_PRETTY_LEVEL_1");
			sr030pc40_sensor_write_list( sr030pc40_Pretty_Level_1, SR030PC40_PRETTY_LEVEL_1_REGS,\
			 "sr030pc40_Pretty_Level_1"); 
		}
		break;	
		
		case EXT_CFG_PRETTY_LEVEL_2:
		{
			CAMDRV_DEBUG( "EXT_CFG_PRETTY_LEVEL_2");
			sr030pc40_sensor_write_list( sr030pc40_Pretty_Level_2, SR030PC40_PRETTY_LEVEL_2_REGS,\
			 "sr030pc40_Pretty_Level_2"); 
		}
		break;	

		case EXT_CFG_PRETTY_LEVEL_3:
		{
			CAMDRV_DEBUG( "EXT_CFG_PRETTY_LEVEL_3");
			sr030pc40_sensor_write_list( sr030pc40_Pretty_Level_3, SR030PC40_PRETTY_LEVEL_3_REGS,\
			 "sr030pc40_Pretty_Level_3"); 
		}
		break;	

		default :
		{
			printk( "<=PCAM=> Unexpected Pretty Effect mode : %d\n",  value);
		}
		break;
	}
}


void sr030pc40_whitebalance_control(char value)
{

	switch(value)
	{
		case EXT_CFG_WB_AUTO :{
		CAMDRV_DEBUG( "EXT_CFG_WB_AUTO");
		sr030pc40_sensor_write_list( sr030pc40_WB_Auto, SR030PC40_WB_AUTO_REGS,\
		 "sr030pc40_WB_Auto"); 
		}
		break;	

		case EXT_CFG_WB_DAYLIGHT:{
		CAMDRV_DEBUG( "EXT_CFG_WB_DAYLIGHT");
		sr030pc40_sensor_write_list( sr030pc40_WB_Daylight, SR030PC40_WB_DAYLIGHT_REGS,\
		 "sr030pc40_WB_Daylight"); 
		}
		break;	

		case EXT_CFG_WB_CLOUDY :{
		CAMDRV_DEBUG( "EXT_CFG_WB_CLOUDY");
		sr030pc40_sensor_write_list( sr030pc40_WB_Cloudy, SR030PC40_WB_CLOUDY_REGS,\
		 "sr030pc40_WB_Cloudy"); 
		}
		break;	

		case EXT_CFG_WB_FLUORESCENT :{
		CAMDRV_DEBUG( "EXT_CFG_WB_FLUORESCENT");
		sr030pc40_sensor_write_list( sr030pc40_WB_Fluorescent, SR030PC40_WB_FLUORESCENT_REGS,\
		 "sr030pc40_WB_Fluorescent"); 
		}
		break;	
		
		case EXT_CFG_WB_INCANDESCENT :{
		CAMDRV_DEBUG( "EXT_CFG_WB_INCANDESCENT");
		sr030pc40_sensor_write_list( sr030pc40_WB_Incandescent, SR030PC40_WB_INCANDESCENT_REGS,\
		 "sr030pc40_WB_Incandescent"); 
		}
		break;	

		default :{
			printk("<=PCAM=> Unexpected WHITEBALANCE mode : %d\n",  value);
		}
		break;
		
	}// end of switch

}

/*
void sr030pc40_VT_brightness_control(char value)
{
	printk("<=PCAM=> VT Brightness Control 0x%x\n", value);

	switch(value)
	{

		case EXT_CFG_BR_STEP_P_4 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_4");
		sr030pc40_sensor_write_list(sr030pc40_VT_brightness_p_4, SR030PC40_BRIGHTNESS_P_4_REGS,\
		 "sr030pc40_VT_brightness_p_4"); 
		}
		break;

		case EXT_CFG_BR_STEP_P_3 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_3");
		sr030pc40_sensor_write_list(sr030pc40_VT_brightness_p_3, SR030PC40_BRIGHTNESS_P_3_REGS,\
		 "sr030pc40_VT_brightness_p_3"); 
		}
		break;

		case EXT_CFG_BR_STEP_P_2 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_2");
		sr030pc40_sensor_write_list(sr030pc40_VT_brightness_p_2, SR030PC40_BRIGHTNESS_P_2_REGS,\
		 "sr030pc40_VT_brightness_p_2"); 
		}
		break;

		case EXT_CFG_BR_STEP_P_1 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_1");
		sr030pc40_sensor_write_list(sr030pc40_VT_brightness_p_1, SR030PC40_BRIGHTNESS_P_1_REGS,\
		 "sr030pc40_VT_brightness_p_1"); 
		}
		break;

		case EXT_CFG_BR_STEP_0 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_0");
		sr030pc40_sensor_write_list(sr030pc40_VT_brightness_0, SR030PC40_BRIGHTNESS_0_REGS, \
		"sr030pc40_VT_brightness_0"); 
		}
		break;

		case EXT_CFG_BR_STEP_M_1 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_1");
		sr030pc40_sensor_write_list(sr030pc40_VT_brightness_m_1, SR030PC40_BRIGHTNESS_M_1_REGS, \
		"sr030pc40_VT_brightness_m_1"); 
		}
		break;

		case EXT_CFG_BR_STEP_M_2 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_2");
		sr030pc40_sensor_write_list(sr030pc40_VT_brightness_m_2, SR030PC40_BRIGHTNESS_M_2_REGS, \
		"sr030pc40_VT_brightness_m_2"); 
		}
		break;

		case EXT_CFG_BR_STEP_M_3 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_3");
		sr030pc40_sensor_write_list(sr030pc40_VT_brightness_m_3, SR030PC40_BRIGHTNESS_M_3_REGS, \
		"sr030pc40_VT_brightness_m_3"); 
		}
		break;

		case EXT_CFG_BR_STEP_M_4 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_4");
		sr030pc40_sensor_write_list(sr030pc40_VT_brightness_m_4, SR030PC40_BRIGHTNESS_M_4_REGS, \
		"sr030pc40_VT_brightness_m_4"); 
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected VT_BR mode : %d\n",  value);
		}
		break;

	}
}
*/

void sr030pc40_brightness_control(char value)
{
	printk("<=PCAM=> Brightness Control 0x%x\n", value);

	switch(value)
	{
		case EXT_CFG_BR_STEP_P_4 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_4");
		sr030pc40_sensor_write_list(sr030pc40_brightness_p_4, SR030PC40_BRIGHTNESS_P_4_REGS,\
		 "sr030pc40_brightness_p_4"); 
		}
		break;

		case EXT_CFG_BR_STEP_P_3 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_3");
		sr030pc40_sensor_write_list(sr030pc40_brightness_p_3, SR030PC40_BRIGHTNESS_P_3_REGS,\
		 "sr030pc40_brightness_p_3"); 
		}
		break;

		case EXT_CFG_BR_STEP_P_2 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_2");
		sr030pc40_sensor_write_list(sr030pc40_brightness_p_2, SR030PC40_BRIGHTNESS_P_2_REGS,\
		 "sr030pc40_brightness_p_2"); 
		}
		break;

		case EXT_CFG_BR_STEP_P_1 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_1");
		sr030pc40_sensor_write_list(sr030pc40_brightness_p_1, SR030PC40_BRIGHTNESS_P_1_REGS,\
		 "sr030pc40_brightness_p_1"); 
		}
		break;

		case EXT_CFG_BR_STEP_0 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_0");
		sr030pc40_sensor_write_list(sr030pc40_brightness_0, SR030PC40_BRIGHTNESS_0_REGS, \
		"sr030pc40_brightness_0"); 
		}
		break;

		case EXT_CFG_BR_STEP_M_1 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_1");
		sr030pc40_sensor_write_list(sr030pc40_brightness_m_1, SR030PC40_BRIGHTNESS_M_1_REGS, \
		"sr030pc40_brightness_m_1"); 
		}
		break;

		case EXT_CFG_BR_STEP_M_2 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_2");
		sr030pc40_sensor_write_list(sr030pc40_brightness_m_2, SR030PC40_BRIGHTNESS_M_2_REGS, \
		"sr030pc40_brightness_m_2"); 
		}
		break;

		case EXT_CFG_BR_STEP_M_3 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_3");
		sr030pc40_sensor_write_list(sr030pc40_brightness_m_3, SR030PC40_BRIGHTNESS_M_3_REGS, \
		"sr030pc40_brightness_m_3"); 
		}
		break;

		case EXT_CFG_BR_STEP_M_4 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_4");
		sr030pc40_sensor_write_list(sr030pc40_brightness_m_4, SR030PC40_BRIGHTNESS_M_4_REGS, \
		"sr030pc40_brightness_m_4"); 
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected BR mode : %d\n",  value);
		}
		break;

	}
}

void sr030pc40_FPS_control(char value)
{
	printk("sr030pc40_FPS_control() value:%d", value );
	
	switch( value)
	{
		case EXT_CFG_FRAME_AUTO :
		{
			CAMDRV_DEBUG( "EXT_CFG_FRAME_AUTO\n");
		}					
		break;
		
		case EXT_CFG_FRAME_FIX_15 :
		{
			CAMDRV_DEBUG( "EXT_CFG_FRAME_FIX_15\n");		
			sr030pc40_sensor_write_list( sr030pc40_15_fps, SR030PC40_15_FPS_REGS,\
			 "sr030pc40_15_fps"); 	
		}
		break;

		case EXT_CFG_FRAME_FIX_10 :
		{
			CAMDRV_DEBUG( "EXT_CFG_FRAME_FIX_10\n");		
			sr030pc40_sensor_write_list( sr030pc40_10_fps, SR030PC40_10_FPS_REGS,\
			 "sr030pc40_10_fps"); 	
		}
		break;

		case EXT_CFG_FRAME_FIX_7 :
		{
			CAMDRV_DEBUG( "EXT_CFG_FRAME_FIX_7\n");		
			sr030pc40_sensor_write_list( sr030pc40_7_fps, SR030PC40_7_FPS_REGS,\
			 "sr030pc40_7_fps"); 	
		}
		break;

		default :
		{
			CAMDRV_DEBUG( "<=PCAM=> Unexpected EXT_CFG_FRAME_CONTROL mode : %d\n", value);
		}
		break;				
	}
}
#if 0

void sensor_iso_control(char value)
{
	printk("<=PCAM=> ISO Control\n");
	return;

	switch(value)
	{
		case EXT_CFG_ISO_AUTO :{
		CAMDRV_DEBUG("EXT_CFG_ISO_AUTO");
		sr030pc40_sensor_write_list(sr030pc40_iso_auto, S5K5CA_ISO_AUTO_REGS, \
		"sr030pc40_iso_auto"); 
		}
		break;

		case EXT_CFG_ISO_50 :{
		CAMDRV_DEBUG("EXT_CFG_ISO_50");
		sr030pc40_sensor_write_list(sr030pc40_iso_50, S5K5CA_ISO_50_REGS, \
		"sr030pc40_iso_50"); 
		}
		break;

		case EXT_CFG_ISO_100 :{
		CAMDRV_DEBUG("EXT_CFG_ISO_100");
		sr030pc40_sensor_write_list(sr030pc40_iso_100, S5K5CA_ISO_100_REGS, \
		"sr030pc40_iso_100"); 
		}
		break;

		CAMDRV_DEBUG("EXT_CFG_ISO_200");
		sr030pc40_sensor_write_list(sr030pc40_iso_200, S5K5CA_ISO_200_REGS, \
		"sr030pc40_iso_200"); 
		}
		break;

		case EXT_CFG_ISO_400 :{
		CAMDRV_DEBUG("EXT_CFG_ISO_400");
		sr030pc40_sensor_write_list(sr030pc40_iso_400, S5K5CA_ISO_400_REGS, \
		"sr030pc40_iso_400"); 
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected ISO mode : %d\n",  value);
		}
		break;
		
	}

}
#endif 

#if 0
void sensor_metering_control(char value)
{
	printk("<=PCAM=> Metering Control\n");
	return;

	switch(value)
	{
		case EXT_CFG_METERING_NORMAL :{
		CAMDRV_DEBUG("EXT_CFG_METERING_NORMAL");
		sr030pc40_sensor_write_list(sr030pc40_metering_normal, S5K5CA_METERING_NORMAL_REGS, \
		"sr030pc40_metering_normal"); 
		}
		break;
		
		case EXT_CFG_METERING_SPOT :{
		CAMDRV_DEBUG("EXT_CFG_METERING_SPOT");
		sr030pc40_sensor_write_list(sr030pc40_metering_spot, S5K5CA_METERING_SPOT_REGS, \
		"sr030pc40_metering_spot"); 
		}
		break;

		case EXT_CFG_METERING_CENTER :{
		CAMDRV_DEBUG("EXT_CFG_METERING_CENTER");
		sr030pc40_sensor_write_list(sr030pc40_metering_center, S5K5CA_METERING_CENTER_REGS, \
		"sr030pc40_metering_center"); 
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected METERING mode : %d\n",  value);
		}
		break;
	}
}
#endif


#if 0

void sensor_contrast_control(char value)
{
	printk("<=PCAM=> Contrast Control\n");
	return;

	switch(value)
	{
		case EXT_CFG_CR_STEP_M_2 :{
		CAMDRV_DEBUG("EXT_CFG_CR_STEP_P_2");
		sr030pc40_sensor_write_list(sr030pc40_contrast_m_2 , S5K5CA_CONTRAST_M_2_REGS, \
		"sr030pc40_contrast_m_2"); 					
		}
		break;

		case EXT_CFG_CR_STEP_M_1 :{
		CAMDRV_DEBUG("EXT_CFG_CR_STEP_P_1");
		sr030pc40_sensor_write_list(sr030pc40_contrast_m_1 , S5K5CA_CONTRAST_M_1_REGS, \
		"sr030pc40_contrast_m_1"); 					
		}
		break;

		case EXT_CFG_CR_STEP_0 :{
		CAMDRV_DEBUG("EXT_CFG_CR_STEP_0");
		sr030pc40_sensor_write_list(sr030pc40_contrast_0 , S5K5CA_CONTRAST_0_REGS, \
		"sr030pc40_contrast_0"); 					
		}
		break;

		case EXT_CFG_CR_STEP_P_1 :{
		CAMDRV_DEBUG("EXT_CFG_CR_STEP_P_1");
		sr030pc40_sensor_write_list(sr030pc40_contrast_p_1 , S5K5CA_CONTRAST_P_1_REGS, \
		"sr030pc40_contrast_p_1"); 					
		}
		break;

		case EXT_CFG_CR_STEP_P_2 :{
		CAMDRV_DEBUG("EXT_CFG_CR_STEP_P_2");
		sr030pc40_sensor_write_list(sr030pc40_contrast_p_2 , S5K5CA_CONTRAST_P_2_REGS, \
		"sr030pc40_contrast_p_2"); 					
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected EXT_CFG_CR_CONTROL mode : %d\n",  value);
		}
		break;												
	}

}
#endif

#if 0

void sensor_saturation_control(char value)
{
	printk("<=PCAM=> Saturation Control\n");
	return;

	switch(value)
	{
		case EXT_CFG_SA_STEP_M_2 :{
		CAMDRV_DEBUG("EXT_CFG_SA_STEP_P_2");
		sr030pc40_sensor_write_list(sr030pc40_saturation_m_2 , S5K5CA_SATURATION_M_2_REGS, \
		"sr030pc40_saturation_m_2"); 					
		}
		break;

		case EXT_CFG_SA_STEP_M_1 :{
		CAMDRV_DEBUG("EXT_CFG_SA_STEP_P_1");
		sr030pc40_sensor_write_list(sr030pc40_saturation_m_1 , S5K5CA_SATURATION_M_1_REGS, \
		"sr030pc40_saturation_m_1"); 					
		}
		break;

		case EXT_CFG_SA_STEP_0 :{
		CAMDRV_DEBUG("EXT_CFG_SA_STEP_0");
		sr030pc40_sensor_write_list(sr030pc40_saturation_0 , S5K5CA_SATURATION_0_REGS, \
		"sr030pc40_saturation_0"); 					
		}
		break;

		case EXT_CFG_SA_STEP_P_1 :{
		CAMDRV_DEBUG("EXT_CFG_SA_STEP_P_1");
		sr030pc40_sensor_write_list(sr030pc40_saturation_p_1 , S5K5CA_SATURATION_P_1_REGS, \
		"sr030pc40_saturation_p_1"); 					
		}
		break;

		case EXT_CFG_SA_STEP_P_2 :{
		CAMDRV_DEBUG("EXT_CFG_SA_STEP_P_2");
		sr030pc40_sensor_write_list(sr030pc40_saturation_p_2 , S5K5CA_SATURATION_P_2_REGS, \
		"sr030pc40_saturation_p_2"); 					
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected EXT_CFG_SA_CONTROL mode : %d\n",  value);
		}
		break;					
	}

}
#endif

#if 0

void sensor_sharpness_control(char value)
{
	printk("<=PCAM=> Sharpness Control\n");
	return;

	switch(value)
	{
		case EXT_CFG_SP_STEP_M_2 :{
		CAMDRV_DEBUG("EXT_CFG_SP_STEP_P_2");
		sr030pc40_sensor_write_list(sr030pc40_sharpness_m_2 , S5K5CA_SHARPNESS_M_2_REGS, \
		"sr030pc40_sharpness_m_2"); 					
		}
		break;

		case EXT_CFG_SP_STEP_M_1 :{
		CAMDRV_DEBUG("EXT_CFG_SP_STEP_P_1");
		sr030pc40_sensor_write_list(sr030pc40_sharpness_m_1 , S5K5CA_SHARPNESS_M_1_REGS, \
		"sr030pc40_sharpness_m_1"); 					
		}
		break;

		case EXT_CFG_SP_STEP_0 :{
		CAMDRV_DEBUG("EXT_CFG_SP_STEP_0");
		sr030pc40_sensor_write_list(sr030pc40_sharpness_0 , S5K5CA_SHARPNESS_0_REGS, \
		"sr030pc40_sharpness_0"); 					
		}
		break;

		case EXT_CFG_SP_STEP_P_1 :{
		CAMDRV_DEBUG("EXT_CFG_SP_STEP_P_1");
		sr030pc40_sensor_write_list(sr030pc40_sharpness_p_1 , S5K5CA_SHARPNESS_P_1_REGS, \
		"sr030pc40_sharpness_p_1"); 					
		}
		break;

		case EXT_CFG_SP_STEP_P_2 :{
		CAMDRV_DEBUG("EXT_CFG_SP_STEP_P_2");
		sr030pc40_sensor_write_list(sr030pc40_sharpness_p_2 , S5K5CA_SHARPNESS_P_2_REGS, \
		"sr030pc40_sharpness_p_2"); 					
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected EXT_CFG_SP_CONTROL mode : %d\n",  value);
		}
		break;					
	}

}
#endif

static void sr030pc40_set_power( int status)
{
    struct vreg *vreg_cam_out8_vddio;
    struct vreg *vreg_cam_out9_vdda;
    struct vreg *vreg_cam_out10_af; 

    vreg_cam_out8_vddio		= vreg_get(NULL, "ldo8");
    vreg_cam_out9_vdda		= vreg_get(NULL, "ldo9");
    vreg_cam_out10_af		= vreg_get(NULL, "ldo10");
    
	if(status == 1) //POWER ON
	{
		CAMDRV_DEBUG("sr030pc40_set_power : POWER ON");
        
		// gpio & pmic config
		gpio_tlmm_config(GPIO_CFG(CAM_MAIN_PWR, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(CAM_MAIN_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(CAM_MAIN_STBY, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(CAM_SUB_PWR, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(CAM_SUB_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(CAM_SUB_STBY, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		
		gpio_set_value(CAM_MAIN_PWR, 0);
		gpio_set_value(CAM_MAIN_RST, 0);
		gpio_set_value(CAM_MAIN_STBY, 0);
		gpio_set_value(CAM_SUB_PWR, 0);
		gpio_set_value(CAM_SUB_STBY, 0);
		gpio_set_value(CAM_SUB_RST, 0);
		
		vreg_set_level(vreg_cam_out10_af,  OUT3000mV);
		vreg_set_level(vreg_cam_out9_vdda,	OUT2800mV);
		vreg_set_level(vreg_cam_out8_vddio,  OUT1800mV); // IO 2.8V -> 1.8V 
		// end of gpio & pmic config

		// start power sequence
		gpio_set_value(CAM_MAIN_PWR, 1);
		udelay(1); // >0us
		vreg_enable(vreg_cam_out9_vdda);
		udelay(30); // >20us
        gpio_set_value(CAM_SUB_PWR, 1);
        udelay(20); // >15us
		vreg_enable(vreg_cam_out8_vddio);
		udelay(30); // >20us
    	
		gpio_set_value(CAM_SUB_STBY, 1);
		mdelay(6); // >5ms
		gpio_set_value(CAM_SUB_STBY, 0);

		//msm_camio_clk_enable(CAMIO_VFE_CLK);
		msm_camio_clk_rate_set(24000000);
		//msm_camio_camif_pad_reg_reset();
		//gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		udelay(20); // >10us

		gpio_set_value(CAM_MAIN_STBY, 1);
		mdelay(6); // >5ms
    	
		gpio_set_value(CAM_MAIN_RST, 1);
		mdelay(7); // >6.5ms

		gpio_set_value(CAM_MAIN_STBY, 0);
		udelay(10); // >10us

		gpio_set_value(CAM_SUB_RST, 1);
		mdelay(51); // >50ms
	}
	else //POWER OFF
	{
		CAMDRV_DEBUG("sr030pc40_set_power : POWER OFF");

		gpio_set_value(CAM_MAIN_RST, 0);
		udelay(60); // >50us
        
		//msm_camio_camif_pad_reg_reset();
		//msm_camio_clk_disable(CAMIO_VFE_CLK);

		gpio_set_value(CAM_MAIN_STBY, 0);
		gpio_set_value(CAM_SUB_RST, 0);
		gpio_set_value(CAM_SUB_STBY, 0);
        
		vreg_disable(vreg_cam_out8_vddio);
		gpio_set_value(CAM_SUB_PWR, 0);
		vreg_disable(vreg_cam_out9_vdda);
		gpio_set_value(CAM_MAIN_PWR, 0);

		// for sleep current
		gpio_tlmm_config(GPIO_CFG(CAM_GPIO_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(CAM_GPIO_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(CAM_GPIO_SCL, 0);
		gpio_set_value(CAM_GPIO_SDA, 0);
	}
}

#if 1
void sr030pc40_DTP_control( char value)
{
	switch( value)
	{
		case EXT_CFG_DTP_OFF:
		{
			CAMDRV_DEBUG( "DTP OFF");
			sr030pc40_data.mDTP = 0;
			sr030pc40_sensor_write_list( sr030pc40_dtp_off, SR030PC40_DTP_OFF_REGS, "sr030pc40_dtp_off");
			sr030pc40_sensor_write_list( sr030pc40_init_reg, SR030PC40_INIT_REGS, "sr030pc40_init_reg");     
			//sr030pc40_reset();
		}
		break;

		case EXT_CFG_DTP_ON:
		{
			CAMDRV_DEBUG( "DTP ON");
			sr030pc40_data.mDTP = 1;
			sr030pc40_sensor_write_list( sr030pc40_dtp_on, SR030PC40_DTP_ON_REGS, "sr030pc40_dtp_on");
		}
		break;

		default:
		{
			printk( "<=PCAM=> unexpected DTP control on PCAM\n");
		}
		break;
	}
}
#endif


int sr030pc40_sensor_ext_config(void __user *arg)
{
	long   rc = 0;
	ioctl_pcam_info_8bit		ctrl_info;

	CAMDRV_DEBUG("START");

	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail copy_from_user!\n", __func__);
	}

	printk("<=PCAM=> TEST %d %d %d %d %d \n", ctrl_info.mode, ctrl_info.address,\
	 ctrl_info.value_1, ctrl_info.value_2, ctrl_info.value_3);

	switch(ctrl_info.mode)
	{
		case EXT_CFG_AUTO_TUNNING:
		break;

		case EXT_CFG_SDCARD_DETECT:
		break;

		case EXT_CFG_GET_INFO:
		{
#if 0
			unsigned short lsb, msb, rough_iso;					
			s5k5ca_sensor_write(0xFCFC, 0xD000);					
			s5k5ca_sensor_write(0x002C, 0x7000);
			s5k5ca_sensor_write(0x002E, 0x23E8);
			s5k5ca_sensor_read(0x0F12, &lsb); //0x23E8
			s5k5ca_sensor_read(0x0F12, &msb); //0x23EA
			s5k5ca_sensor_read(0x0F12, &rough_iso); //0x23EC
													
			ctrl_info.value_1 = lsb;					
			ctrl_info.value_2 = msb;	
			//printk("<=PCAM=> exposure %x %x \n", lsb, msb);
			ctrl_info.value_3 = rough_iso;	
			//printk("<=PCAM=> rough_iso %x \n", rough_iso);			
#endif
		}
		break;

		case EXT_CFG_VT_MODE_CONTROL:
		{
			if( 1 == ctrl_info.value_1)
			{
				printk( "EXT_CFG_VT_MODE\n");
				sr030pc40_data.mVtMode = 1;
				//mVtMode = 1;
			}
			else if( 0 == ctrl_info.value_1)
			{
				printk( "EXT_CFG_NORMAL_MODE\n");
				sr030pc40_data.mVtMode = 0;
				//mVtMode = 0;
			}
			else
			{
				CAMDRV_DEBUG( "EXT_CFG_MODE CHANGE ERROR\n");
			}
		}
		break;

		case EXT_CFG_FRAME_CONTROL:
		{
			sr030pc40_data.mFPS = ctrl_info.value_1;
			sr030pc40_FPS_control( sr030pc40_data.mFPS);
		}
		break;

		case EXT_CFG_EFFECT_CONTROL:
		{
			sr030pc40_data.mEffect = ctrl_info.value_1;
			sr030pc40_effect_control( sr030pc40_data.mEffect);
		}// end of EXT_CFG_EFFECT_CONTROL
		break;

		case EXT_CFG_WB_CONTROL:
		{
			sr030pc40_data.mWhiteBalance = ctrl_info.value_1;
			sr030pc40_whitebalance_control( sr030pc40_data.mWhiteBalance);
		}//end of EXT_CFG_WB_CONTROL
		break;

		case EXT_CFG_BR_CONTROL:
		{
			sr030pc40_data.mBrightness = ctrl_info.value_1;
			//if(mInit)
			//if( 0 == sr030pc40_data.mVtMode)
				sr030pc40_brightness_control( sr030pc40_data.mBrightness);
/*			if( 1 == sr030pc40_data.mVtMode)
				sr030pc40_VT_brightness_control( sr030pc40_data.mBrightness);*/
			
			
		}//end of EXT_CFG_BR_CONTROL
		break;

		case EXT_CFG_ISO_CONTROL:
		{
			sr030pc40_data.mISO = ctrl_info.value_1;
			//sensor_iso_control(mISO);
		}
		break;

		case EXT_CFG_METERING_CONTROL:
		{
			sr030pc40_data.mAutoExposure = ctrl_info.value_1;
			//sensor_metering_control(mAutoExposure);
			
		}//end of case
		break;

		case EXT_CFG_AE_AWB_CONTROL:
		{
#if 0
			switch(ctrl_info.value_1)
			{
				case EXT_CFG_AWB_AE_LOCK :{
				CAMDRV_DEBUG("EXT_CFG_AWB_AE_LOCK");
				sr030pc40_sensor_write_list(sr030pc40_awb_ae_lock , S5K5CA_AWB_AE_LOCK_REGS, \
				"sr030pc40_awb_ae_lock"); 					
				}
				break;

				case EXT_CFG_AWB_AE_UNLOCK :{
				CAMDRV_DEBUG("EXT_CFG_AWB_AE_UNLOCK");
				sr030pc40_sensor_write_list(sr030pc40_awb_ae_unlock , S5K5CA_AWB_AE_UNLOCK_REGS, \
				"sr030pc40_awb_ae_unlock"); 
					
				}
				break;

				default :{
					printk("<=PCAM=> Unexpected AWB_AE mode : %d\n", ctrl_info.value_1);
				}
				break;						
				
			}
#endif
		}
		break;

		case EXT_CFG_PRETTY_CONTROL:
		{
			sr030pc40_data.mPrettyEffect = ctrl_info.value_1;
			sr030pc40_pretty_control( sr030pc40_data.mPrettyEffect);
		}
		break;
			
		case EXT_CFG_CR_CONTROL:
		{
			sr030pc40_data.mContrast = ctrl_info.value_1;
			//if(mInit)
				//sensor_contrast_control(mContrast);
		}
		break;
			
		case EXT_CFG_SA_CONTROL:
		{
			sr030pc40_data.mSaturation = ctrl_info.value_1;
			//if(mInit)
				//sensor_saturation_control(mSaturation);
		}
		break;

		case EXT_CFG_SP_CONTROL:
		{
			sr030pc40_data.mSharpness = ctrl_info.value_1;
			//if(mInit)
			//	sensor_sharpness_control(mSharpness);
		}
		break;

		case EXT_CFG_CPU_CONTROL:
		{
#if 0
			switch(ctrl_info.value_1)
			{
				case EXT_CFG_CPU_CONSERVATIVE:{
				CAMDRV_DEBUG("now conservative");
				cpufreq_direct_set_policy(0, "conservative");
				}
				break;

				case EXT_CFG_CPU_ONDEMAND:{
				CAMDRV_DEBUG("now ondemand");
				cpufreq_direct_set_policy(0, "ondemand");
				}
				break;	

				case EXT_CFG_CPU_PERFORMANCE:{
				CAMDRV_DEBUG("now performance");
				cpufreq_direct_set_policy(0, "performance");
				}
				break;
				
				default:{
					printk("<=PCAM=> unexpected CPU control on PCAM\n");
				}
				break;
			}
#endif
		}

		break;

		case EXT_CFG_DTP_CONTROL:
		{
#if 1
			if(sr030pc40_data.mInit == sr030pc40_sensor_init_done)
			{
				sr030pc40_DTP_control( ctrl_info.value_1);

				if(ctrl_info.value_1 == 0)
					ctrl_info.value_3 = 2;

				else if(ctrl_info.value_1 == 1)
					ctrl_info.value_3 = 3;
			}
			else
			{
				ctrl_info.value_3 = ctrl_info.value_1;
			}
#endif
		}
		break;


		default :{
			printk("<=PCAM=> Unexpected mode on sr030pc40_sensor_control : %d\n", ctrl_info.mode);
		}
		break;
	}

	if(copy_to_user((void *)arg, (const void *)&ctrl_info, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail on copy_to_user!\n", __func__);
	}
    return rc;
}


/*===========================================================================
FUNCTION      CAMSENSOR_sr030pc40_CHECK_SENSOR_REV
===========================================================================*/

static unsigned char sr030pc40_check_sensor_rev( void)
{
    unsigned char id = 0xff;
    int ret0, ret1;
  
  	printk("sr030pc40_check_sensor_rev!!!\n");
  
    sr030pc40_sensor_version = 0xff;
    
    ret0 = sr030pc40_sensor_write( 0x03, 0x01);
	ret1 = sr030pc40_sensor_read( 0x04, &id);
    
    if( id == 0x96 )
    {
        printk("============================\n");
        printk("[cam] SR030PC40				\n");
        printk("============================\n");

        sr030pc40_sensor_version 	= 0x96; //SR030PC40
    }
    else 
    {
        printk("-----------------------------------------------\n");
        printk("   [cam] INVALID SENSOR  : %d   \n",id);
        printk("-----------------------------------------------\n");
	
        sr030pc40_sensor_version = 0xFF; //No sensor
    }
    return sr030pc40_sensor_version;
}

static int sr030pc40_hw_init()
{

	int rc = 0;
    unsigned short	id = 0; //CAM FOR FW

    CAMDRV_DEBUG("next sr030pc40_hw_init");

    id = sr030pc40_check_sensor_rev();

    if(id != 0x96)
    {
        printk("<=PCAM=> WRONG SENSOR ID => id 0x%x \n", id);
		rc = -1;
    }
	else
	{
		printk("<=PCAM=> CURRENT FRONT SENSOR ID => id 0x%x \n", id);
	}

    return rc;
}


static long sr030pc40_set_effect(int mode, int effect)
{
    long rc = 0;

    switch (mode) {
        case SENSOR_PREVIEW_MODE:
        	CAMDRV_DEBUG("SENSOR_PREVIEW_MODE");
        	break;

        case SENSOR_SNAPSHOT_MODE:
        	CAMDRV_DEBUG("SENSOR_SNAPSHOT_MODE");
        	break;

        default:
        	printk("[PGH] %s default\n", __func__);
        	break;
    }

    switch (effect) {
        case CAMERA_EFFECT_OFF: {
            CAMDRV_DEBUG("CAMERA_EFFECT_OFF");
    }
        break;

        case CAMERA_EFFECT_MONO: {
            CAMDRV_DEBUG("CAMERA_EFFECT_MONO");
    }
        break;

        case CAMERA_EFFECT_NEGATIVE: {
            CAMDRV_DEBUG("CAMERA_EFFECT_NEGATIVE");
    }
        break;

        case CAMERA_EFFECT_SOLARIZE: {
            CAMDRV_DEBUG("CAMERA_EFFECT_SOLARIZE");
    }
        break;

        case CAMERA_EFFECT_SEPIA: {
            CAMDRV_DEBUG("CAMERA_EFFECT_SEPIA");
    }
        break;

    default: {
        printk("<=PCAM=> unexpeceted effect  %s/%d\n", __func__, __LINE__);
        return -EINVAL;
    }

    }
    sr030pc40_effect = effect;

    return rc;
}


static int sr030pc40_debugprint_preview_data( void)
{
	unsigned char read_value = 0;

	printk( "sr030pc40_preview start\n");
	printk( "==============================\n");
	sr030pc40_sensor_write( 0xef, 0x03);
	sr030pc40_sensor_read( 0x33, &read_value);
	printk("Normal Light Contrast : 0x%2x\n", read_value);
	sr030pc40_sensor_read( 0x34, &read_value);
	printk("Low Light Contrast    : 0x%2x\n", read_value);
	sr030pc40_sensor_read( 0x01, &read_value);
	printk("AE Target             : 0x%2x\n", read_value);
	sr030pc40_sensor_read( 0x02, &read_value);
	printk("AE Threshold          : 0x%2x\n", read_value);
	sr030pc40_sensor_read( 0x67, &read_value);
	printk("AE Speed              : 0x%2x\n", read_value);
	printk( "==============================\n");

	return 0;
}

static int sr030pc40_start_preview( void)
{
	sr030pc40_debugprint_preview_data();


	if( sr030pc40_data.mDTP == 1)
	{
		sr030pc40_sensor_write_list( sr030pc40_dtp_on, SR030PC40_DTP_ON_REGS, "sr030pc40_dtp_on");
	}
	else
	{
		sr030pc40_DTP_control( EXT_CFG_DTP_OFF);
		
		if( 0 == sr030pc40_data.mVtMode)
		{
			printk( "sr030pc40 Normal Preview start");
			sr030pc40_sensor_write_list( sr030pc40_init_reg, SR030PC40_INIT_REGS, "sr030pc40_init_reg");     
		}
		else if( 1 == sr030pc40_data.mVtMode)
		{
			printk( "sr030pc40 VtMode Preview start");
			sr030pc40_sensor_write_list( sr030pc40_VT_init_reg, SR030PC40_VT_INIT_REGS, "sr030pc40_VT_init_reg"); 

			sr030pc40_FPS_control( sr030pc40_data.mFPS);
		}

		//sr030pc40_sensor_write_list(sr030pc40_preview, SR030PC40_PREVIEW_REGS, "sr030pc40_preview");
	}

	sr030pc40_brightness_control( sr030pc40_data.mBrightness);

	//sr030pc40_effect_control( mEffect);
	//sensor_whitebalance_control( mWhiteBalance);
#if 0 //Gopeace LeeSangmin DJ26 Effect Setting skip for preview test
	sr030pc40_iso_control(mISO);			
	sensor_metering_control(mAutoExposure);
	sensor_contrast_control(mContrast);
	sensor_saturation_control(mSaturation);
	sensor_sharpness_control(mSharpness);
#endif	
	return 0;
}


static long sr030pc40_set_sensor_mode(int mode)
{
	printk( "sr030pc40_set_sensor_mode start : %d\n", mode);

	switch (mode) 
	{
		case SENSOR_PREVIEW_MODE:
		{
			CAMDRV_DEBUG("PREVIEW~~~\n");
			sr030pc40_start_preview();
		}
		break;
			
		case SENSOR_SNAPSHOT_MODE:
		{
			CAMDRV_DEBUG("SNAPSHOT~~~\n");
		}
		break;

		case SENSOR_RAW_SNAPSHOT_MODE:
		{
			CAMDRV_DEBUG("RAW_SNAPSHOT NOT SUPPORT!!");
		}
		break;

		default:
		{
			return -EINVAL;
		}
	}

	return 0;
}

static int sr030pc40_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	return rc;
}

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

static char *sr030pc40_regs_table = NULL;

static int sr030pc40_regs_table_size;

void sr030pc40_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
//	int i;
	int ret;
	mm_segment_t fs = get_fs();

	printk("%s %d\n", __func__, __LINE__);

	set_fs(get_ds());
#if 0
	filp = filp_open("/data/camera/s5k5ca.h", O_RDONLY, 0);
#else
	filp = filp_open("/mnt/sdcard/sr030pc40.h", O_RDONLY, 0);
#endif
	if (IS_ERR(filp)) {
		printk("file open error\n");
		return;
	}
	l = filp->f_path.dentry->d_inode->i_size;	
	printk("l = %ld\n", l);
	dp = kmalloc(l, GFP_KERNEL);
	if (dp == NULL) {
		printk("Out of Memory\n");
		filp_close(filp, current->files);
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		printk("Failed to read file ret = %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return;
	}

	filp_close(filp, current->files);
	
	set_fs(fs);

	sr030pc40_regs_table = dp;
	
	sr030pc40_regs_table_size = l;

	*((sr030pc40_regs_table + sr030pc40_regs_table_size) - 1) = '\0';

	printk("sr030pc40_regs_table 0x%x, %ld\n", dp, l);
}

void sr030pc40_regs_table_exit(void)
{
	printk("%s %d\n", __func__, __LINE__);
	if (sr030pc40_regs_table) {
		kfree(sr030pc40_regs_table);
		sr030pc40_regs_table = NULL;
	}	
}

static int sr030pc40_regs_table_write(char *name)
{
	char *start, *end, *reg;//, *data;	
	unsigned short addr, value;
	char reg_buf[7], data_buf[7];

    printk("sr030pc40_regs_table_write name=%s \n", name);
    

	addr = value = 0;

	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';

	start = strstr(sr030pc40_regs_table, name);	
	end = strstr(start, "};");

	while (1) {	
		/* Find Address */	
		reg = strstr(start,"0x");		
		if (reg)
			start = (reg + 4);
			
		if ((reg == NULL) || (reg > end))
			break;
			
		/* Write Value to Address */	
		if (reg != NULL) {
			memcpy(reg_buf, (reg), 6);	

			value = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
            printk("value 0x%04x : ", value);

			if (addr == 0xffff)
			{
				msleep(value);
				printk("delay 0x%04x, value 0x%04x\n", addr, value);
			}	
			else
			{
    			if(sr030pc40_sensor_write_tunning_data(value) < 0)
				{
					printk("<=PCAM=> %s fail on sensor_write\n", __func__);
				}
			}
		}
		else
			printk("<=PCAM=> EXCEPTION! reg value : %c  addr : 0x%x,  value : 0x%x\n", *reg, addr, value);
	}

	return 0;
}
#endif



int sr030pc40_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	sr030pc40_ctrl = kzalloc(sizeof(struct sr030pc40_ctrl), GFP_KERNEL);
	if (!sr030pc40_ctrl) {
		CDBG("sr030pc40_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		sr030pc40_ctrl->sensordata = data;

	rc = sr030pc40_sensor_init_probe(data);

    sr030pc40_set_power(1);
	
	rc = sr030pc40_hw_init();

	if(rc < 0)
	{
		printk("<=PCAM=> cam_fw_init failed!\n");
		goto init_fail;  //Gopeace LeeSangmin DJ26 For test
	}

    


#ifdef CONFIG_LOAD_FILE
	sr030pc40_regs_table_init();
#endif	

	rc = sr030pc40_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("sr030pc40_sensor_init failed!\n");
		goto init_fail;
	}

init_done:
	return rc;

init_fail:
	kfree(sr030pc40_ctrl);
	return rc;
}

static int sr030pc40_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&sr030pc40_wait_queue);
	return 0;
}

int sr030pc40_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&sr030pc40_sem); */

	CDBG("sr030pc40_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

		switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = sr030pc40_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = sr030pc40_set_effect(cfg_data.mode,
						cfg_data.cfg.effect);
			break;

		case CFG_GET_AF_MAX_STEPS:
		default:
			rc = -EINVAL;
			break;
		}

	/* up(&sr030pc40_sem); */

	return rc; 
}

int sr030pc40_sensor_release(void)
{
	int rc = 0;

	//If did not init below that, it can keep the previous status. it depend on concept by PCAM

	sr030pc40_data.mEffect = 0;
	sr030pc40_data.mBrightness = 0;
	sr030pc40_data.mContrast = 0;
	sr030pc40_data.mSaturation = 0;
	sr030pc40_data.mSharpness = 0;
	sr030pc40_data.mWhiteBalance = 0;
	sr030pc40_data.mISO = 0;
	sr030pc40_data.mAutoExposure = 0;
	//sr030pc40_data.mScene = 0;
	//mAfMode = 0;
	sr030pc40_data.mDTP = 0;
	sr030pc40_data.mInit = 0;
	sr030pc40_data.mFPS = 0;

	printk("<=PCAM=> sr030pc40_sensor_release\n");

	kfree(sr030pc40_ctrl);

#ifdef CONFIG_LOAD_FILE
	sr030pc40_regs_table_exit();
#endif

	sr030pc40_set_power(0);
	return rc;
}

static int sr030pc40_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	sr030pc40_sensorw =
		kzalloc(sizeof(struct sr030pc40_work), GFP_KERNEL);

	if (!sr030pc40_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, sr030pc40_sensorw);
	sr030pc40_init_client(client);
	sr030pc40_client = client;

	printk("sr030pc40_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(sr030pc40_sensorw);
	sr030pc40_sensorw = NULL;
	CDBG("sr030pc40_probe failed!\n");
	return rc;
}

static const struct i2c_device_id sr030pc40_i2c_id[] = {
	{ "sr030pc40", 0},
	{ },
};

static struct i2c_driver sr030pc40_i2c_driver = {
	.id_table = sr030pc40_i2c_id,
	.probe  = sr030pc40_i2c_probe,
	.remove = __exit_p(sr030pc40_i2c_remove),
	.driver = {
		.name = "sr030pc40",
	},
};

static int sr030pc40_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
    unsigned char id;
	int rc = i2c_add_driver(&sr030pc40_i2c_driver);
	if (rc < 0 || sr030pc40_sensorw == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}
	
/*	sr030pc40_set_power(1);

    id = sr030pc40_check_sensor_rev();

    if(id != 0x96)
    {
        printk("<=PCAM=> WRONG SENSOR ID => id 0x%x \n", id);
        rc = -EINVAL;
    }
*/
	s->s_init = sr030pc40_sensor_init;
	s->s_release = sr030pc40_sensor_release;
	s->s_config  = sr030pc40_sensor_config;
	s->s_ext_config = sr030pc40_sensor_ext_config; // for samsung camsensor control
    
	s->s_camera_type = FRONT_CAMERA_2D;

probe_done:

	sr030pc40_set_power(0);
	
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __sr030pc40_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, sr030pc40_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __sr030pc40_probe,
	.driver = {
		.name = "msm_camera_sr030pc40",
		.owner = THIS_MODULE,
	},
};

static int __init sr030pc40_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(sr030pc40_init);
