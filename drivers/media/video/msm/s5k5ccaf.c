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

//PCAM 1/5" s5k5ccaf

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>


#include "s5k5ccaf.h"
#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/io.h>


//#define SENSOR_DEBUG 0


//#define CONFIG_LOAD_FILE


#define S5K5CCAF_BURST_WRITE_LIST(A)	s5k5ccaf_sensor_burst_write_list(A,(sizeof(A) / sizeof(A[0])),#A);
#define S5K5CCAF_WRITE_LIST(A)			s5k5ccaf_sensor_write_list(A,(sizeof(A) / sizeof(A[0])),#A);


#define CAM_MAIN_PWR		123
#define CAM_MAIN_STBY	96
#define CAM_MAIN_RST	0
#define CAM_SUB_PWR		124
#define CAM_SUB_STBY	1
#define CAM_SUB_RST		31
#define CAM_GPIO_SCL	2
#define CAM_GPIO_SDA	3

static char first_start_camera = 1;//  1 is not init a sensor
static char set_init0 = 0;
static char af_low_lux = 0;

static char mEffect = 0;
static char mBrightness = 0;
static char mContrast = 0;
static char mSaturation = 0;
static char mSharpness = 0;
static char mWhiteBalance = 0;
static char mISO = 0;
static char mAutoExposure = 0;
static char mScene = 0;
static char mAfMode = 0;
static char mDTP = 0;
static char mInit = 0;
static char mMode = 0;


struct s5k5ccaf_work {
	struct work_struct work;
};

static struct  s5k5ccaf_work *s5k5ccaf_sensorw;
static struct  i2c_client *s5k5ccaf_client;

struct s5k5ccaf_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};


static struct s5k5ccaf_ctrl *s5k5ccaf_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(s5k5ccaf_wait_queue);
DECLARE_MUTEX(s5k5ccaf_sem);
static int16_t s5k5ccaf_effect = CAMERA_EFFECT_OFF;

/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct s5k5ccaf_reg s5k5ccaf_regs;
extern int cpufreq_direct_set_policy(unsigned int cpu, const char *buf);
extern void pcam_msm_i2c_pwr_mgmt(struct i2c_adapter *adap, int on);
extern int* get_i2c_clock_addr(struct i2c_adapter *adap);
/*=============================================================*/

static int cam_hw_init(void);

void sensor_DTP_control(char value);

#ifdef CONFIG_LOAD_FILE
	static int s5k5ccaf_regs_table_write(char *name);
#endif





static int s5k5ccaf_sensor_read(unsigned short subaddr, unsigned short *data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_msg msg = { s5k5ccaf_client->addr, 0, 2, buf };
	
	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);

	ret = i2c_transfer(s5k5ccaf_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	msg.flags = I2C_M_RD;
	
	ret = i2c_transfer(s5k5ccaf_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) 
		goto error;

	*data = ((buf[0] << 8) | buf[1]);

error:
	return ret;
}

static int s5k5ccaf_sensor_write(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[4];
	struct i2c_msg msg = { s5k5ccaf_client->addr, 0, 4, buf };

//	printk("[PGH] on write func s5k5ccaf_client->addr : %x\n", s5k5ccaf_client->addr);
//	printk("[PGH] on write func  s5k5ccaf_client->adapter->nr : %d\n", s5k5ccaf_client->adapter->nr);


	buf[0] = (subaddr >> 8);
	buf[1] = (subaddr & 0xFF);
	buf[2] = (val >> 8);
	buf[3] = (val & 0xFF);

	return i2c_transfer(s5k5ccaf_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}



static int s5k5ccaf_sensor_write_list(struct samsung_short_t *list,int size, char *name)	
{
	int ret = 0;

#ifdef CONFIG_LOAD_FILE	
	ret = s5k5ccaf_regs_table_write(name);
#else
	int i = 0;

	//printk("<=PCAM=> s5k5ccaf_sensor_write_list : %s\n", name);


	for (i = 0; i < size; i++)
	{
	//	printk("[PGH] %x      %x\n", list[i].subaddr, list[i].value);

		if(list[i].subaddr == 0xffff)
		{
//			printk("<=PCAM=> now SLEEP!!!!\n  %d", list[i].value);
			msleep(list[i].value);
		}
		else
		{
		    if(s5k5ccaf_sensor_write(list[i].subaddr, list[i].value) < 0)
		    {
			    printk("<=PCAM=> sensor_write_list fail...-_-\n");
			    return -1;
		    }
		}
	}


#endif
	return ret;
}


#define BURST_MODE_BUFFER_MAX_SIZE 2700
unsigned char s5k5ccaf_buf_for_burstmode[BURST_MODE_BUFFER_MAX_SIZE];
static int s5k5ccaf_sensor_burst_write_list(struct samsung_short_t *list,int size, char *name)	
{
	int err = -EINVAL;
	int i = 0;
	int idx = 0;

	unsigned short subaddr = 0, next_subaddr = 0;
	unsigned short value = 0;

	struct i2c_msg msg = {  s5k5ccaf_client->addr, 0, 0, s5k5ccaf_buf_for_burstmode};
	


	for (i = 0; i < size; i++)
	{

		if(idx > (BURST_MODE_BUFFER_MAX_SIZE-10))
		{
			printk("<=PCAM=> BURST MODE buffer overflow!!!\n");
			 return err;
		}



		subaddr = list[i].subaddr;

		if(subaddr == 0x0F12)
			next_subaddr = list[i+1].subaddr;

		value = list[i].value;
		

		switch(subaddr)
		{

			case 0x0F12:
			{
				// make and fill buffer for burst mode write
				if(idx ==0) 
				{
					s5k5ccaf_buf_for_burstmode[idx++] = 0x0F;
					s5k5ccaf_buf_for_burstmode[idx++] = 0x12;
				}
				s5k5ccaf_buf_for_burstmode[idx++] = value>> 8;
				s5k5ccaf_buf_for_burstmode[idx++] = value & 0xFF;

			
			 	//write in burstmode	
				if(next_subaddr != 0x0F12)
				{
					msg.len = idx;
					err = i2c_transfer(s5k5ccaf_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
					//printk("s5k4ecgx_sensor_burst_write, idx = %d\n",idx);
					idx=0;
				}
				
			}
			break;

			case 0xFFFF:
			{
				msleep(value);
			}
			break;
		
			default:
			{
			    idx = 0;
			    err = s5k5ccaf_sensor_write(subaddr, value);
			}
			break;
			
		}

		
	}

	if (unlikely(err < 0))
	{
		printk("<=PCAM=>%s: register set failed\n",__func__);
		return err;
	}
	return 0;

}






void sensor_effect_control(char value)
{

	//int *i2c_clk_addr; //TEMP Dirty Code, Do not use it!
	//i2c_clk_addr = 0xd500c004;

	switch(value)
	{
		case EXT_CFG_EFFECT_NORMAL :{
		CAMDRV_DEBUG("EXT_CFG_EFFECT_NORMAL");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_effect_off);
		}
		break;		

		case EXT_CFG_EFFECT_NEGATIVE :{
		CAMDRV_DEBUG("EXT_CFG_EFFECT_NEGATIVE");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_effect_negative);
		}
		break;	
		
		case EXT_CFG_EFFECT_MONO :{
		CAMDRV_DEBUG("EXT_CFG_EFFECT_MONO");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_effect_mono);
		}
		break;	

		case EXT_CFG_EFFECT_SEPIA :{
		CAMDRV_DEBUG("EXT_CFG_EFFECT_SEPIA");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_effect_sepia);
		}
		break;	

		default :{
			printk("<=PCAM=> Unexpected Effect mode : %d\n",  value);
		}
		break;
				
	}

}


void sensor_whitebalance_control(char value)
{
	switch(value)
	{
		case EXT_CFG_WB_AUTO :{
		CAMDRV_DEBUG("EXT_CFG_WB_AUTO");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_wb_auto);
		}
		break;	

		case EXT_CFG_WB_DAYLIGHT :{
		CAMDRV_DEBUG("EXT_CFG_WB_DAYLIGHT");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_wb_daylight);
		}
		break;	

		case EXT_CFG_WB_CLOUDY :{
		CAMDRV_DEBUG("EXT_CFG_WB_CLOUDY");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_wb_cloudy);
		}
		break;	

		case EXT_CFG_WB_FLUORESCENT :{
		CAMDRV_DEBUG("EXT_CFG_WB_FLUORESCENT");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_wb_fluorescent);
		}
		break;	
		
		case EXT_CFG_WB_INCANDESCENT :{
		CAMDRV_DEBUG("EXT_CFG_WB_INCANDESCENT");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_wb_incandescent);
		}
		break;	

		default :{
			printk("<=PCAM=> Unexpected WB mode : %d\n",  value);
		}
		break;
		
	}// end of switch

}


void sensor_brightness_control(char value)
{
	switch(value)
	{
		case EXT_CFG_BR_STEP_P_4 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_4");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_p_4);
		}
		break;
		
		case EXT_CFG_BR_STEP_P_3 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_3");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_p_3);
		}
		break;

		case EXT_CFG_BR_STEP_P_2 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_2");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_p_2);
		}
		break;

		case EXT_CFG_BR_STEP_P_1 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_P_1");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_p_1);
		}
		break;

		case EXT_CFG_BR_STEP_0 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_0");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_0);
		}
		break;

		case EXT_CFG_BR_STEP_M_1 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_1");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_m_1);
		}
		break;

		case EXT_CFG_BR_STEP_M_2 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_2");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_m_2);
		}
		break;

		case EXT_CFG_BR_STEP_M_3 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_3");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_m_3);
		}
		break;

		case EXT_CFG_BR_STEP_M_4 :{
		CAMDRV_DEBUG("EXT_CFG_BR_STEP_M_4");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_brightness_m_4);
		}
		break;


		default :{
			printk("<=PCAM=> Unexpected BR mode : %d\n",  value);
		}
		break;

	}
	
}


void sensor_iso_control(char value)
{
	switch(value)
	{
		case EXT_CFG_ISO_AUTO :{
		CAMDRV_DEBUG("EXT_CFG_ISO_AUTO");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_iso_auto);
		}
		break;

		case EXT_CFG_ISO_50 :{
		CAMDRV_DEBUG("EXT_CFG_ISO_50");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_iso_50);
		}
		break;

		case EXT_CFG_ISO_100 :{
		CAMDRV_DEBUG("EXT_CFG_ISO_100");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_iso_100);
		}
		break;

		case EXT_CFG_ISO_200 :{
		CAMDRV_DEBUG("EXT_CFG_ISO_200");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_iso_200);
		}
		break;

		case EXT_CFG_ISO_400 :{
		CAMDRV_DEBUG("EXT_CFG_ISO_400");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_iso_400);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected ISO mode : %d\n",  value);
		}
		break;
		
	}

}


void sensor_metering_control(char value)
{
	switch(value)
	{
		case EXT_CFG_METERING_NORMAL :{
		CAMDRV_DEBUG("EXT_CFG_METERING_NORMAL");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_metering_normal);
		}
		break;
		
		case EXT_CFG_METERING_SPOT :{
		CAMDRV_DEBUG("EXT_CFG_METERING_SPOT");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_metering_spot);
		}
		break;

		case EXT_CFG_METERING_CENTER :{
		CAMDRV_DEBUG("EXT_CFG_METERING_CENTER");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_metering_center);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected METERING mode : %d\n",  value);
		}
		break;
	}

}


void sensor_scene_control(char value)
{
	switch(value)
	{
		case EXT_CFG_SCENE_OFF :{
		CAMDRV_DEBUG("EXT_CFG_SCENE_OFF");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
		}
		break;

		case EXT_CFG_SCENE_PORTRAIT :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
		CAMDRV_DEBUG("EXT_CFG_SCENE_PORTRAIT");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_portrait);
		}
		break;

		case EXT_CFG_SCENE_LANDSCAPE :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
		CAMDRV_DEBUG("EXT_CFG_SCENE_LANDSCAPE");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_landscape);
		}
		break;

		case EXT_CFG_SCENE_SPORTS :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
		CAMDRV_DEBUG("EXT_CFG_SCENE_SPORTS");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_sports);
		}
		break;

		case EXT_CFG_SCENE_PARTY :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);		
		CAMDRV_DEBUG("EXT_CFG_SCENE_PARTY");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_party);			
		}
		break;

		case EXT_CFG_SCENE_BEACH :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);		
		CAMDRV_DEBUG("EXT_CFG_SCENE_BEACH");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_beach);
		}
		break;

		case EXT_CFG_SCENE_SUNSET :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);	
		CAMDRV_DEBUG("EXT_CFG_SCENE_SUNSET");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_sunset);
		}
		break;
		
		case EXT_CFG_SCENE_DAWN :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);			
		CAMDRV_DEBUG("EXT_CFG_SCENE_DAWN");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_dawn);	
		}
		break;

		case EXT_CFG_SCENE_FALL :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);						
		CAMDRV_DEBUG("EXT_CFG_SCENE_FALL");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_fall);		
		}
		break;

		case EXT_CFG_SCENE_NIGHTSHOT :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);
		CAMDRV_DEBUG("EXT_CFG_SCENE_NIGHTSHOT");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_nightshot);			
		}
		break;

		case EXT_CFG_SCENE_BACKLIGHT :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);	
		CAMDRV_DEBUG("EXT_CFG_SCENE_BACKLIGHT");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_backlight);
		}
		break;

		case EXT_CFG_SCENE_FIREWORK :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);		
		CAMDRV_DEBUG("EXT_CFG_SCENE_FIREWORK");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_firework);				
		}
		break;

		case EXT_CFG_SCENE_TEXT :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);					
		CAMDRV_DEBUG("EXT_CFG_SCENE_TEXT");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_text);
		}
		break;

		case EXT_CFG_SCENE_CANDLE :{
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_off);							
		CAMDRV_DEBUG("EXT_CFG_SCENE_CANDLE");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_scene_candle);	
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected SCENE mode : %d\n",  value);
		}
		break;				
	}
}


void sensor_contrast_control(char value)
{
	switch(value)
	{
		case EXT_CFG_CR_STEP_M_2 :{
		CAMDRV_DEBUG("EXT_CFG_CR_STEP_M_2");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_contrast_m_2);
		}
		break;

		case EXT_CFG_CR_STEP_M_1 :{
		CAMDRV_DEBUG("EXT_CFG_CR_STEP_M_1");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_contrast_m_1);	
		}
		break;

		case EXT_CFG_CR_STEP_0 :{
		CAMDRV_DEBUG("EXT_CFG_CR_STEP_0");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_contrast_0);
		}
		break;

		case EXT_CFG_CR_STEP_P_1 :{
		CAMDRV_DEBUG("EXT_CFG_CR_STEP_P_1");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_contrast_p_1);
		}
		break;

		case EXT_CFG_CR_STEP_P_2 :{
		CAMDRV_DEBUG("EXT_CFG_CR_STEP_P_2");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_contrast_p_2);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected EXT_CFG_CR_CONTROL mode : %d\n",  value);
		}
		break;												
	}
}


void sensor_saturation_control(char value)
{
	switch(value)
	{
		case EXT_CFG_SA_STEP_M_2 :{
		CAMDRV_DEBUG("EXT_CFG_SA_STEP_M_2");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_saturation_m_2);
		}
		break;

		case EXT_CFG_SA_STEP_M_1 :{
		CAMDRV_DEBUG("EXT_CFG_SA_STEP_M_1");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_saturation_m_1);		
		}
		break;

		case EXT_CFG_SA_STEP_0 :{
		CAMDRV_DEBUG("EXT_CFG_SA_STEP_0");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_saturation_0);
		}
		break;

		case EXT_CFG_SA_STEP_P_1 :{
		CAMDRV_DEBUG("EXT_CFG_SA_STEP_P_1");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_saturation_p_1);
		}
		break;

		case EXT_CFG_SA_STEP_P_2 :{
		CAMDRV_DEBUG("EXT_CFG_SA_STEP_P_2");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_saturation_p_2);
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected EXT_CFG_SA_CONTROL mode : %d\n",  value);
		}
		break;					
	}

}


void sensor_sharpness_control(char value)
{
	switch(value)
	{
		case EXT_CFG_SP_STEP_M_2 :{
		CAMDRV_DEBUG("EXT_CFG_SP_STEP_M_2");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_sharpness_m_2);
		}
		break;

		case EXT_CFG_SP_STEP_M_1 :{
		CAMDRV_DEBUG("EXT_CFG_SP_STEP_M_1");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_sharpness_m_1);		
		}
		break;

		case EXT_CFG_SP_STEP_0 :{
		CAMDRV_DEBUG("EXT_CFG_SP_STEP_0");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_sharpness_0);	
		}
		break;

		case EXT_CFG_SP_STEP_P_1 :{
		CAMDRV_DEBUG("EXT_CFG_SP_STEP_P_1");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_sharpness_p_1);			
		}
		break;

		case EXT_CFG_SP_STEP_P_2 :{
		CAMDRV_DEBUG("EXT_CFG_SP_STEP_P_2");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_sharpness_p_2);			
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected EXT_CFG_SP_CONTROL mode : %d\n",  value);
		}
		break;					
	}
}


int sensor_af_control(char value)
{
	switch(value)
	{

		case EXT_CFG_AF_CHECK_STATUS :{
		unsigned short af_status;
		CAMDRV_DEBUG("EXT_CFG_AF_CHECK_STATUS\n");
		s5k5ccaf_sensor_write(0x002C, 0x7000);
		s5k5ccaf_sensor_write(0x002E, 0x2D12);
		s5k5ccaf_sensor_read(0x0F12, &af_status);

		switch( af_status ){
		case 0:
		case 3:
		case 4:
		case 6:
		case 8:
			return EXT_CFG_AF_LOWCONF;
		case 1 :
			return EXT_CFG_AF_PROGRESS;
		case 2 :
			return EXT_CFG_AF_SUCCESS;
		case 9 :
			return EXT_CFG_AF_CANCELED;
		case 10 :
			return EXT_CFG_AF_TIMEOUT;
		default:
			return af_status;
			
		}
		}
		break;
		
		case EXT_CFG_AF_OFF :{
		CAMDRV_DEBUG("EXT_CFG_AF_OFF");			
		S5K5CCAF_WRITE_LIST(s5k5ccaf_af_off);			
		}
		break;

		case EXT_CFG_AF_SET_NORMAL :{
		CAMDRV_DEBUG("EXT_CFG_AF_SET_NORMAL");						
		S5K5CCAF_WRITE_LIST(s5k5ccaf_af_normal_on); 						
		}
		break;

		case EXT_CFG_AF_SET_MACRO :{
		CAMDRV_DEBUG("EXT_CFG_AF_SET_MACRO");						
		S5K5CCAF_WRITE_LIST(s5k5ccaf_af_macro_on);									
		}
		break;

		case EXT_CFG_AF_DO :{

		unsigned short	msb, lsb;
		int cur_lux;

		CAMDRV_DEBUG("EXT_CFG_AF_DO");

		af_low_lux = 0;
		msb = lsb = 0;
		cur_lux = 0;

		s5k5ccaf_sensor_write(0xFCFC, 0xD000);
		s5k5ccaf_sensor_write(0x002C, 0x7000);
		s5k5ccaf_sensor_write(0x002E, 0x2A3C);
		s5k5ccaf_sensor_read(0x0F12, &lsb);
		s5k5ccaf_sensor_read(0x0F12, &msb);

		cur_lux = (msb<<16) | lsb;
		//printk("cur_lux : 0x%08x on AF_DO\n", cur_lux);

		if(cur_lux > 0xFFFE)
			;
		else if(cur_lux < 0x0020)
		{
			printk("<=PCAM=> low lux AF ~~\n");
			af_low_lux = 1;
		}
		else
			;
		


		S5K5CCAF_WRITE_LIST(s5k5ccaf_af_do);												
		}
		break;
	
		case EXT_CFG_AF_ABORT : {
		CAMDRV_DEBUG("EXT_CFG_AF_ABORT\n");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_af_abort);
		}
		break;

		case EXT_CFG_AF_CHECK_2nd_STATUS :{
		unsigned short af_status;
		//printk("EXT_CFG_AF_2ND_CHECK_STATUS");
		s5k5ccaf_sensor_write(0x002C, 0x7000);
		s5k5ccaf_sensor_write(0x002E, 0x1F2F);
		s5k5ccaf_sensor_read(0x0F12, &af_status);
		CAMDRV_DEBUG("<=PCAM=> AF 2nd check status : %x\n", af_status);
		return af_status;
		}
		break;


		default :{
			printk("<=PCAM=> unexpected AF command : %d\n",value);
		}		
		break;
		
	}	
	return 0;
}



void sensor_DTP_control(char value)
{
	switch(value)
	{
		case EXT_CFG_DTP_OFF:{
		CAMDRV_DEBUG("DTP OFF");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_dtp_off);
		}
		break;

		case EXT_CFG_DTP_ON:{
		CAMDRV_DEBUG("DTP ON");
		S5K5CCAF_WRITE_LIST(s5k5ccaf_dtp_on);
		}
		break;

		default:{
		printk("<=PCAM=> unexpected DTP control on PCAM\n");
		}
		break;
	}
}





static int s5k5ccaf_sensor_ext_config(void __user *arg)
{
	long   rc = 0;
	ioctl_pcam_info_8bit		ctrl_info;


	CAMDRV_DEBUG("START");


	if(copy_from_user((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail copy_from_user!\n", __func__);
	}

/*
	printk("<=PCAM=> TEST %d %d %d %d %d \n", ctrl_info.mode, ctrl_info.address,\
	 ctrl_info.value_1, ctrl_info.value_2, ctrl_info.value_3);
*/

	switch(ctrl_info.mode)
	{
		case EXT_CFG_AUTO_TUNNING:
		break;

		
		case EXT_CFG_SDCARD_DETECT:
		break;

		case EXT_CFG_GET_INFO:{
			unsigned short lsb, msb, rough_iso;					
			s5k5ccaf_sensor_write(0xFCFC, 0xD000);					
			s5k5ccaf_sensor_write(0x002C, 0x7000);
			s5k5ccaf_sensor_write(0x002E, 0x2A14);
			s5k5ccaf_sensor_read(0x0F12, &lsb); 		//0x2A14
			s5k5ccaf_sensor_read(0x0F12, &msb);		//0x2A16
			s5k5ccaf_sensor_read(0x0F12, &rough_iso); //0x2A8
													
			ctrl_info.value_1 = lsb;					
			ctrl_info.value_2 = msb;	
			//printk("<=PCAM=> exposure %x %x \n", lsb, msb);
			ctrl_info.value_3 = rough_iso;	
			//printk("<=PCAM=> rough_iso %x \n", rough_iso);			
		}
		break;

		case EXT_CFG_FRAME_CONTROL:
		{
			switch(ctrl_info.value_1)
			{

				case EXT_CFG_FRAME_AUTO :{
				CAMDRV_DEBUG("EXT_CFG_FRAME_AUTO");		
				}					
				break;
				
				case EXT_CFG_FRAME_FIX_15 :{
				CAMDRV_DEBUG("EXT_CFG_FRAME_FIX_15");
				S5K5CCAF_WRITE_LIST(s5k5ccaf_15_fixed_fps);		
				}
				break;

				case EXT_CFG_FRAME_FIX_24 :{
				CAMDRV_DEBUG("EXT_CFG_FRAME_FIX_24");
				S5K5CCAF_WRITE_LIST(s5k5ccaf_24_fixed_fps);		
				}
				break;
                
				case EXT_CFG_FRAME_FIX_30 :{
				CAMDRV_DEBUG("EXT_CFG_FRAME_FIX_30");
				S5K5CCAF_WRITE_LIST(s5k5ccaf_30_fixed_fps);
				}
				break;


				default :{
					printk("<=PCAM=> Unexpected EXT_CFG_FRAME_CONTROL mode : %d\n", ctrl_info.value_1);
				}
				break;				
			
			}
		}
		break;


		case EXT_CFG_AF_CONTROL:
		{
			mAfMode= ctrl_info.value_1;
			ctrl_info.value_3 = sensor_af_control(mAfMode);
		}
		break;

		
		case EXT_CFG_EFFECT_CONTROL:
		{
			mEffect = ctrl_info.value_1;
			sensor_effect_control(mEffect);
				
			
		}// end of EXT_CFG_EFFECT_CONTROL
		break;


		case EXT_CFG_WB_CONTROL:
		{
			mWhiteBalance = ctrl_info.value_1;
			sensor_whitebalance_control(mWhiteBalance);
			

		}//end of EXT_CFG_WB_CONTROL
		break;


		case EXT_CFG_BR_CONTROL:
		{
			mBrightness = ctrl_info.value_1;
			if(mInit)
				sensor_brightness_control(mBrightness);
			
		}//end of EXT_CFG_BR_CONTROL
		break;

		case EXT_CFG_ISO_CONTROL:
		{
			mISO = ctrl_info.value_1;
			sensor_iso_control(mISO);
	
		}
		break;


		case EXT_CFG_METERING_CONTROL:
		{
			mAutoExposure = ctrl_info.value_1;
			sensor_metering_control(mAutoExposure);
			
		}//end of case
		break;


		case EXT_CFG_SCENE_CONTROL:
		{
			mScene = ctrl_info.value_1;
			sensor_scene_control(mScene);

		}
		break;


		case EXT_CFG_AE_AWB_CONTROL:
		{
			switch(ctrl_info.value_1)
			{
				case EXT_CFG_AE_LOCK :
                    CAMDRV_DEBUG("EXT_CFG_AE_LOCK");
					S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_lock);
                    break;
                    
                case EXT_CFG_AE_UNLOCK :
                    CAMDRV_DEBUG("EXT_CFG_AE_UNLOCK");
					S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_unlock);
                    break;
                    
                case EXT_CFG_AWB_LOCK :
                    CAMDRV_DEBUG("EXT_CFG_AWB_LOCK");
					S5K5CCAF_WRITE_LIST(s5k5ccaf_awb_lock);
                    break;
                    
                case EXT_CFG_AWB_UNLOCK :
                    CAMDRV_DEBUG("EXT_CFG_AWB_UNLOCK");
					S5K5CCAF_WRITE_LIST(s5k5ccaf_awb_unlock);
                    break;
                    
				case EXT_CFG_AE_AWB_LOCK :{
				CAMDRV_DEBUG("EXT_CFG_AWB_AE_LOCK");

				if(mWhiteBalance == 0)
				{
					S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_lock);
					S5K5CCAF_WRITE_LIST(s5k5ccaf_awb_lock);
				}
				else
				{
					S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_lock);
				}
				}
				break;

				case EXT_CFG_AE_AWB_UNLOCK :{
				CAMDRV_DEBUG("EXT_CFG_AWB_AE_UNLOCK");
				if(mWhiteBalance == 0)
				{
					S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_unlock);
					S5K5CCAF_WRITE_LIST(s5k5ccaf_awb_unlock);
				}
				else
				{
					S5K5CCAF_WRITE_LIST(s5k5ccaf_ae_unlock);
				}
				}
				break;

				default :{
					printk("<=PCAM=> Unexpected AWB_AE mode : %d\n", ctrl_info.value_1);
				}
				break;						
				
			}
		}
		break;
			
		case EXT_CFG_CR_CONTROL:
		{
			mContrast = ctrl_info.value_1;
			if(mInit)
				sensor_contrast_control(mContrast);

		}
		break;
			

		case EXT_CFG_SA_CONTROL:
		{
			mSaturation = ctrl_info.value_1;
			if(mInit)
				sensor_saturation_control(mSaturation);
			
		}
		break;

		

		case EXT_CFG_SP_CONTROL:
		{
			mSharpness = ctrl_info.value_1;
			if(mInit)
				sensor_sharpness_control(mSharpness);
			
		}
		break;

		case EXT_CFG_CPU_CONTROL:
		{

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
		}
		break;

		case EXT_CFG_DTP_CONTROL:
		{
			if(mInit == 0)
			{
				if(ctrl_info.value_1 == 0)
					ctrl_info.value_3 = 2;

				else if(ctrl_info.value_1 == 1)
					ctrl_info.value_3 = 3;

				mDTP = 1;
			}

			else
			{
				sensor_DTP_control(ctrl_info.value_1);

				if(ctrl_info.value_1 == 0)
					ctrl_info.value_3 = 2;

				else if(ctrl_info.value_1 == 1)
					ctrl_info.value_3 = 3;
			
				mDTP = 0;
			}

		}
		break;

		case EXT_CFG_PREVIEW_SIZE_CONTROL:
           
		{
			mMode = ctrl_info.value_1;
		}
		break;

		case EXT_CFG_GET_MODULE_STATUS:
		{
			unsigned short	id = 0; //CAM FOR FW
			//ctrl_info.value_3 = gpio_get_value(0);

			s5k5ccaf_sensor_write(0x002C, 0x0000);
			s5k5ccaf_sensor_write(0x002E, 0x0040);
			s5k5ccaf_sensor_read(0x0F12, &id);
			
			ctrl_info.value_3 = id;
			
			printk("<=PCAM=> check current module status : %x\n", ctrl_info.value_3);
			printk("<=PCAM=> PINON/OFF : %d\n", gpio_get_value(0));
		}
		break;

		default :{
			printk("<=PCAM=> Unexpected mode on sensor_rough_control : %d\n", ctrl_info.mode);
		}
		break;
	}



	if(copy_to_user((void *)arg, (const void *)&ctrl_info, sizeof(ctrl_info)))
	{
		printk("<=PCAM=> %s fail on copy_to_user!\n", __func__);
	}
	return rc;
}



void s5k5ccaf_set_power(int status)
{
    struct vreg *vreg_cam_out8_vddio;
    struct vreg *vreg_cam_out9_vdda;
    struct vreg *vreg_cam_out10_af; 

    vreg_cam_out8_vddio		= vreg_get(NULL, "ldo8");
    vreg_cam_out9_vdda		= vreg_get(NULL, "ldo9");
    vreg_cam_out10_af			= vreg_get(NULL, "ldo10");
    
	if(status == 1) //POWER ON
	{
		CAMDRV_DEBUG("s5k5ccaf_set_power : POWER ON");
		
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
		udelay(20); // >20us
        gpio_set_value(CAM_SUB_PWR, 1);
        udelay(15); // >15us
		vreg_enable(vreg_cam_out8_vddio);
		udelay(15); // >15us
		vreg_enable(vreg_cam_out10_af);
		udelay(20); // >20us
		gpio_set_value(CAM_SUB_STBY, 1);
		mdelay(4); // >4ms

		//msm_camio_clk_enable(CAMIO_VFE_CLK);
		msm_camio_clk_rate_set(24000000);
		//msm_camio_camif_pad_reg_reset();
		//gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		udelay(10); // >10us

		gpio_set_value(CAM_SUB_RST, 1);
		mdelay(1); // >1ms

		gpio_set_value(CAM_SUB_STBY, 0);
#ifdef CONFIG_SR030PC40
		mdelay(5); // >5ms
		gpio_set_value(CAM_SUB_STBY, 1);
#endif
		udelay(10); // >10us
    	
		gpio_set_value(CAM_MAIN_STBY, 1);
		udelay(15); // >15us
    	
		gpio_set_value(CAM_MAIN_RST, 1);
		mdelay(50); // >50ms
        
	}
	else //POWER OFF
	{
		CAMDRV_DEBUG("s5k5ccaf_set_power : POWER OFF");

		gpio_set_value(CAM_MAIN_RST, 0);
		udelay(50); // >50us
        
		//msm_camio_camif_pad_reg_reset();
		//msm_camio_clk_disable(CAMIO_VFE_CLK);

		gpio_set_value(CAM_MAIN_STBY, 0);
		gpio_set_value(CAM_SUB_RST, 0);
		gpio_set_value(CAM_SUB_STBY, 0);
        
		vreg_disable(vreg_cam_out10_af);			
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



static int cam_hw_init()
{

	int rc = 0;
	unsigned short	id = 0; //CAM FOR FW
//	unsigned int	before_time, after_time, i;//I2C SPEED TEST

	CAMDRV_DEBUG("next cam_hw_init");

	s5k5ccaf_sensor_write(0x002C, 0x0000);
	s5k5ccaf_sensor_write(0x002E, 0x0040);
	s5k5ccaf_sensor_read(0x0F12, &id);

	if(id != 0x05CC)
	{
		printk("<=PCAM=> WRONG SENSOR FW => id 0x%x \n", id);
		rc = -1;
	}
	else
		printk("<=PCAM=> CURRENT REAR SENSOR ID => id 0x%x \n", id);

	return rc;
}







static long s5k5ccaf_set_effect(int mode, int effect)
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
	s5k5ccaf_effect = effect;
	
	return rc;
}

static long s5k5ccaf_set_sensor_mode(int mode)
{
	//long rc = 0;
	unsigned short	msb, lsb;
	int cur_lux;
	char vsync_value, cnt;

	unsigned int	before_time, after_time, i, time_gab;//I2C SPEED TEST

	//int *i2c_clk_addr; //TEMP Dirty Code, Do not use it!

	CAMDRV_DEBUG("START");
	//i2c_clk_addr = get_i2c_clock_addr(s5k5ccaf_client->adapter);
	

	if(first_start_camera)
	{
		printk("<=PCAM=> camera init for BENNET ver0.3");

		//i2c_clk_addr = 0xd500c004;
		//printk("<=PCAM=> i2c clk :%x\n", readl(i2c_clk_addr));

		before_time = get_jiffies_64();

		#if defined(CONFIG_LOAD_FILE)
		S5K5CCAF_WRITE_LIST(s5k5ccaf_init0);
		#else
		S5K5CCAF_BURST_WRITE_LIST(s5k5ccaf_init0);	
		#endif

		after_time = get_jiffies_64();
		time_gab = jiffies_to_msecs(after_time-before_time);
/*
		if(time_gab > 280)
		{
			printk("<=PCAM=> i2c speed is going down : %dmsec\n", time_gab);

			pcam_msm_i2c_pwr_mgmt(s5k5ccaf_client->adapter, 1);
			//this funcion have to call after clk_enable by PCAM, do not use it without pcam's allowance
			printk("<=PCAM=> i2c clk set forcely 0x316\n");
			writel(0x316, i2c_clk_addr);
			printk("<=PCAM=> re-check i2c clk :%x\n", readl(i2c_clk_addr));

		    }
		else*/
		{
			printk("<=PCAM=> init speed is fine : %dmsec\n", time_gab);
		}
		first_start_camera = 0;
		mInit = 1;
	} 



	switch (mode) {
	case SENSOR_PREVIEW_MODE:
	{
		CAMDRV_DEBUG("PREVIEW~~~");

		/*
		if(mMode)
		{
			printk("<=PCAM=> now CAMCORDER MODE~~~~~~~~~\n");	
			S5K5CCAF_WRITE_LIST(s5k5ccaf_camcorder_set);	
			S5K5CCAF_WRITE_LIST(s5k5ccaf_fps_30fix);	
		}
		else
		{
			printk("<=PCAM=> now CAMERA MODE~~~~~~~~~\n");
		}
		*/


		if((mScene == EXT_CFG_SCENE_NIGHTSHOT) ||(mScene == EXT_CFG_SCENE_FIREWORK) )
		{
			for(cnt=0; cnt<1200; cnt++)
			{
				vsync_value = gpio_get_value(14);

				if(vsync_value)
					break;
				else
				{
					CAMDRV_DEBUG(" wait cnt:%d vsync_value:%d\n", cnt, vsync_value);
					msleep(1);
				}
		
			}

	
		}

		if(mDTP == 1)
		{
			S5K5CCAF_WRITE_LIST(s5k5ccaf_dtp_on);		
		}
		else
		{
			S5K5CCAF_WRITE_LIST(s5k5ccaf_preview);	
		}


		if(mScene == EXT_CFG_SCENE_OFF)
		sensor_effect_control(mEffect);

		if((set_init0 ==1) &&(mScene == EXT_CFG_SCENE_OFF))
			printk("<=PCAM=> skip scene off\n");
		else
			sensor_scene_control(mScene);

		//if(mScene != EXT_CFG_SCENE_SPORTS && mScene != EXT_CFG_SCENE_NIGHTSHOT)
		//	sensor_iso_control(mISO);			

		if(mScene == EXT_CFG_SCENE_OFF)
		{
		    sensor_brightness_control(mBrightness);
		    sensor_metering_control(mAutoExposure);
		    sensor_contrast_control(mContrast);
		    sensor_saturation_control(mSaturation);
		    sensor_sharpness_control(mSharpness);

		    if((set_init0 ==1) &&(mWhiteBalance == EXT_CFG_WB_AUTO))
			    printk("<=PCAM=> awb aleady done!\n");
		    else 
			    sensor_whitebalance_control(mWhiteBalance);

		    sensor_af_control(mAfMode);
		}

		
	}
		break;
		
	case SENSOR_SNAPSHOT_MODE:
	{
		CAMDRV_DEBUG("SNAPSHOT~~~");

		msb = lsb = 0;
		cur_lux = 0;

		s5k5ccaf_sensor_write(0xFCFC, 0xD000);
		s5k5ccaf_sensor_write(0x002C, 0x7000);
		s5k5ccaf_sensor_write(0x002E, 0x2A3C);
		s5k5ccaf_sensor_read(0x0F12, &lsb);
		s5k5ccaf_sensor_read(0x0F12, &msb);

		cur_lux = (msb<<16) | lsb;
		printk("cur_lux : 0x%08x \n", cur_lux);


		for(cnt=0; cnt<700; cnt++)
		{
			vsync_value = gpio_get_value(14);

			if(vsync_value)
				break;
			else
			{
				printk(" on snapshot,  wait cnt:%d vsync_value:%d\n", cnt, vsync_value);			
				CAMDRV_DEBUG(" on snapshot,  wait cnt:%d vsync_value:%d\n", cnt, vsync_value);
				msleep(3);
			}
	
		}


		if(cur_lux > 0xFFFE)
		{
			CAMDRV_DEBUG("HighLight Snapshot!\n");
			printk("HighLight Snapshot!\n");
			S5K5CCAF_WRITE_LIST(s5k5ccaf_high_snapshot);
			if(af_low_lux)
			{
				printk("<=PCAM=> low lux add extra delay \n");
				msleep(200);
			}	
		}
		else if(cur_lux < 0x0020)
		{
			if((mScene == EXT_CFG_SCENE_NIGHTSHOT) ||(mScene == EXT_CFG_SCENE_FIREWORK) )
			{
				CAMDRV_DEBUG("Night or Firework  Snapshot!\n");
				printk("Night or Firework Snapshot!\n");			
				S5K5CCAF_WRITE_LIST(s5k5ccaf_night_snapshot);	
				msleep(300);	
			}
			else
			{
				CAMDRV_DEBUG("LowLight Snapshot delay ~~~~!\n");
				printk("LowLight Snapshot 500...sec!\n");
				S5K5CCAF_WRITE_LIST(s5k5ccaf_lowlight_snapshot);
				msleep(250);	
			}
		}
		else
		{
			CAMDRV_DEBUG("Normal Snapshot!\n");
			printk("Normal Snapshot!\n");		
			S5K5CCAF_WRITE_LIST(s5k5ccaf_snapshot);
			if(af_low_lux)
			{
				printk("<=PCAM=> af_lux_low add extra delay \n");
				msleep(200);
			}	

		}		
		
	}
	break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		CAMDRV_DEBUG("RAW_SNAPSHOT NOT SUPPORT!!");
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int s5k5ccaf_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	#ifndef CONFIG_LOAD_FILE
	rc = S5K5CCAF_WRITE_LIST(s5k5ccaf_pre_init0);
	#endif
	mdelay(10);

	return rc;
}

#ifdef CONFIG_LOAD_FILE

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

static char *s5k5ccaf_regs_table = NULL;

static int s5k5ccaf_regs_table_size;

void s5k5ccaf_regs_table_init(void)
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
	filp = filp_open("/data/camera/s5k5ccaf.h", O_RDONLY, 0);
#else
	filp = filp_open("/mnt/sdcard/s5k5ccaf.h", O_RDONLY, 0);
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

	s5k5ccaf_regs_table = dp;
	
	s5k5ccaf_regs_table_size = l;

	*((s5k5ccaf_regs_table + s5k5ccaf_regs_table_size) - 1) = '\0';

//	printk("s5k5ccaf_regs_table 0x%x, %ld\n", dp, l);
}

void s5k5ccaf_regs_table_exit(void)
{
	printk("%s %d\n", __func__, __LINE__);
	if (s5k5ccaf_regs_table) {
		kfree(s5k5ccaf_regs_table);
		s5k5ccaf_regs_table = NULL;
	}	
}

static int s5k5ccaf_regs_table_write(char *name)
{
	char *start, *end, *reg;//, *data;	
	unsigned short addr, value;
	char reg_buf[7], data_buf[7];

	addr = value = 0;

	*(reg_buf + 6) = '\0';
	*(data_buf + 6) = '\0';

	start = strstr(s5k5ccaf_regs_table, name);
	
	end = strstr(start, "};");

	while (1) {	
		/* Find Address */	
		reg = strstr(start,"{0x");		
		if (reg)
			start = (reg + 16);
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */	
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 1), 6);	
			memcpy(data_buf, (reg + 9), 6);	
			addr = (unsigned short)simple_strtoul(reg_buf, NULL, 16); 
			value = (unsigned short)simple_strtoul(data_buf, NULL, 16); 
//			printk("addr 0x%04x, value 0x%04x\n", addr, value);
			if (addr == 0xffff)
			{
				msleep(value);
				printk("delay 0x%04x, value 0x%04x\n", addr, value);
			}	
			else
			{
				if( s5k5ccaf_sensor_write(addr, value) < 0 )
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



int s5k5ccaf_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;
	int i = 0;
	

	s5k5ccaf_ctrl = kzalloc(sizeof(struct s5k5ccaf_ctrl), GFP_KERNEL);
	if (!s5k5ccaf_ctrl) {
		CDBG("s5k5ccaf_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		s5k5ccaf_ctrl->sensordata = data;

	s5k5ccaf_set_power(1);
	
	rc = cam_hw_init();
	if(rc < 0)
	{
		printk("<=PCAM=> cam_hw_init failed, try again!\n");
		//goto init_fail;


		for(i=0; i<500; i++)
		{
			s5k5ccaf_set_power(0);
			s5k5ccaf_set_power(1);
			msleep(1);
			if(cam_hw_init()==0)
			{
				printk("<=PCAM=> cam_hw_init retry success!\n");
				break;
			}
			printk(" try again %d times~\n", i);
		}
	}



#ifdef CONFIG_LOAD_FILE
	s5k5ccaf_regs_table_init();
#endif	

	rc = s5k5ccaf_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("s5k5ccaf_sensor_init failed!\n");
		goto init_fail;
	}

init_done:
	return rc;

init_fail:
	kfree(s5k5ccaf_ctrl);
	return rc;
}

static int s5k5ccaf_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&s5k5ccaf_wait_queue);
	return 0;
}

int s5k5ccaf_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	if (copy_from_user(&cfg_data,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&s5k5ccaf_sem); */

	CDBG("s5k5ccaf_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

		switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = s5k5ccaf_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = s5k5ccaf_set_effect(cfg_data.mode,
						cfg_data.cfg.effect);
			break;

		case CFG_GET_AF_MAX_STEPS:
		default:
			rc = -EINVAL;
			break;
		}

	/* up(&s5k5ccaf_sem); */

	return rc;
}

int s5k5ccaf_sensor_release(void)
{
	int rc = 0;
        //int *switch_i2c_addr; //TEMP Dirty Code, Do not use it!


	first_start_camera = 1;
	set_init0 = 0;

	//If did not init below that, it can keep the previous status. it depend on concept by PCAM
	mEffect = 0;
	mBrightness = 0;
	mContrast = 0;
	mSaturation = 0;
	mSharpness = 0;
	mWhiteBalance = 0;
	mISO = 0;
	mAutoExposure = 0;
	mScene = 0;
	//mAfMode = 0;
	mDTP = 0;
	mInit = 0;


	//TEMP Dirty Code, Do not use it! PCAM...
        //switch_i2c_addr = 0xd500c010;
        //writel(1, switch_i2c_addr);
	
	if(mAfMode == EXT_CFG_AF_SET_MACRO)
	{
		printk("<=PCAM=> wait change lens position\n");
		sensor_af_control(EXT_CFG_AF_SET_NORMAL);
		msleep(150);
	}
	else
		printk("<=PCAM=> mAfMode : %d\n", mAfMode);

	printk("<=PCAM=> s5k5ccaf_sensor_release\n");

	kfree(s5k5ccaf_ctrl);
	/* up(&s5k5ccaf_sem); */

#ifdef CONFIG_LOAD_FILE
	s5k5ccaf_regs_table_exit();
#endif

	s5k5ccaf_set_power(0);
	return rc;
}

static int s5k5ccaf_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;


	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	s5k5ccaf_sensorw =
		kzalloc(sizeof(struct s5k5ccaf_work), GFP_KERNEL);

	if (!s5k5ccaf_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, s5k5ccaf_sensorw);
	s5k5ccaf_init_client(client);
	s5k5ccaf_client = client;


	CDBG("s5k5ccaf_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(s5k5ccaf_sensorw);
	s5k5ccaf_sensorw = NULL;
	CDBG("s5k5ccaf_probe failed!\n");
	return rc;
}

static const struct i2c_device_id s5k5ccaf_i2c_id[] = {
	{ "s5k5ccaf", 0},
	{ },
};

static struct i2c_driver s5k5ccaf_i2c_driver = {
	.id_table = s5k5ccaf_i2c_id,
	.probe  = s5k5ccaf_i2c_probe,
	.remove = __exit_p(s5k5ccaf_i2c_remove),
	.driver = {
		.name = "s5k5ccaf",
	},
};

static int s5k5ccaf_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{

	int rc = i2c_add_driver(&s5k5ccaf_i2c_driver);
	if (rc < 0 || s5k5ccaf_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

/*	s5k5ccaf_set_power(1);
	printk("<=PCAM=> s5k5ccaf_set_power : true\n");
    
	rc = s5k5ccaf_sensor_init_probe(info);
	if (rc < 0)
	{
        printk("<=PCAM=> s5k5ccaf_sensor_init_probe : fail\n");
		goto probe_done;
	}*/

	s->s_init = s5k5ccaf_sensor_init;
	s->s_release = s5k5ccaf_sensor_release;
	s->s_config  = s5k5ccaf_sensor_config;
	s->s_ext_config = s5k5ccaf_sensor_ext_config;

    s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle = 90;
	

probe_done:

	s5k5ccaf_set_power(0);
	
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __s5k5ccaf_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, s5k5ccaf_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __s5k5ccaf_probe,
	.driver = {
		.name = "msm_camera_s5k5ccaf",
		.owner = THIS_MODULE,
	},
};

static int __init s5k5ccaf_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(s5k5ccaf_init);
