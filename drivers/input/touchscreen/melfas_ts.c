/* drivers/input/touchscreen/melfas_ts.c
 *
 * Copyright (C) 2010 Melfas, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input/melfas_ts.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/i2c/tsp_gpio.h>

#define P7			0
#define P5			1
#define N1			2
#define CHIEF			3
#define JUNO			4
#define TELESTO		5

#define MODEL		JUNO

#define MMS152_MAX_TOUCH 	10
#define MMS144_MAX_TOUCH	5
#define MMS136_MAX_TOUCH	5
#define MMS128_MAX_TOUCH	5

#define TS_MAX_Z_TOUCH		255
#define TS_MAX_W_TOUCH		30

#define MTSI_VERSION			0x08

#if MTSI_VERSION > 0x06
#define TS_READ_REGS_LEN	6
#else //MTSI_VERSION
#define TS_READ_REGS_LEN	5
#endif //MTSI_VERSION

#if MODEL == CHIEF
#define TS_MAX_X_COORD			320
#define TS_MAX_Y_COORD			480

#define TS_READ_START_ADDR		0x10
#define TS_READ_VERSION_ADDR	0x31

#define HW_VERSION				0x00
#define FW_VERSION				0x01

#define MELFAS_MAX_TOUCH		MMS136_MAX_TOUCH
#elif MODEL == P7
#define TS_MAX_X_COORD			800
#define TS_MAX_Y_COORD			1280

#define TS_READ_START_ADDR		0x10
#define TS_READ_VERSION_ADDR	0x31

#define HW_VERSION				0x00
#define FW_VERSION				0x01

#define MELFAS_MAX_TOUCH		MMS152_MAX_TOUCH
#elif MODEL == P5
#define TS_MAX_X_COORD			800
#define TS_MAX_Y_COORD			1280

#define TS_READ_START_ADDR		0x10
#define TS_READ_VERSION_ADDR	0x31

#define HW_VERSION				0x00
#define FW_VERSION				0x01

#define MELFAS_MAX_TOUCH		MMS152_MAX_TOUCH
#elif MODEL == N1
#define TS_MAX_X_COORD			480
#define TS_MAX_Y_COORD			800

#define TS_READ_START_ADDR		0x10
#define TS_READ_VERSION_ADDR	0x31

#define HW_VERSION				0x00
#define FW_VERSION				0x01

#define MELFAS_MAX_TOUCH		MMS136_MAX_TOUCH
#elif MODEL == JUNO
#define TS_MAX_X_COORD			320
#define TS_MAX_Y_COORD			480

#define TS_READ_START_ADDR		0x10
#define TS_READ_MODULE_VERSION_ADDR	0x30
#define TS_READ_FIRMWARE_VERSION_ADDR	0x31

#define HW_VERSION				0x00
#define FW_VERSION				0x04

#define MELFAS_MAX_TOUCH		MMS136_MAX_TOUCH
#elif MODEL == TELESTO
#define TS_MAX_X_COORD			320
#define TS_MAX_Y_COORD			480

#define TS_READ_START_ADDR		0x10
#define TS_READ_VERSION_ADDR	0x31

#define HW_VERSION				0x00
#define FW_VERSION				0x01

#define MELFAS_MAX_TOUCH		MMS128_MAX_TOUCH
#endif // MODEL

#define I2C_RETRY_CNT			10
#define DOWNLOAD_RETRY_CNT      5

#define PRESS_KEY				1
#define RELEASE_KEY				0

#define DEBUG_PRINT				1

#define SET_DOWNLOAD_BY_GPIO	1

#if SET_DOWNLOAD_BY_GPIO
#include <melfas_download.h>
#endif // SET_DOWNLOAD_BY_GPIO

enum
{
	None = 0,
	TOUCH_SCREEN,
	TOUCH_KEY
};

struct muti_touch_info
{
	int strength;
	int width;	
	int posX;
	int posY;
	int reportID;
};

struct melfas_ts_data 
{
	uint16_t addr;
	struct i2c_client *client; 
	struct input_dev *input_dev;
	struct work_struct  work;
	uint32_t flags;
	int (*power)(int on);
	struct early_suspend early_suspend;
};
/////////////////////////////// for 3x4 input method //////////////////////////////////////////////////////////////////////
struct melfas_ts_data *ts_global;
static int touchkey_status[2];

#define TK_STATUS_PRESS		1
#define TK_STATUS_RELEASE		0

static int firmware_ret_val = -1;
#if defined(CONFIG_MACH_JUNO_SKT) || defined(CONFIG_MACH_JUNO_KT)
static int jump_limit_X  = 180;
static int jump_limit_Y  = 180;
#endif

struct class *touch_class;
EXPORT_SYMBOL(touch_class);

struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);

#if defined(CONFIG_MACH_JUNO_SKT) || defined(CONFIG_MACH_JUNO_KT)
struct device *set_tsp_dev;
EXPORT_SYMBOL(set_tsp_dev);
#endif


int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size);

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_ret_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
#if defined(CONFIG_MACH_JUNO_SKT) || defined(CONFIG_MACH_JUNO_KT)
static ssize_t set_tsp_for_inputmethod_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t set_tsp_for_inputmethod_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
#endif

static DEVICE_ATTR(firmware, 0660, firmware_show, firmware_store);
static DEVICE_ATTR(firmware_ret, 0660, firmware_ret_show, firmware_ret_store);
#if defined(CONFIG_MACH_JUNO_SKT) || defined(CONFIG_MACH_JUNO_KT)
static DEVICE_ATTR(set_tsp_for_inputmethod, 0660 , set_tsp_for_inputmethod_show, set_tsp_for_inputmethod_store);
#endif

static const int touchkey_keycodes[] = {
			KEY_MENU,
			KEY_BACK,
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif //CONFIG_HAS_EARLYSUSPEND

static struct muti_touch_info g_Mtouch_info[MELFAS_MAX_TOUCH];

int melfas_power_control(BOOLEAN OnOff)
{
    struct vreg *vreg_touch;
    int ret = 0;

    vreg_touch = vreg_get(NULL, "ldo6");

    if(OnOff)
    {
        ret = vreg_set_level(vreg_touch, OUT2800mV);
        
        if (ret) {
            printk(KERN_ERR "%s: vreg set level failed (%d)\n",
                    __func__, ret);
            return -EIO;
        }

        ret = vreg_enable(vreg_touch);
        
        if (ret) {
            printk(KERN_ERR "%s: vreg enable failed (%d)\n",
                    __func__, ret);
            return -EIO;
        }
    }
    else
    {
        ret = vreg_disable(vreg_touch);

        if (ret) {
            printk(KERN_ERR "%s: vreg enable failed (%d)\n",
                    __func__, ret);
            return -EIO;
        }   
    }

	msleep(100);

    return ret;
}

static int melfas_init_panel(struct melfas_ts_data *ts)
{
	uint8_t buf = 0x00;
    int ret = 0;

	ret = i2c_master_send(ts->client, &buf, 1);

	if(ret <0)
	{
		printk(KERN_ERR "melfas_ts_probe: i2c_master_send() failed\n [%d]", ret);	
		return 0;
	}

	return true;
}

#define ABS(a,b) (a>b?(a-b):(b-a))
static void melfas_ts_work_func(struct work_struct *work)
{
	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);
	int ret = 0, i;
	uint8_t buf[TS_READ_REGS_LEN];
	int touchType=0, touchState =0, touchID=0, posX=0, posY=0, width = 0, strength=0, keyID = 0, reportID = 0;

#if 0//DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_work_func\n");

	if(ts ==NULL)
			printk(KERN_ERR "melfas_ts_work_func : TS NULL\n");
#endif //DEBUG_PRINT

#if 0
	/** 
	SMBus Block Read:
		S Addr Wr [A] Comm [A]
				S Addr Rd [A] [Data] A [Data] A ... A [Data] NA P
	*/
	ret = i2c_smbus_read_i2c_block_data(ts->client, TS_READ_START_ADDR, TS_READ_REGS_LEN, buf);
	if (ret < 0)
	{
		printk(KERN_ERR "melfas_ts_work_func: i2c_smbus_read_i2c_block_data(), failed\n");
	}
#else
	/**
	Simple send transaction:
		S Addr Wr [A]  Data [A] Data [A] ... [A] Data [A] P
	Simple recv transaction:
		S Addr Rd [A]  [Data] A [Data] A ... A [Data] NA P
	*/

	buf[0] = TS_READ_START_ADDR;

	for(i=0; i<I2C_RETRY_CNT; i++)
	{
		ret = i2c_master_send(ts->client, buf, 1);
        msleep(10);
#if 0//DEBUG_PRINT
		printk(KERN_ERR "melfas_ts_work_func : i2c_master_send [%d]\n", ret);
#endif //DEBUG_PRINT
		if(ret >=0)
		{
			ret = i2c_master_recv(ts->client, buf, TS_READ_REGS_LEN);
#if 0//DEBUG_PRINT			
			printk(KERN_ERR "melfas_ts_work_func : i2c_master_recv [%d]\n", ret);			
#endif //DEBUG_PRINT

			if(ret >=0)
			{
				break; // i2c success
			}
		}
	}
#endif

	if (ret < 0)
	{
		printk(KERN_ERR "melfas_ts_work_func: i2c failed\n");
		return ;	
	}
	else 
	{
#if MTSI_VERSION > 0x06 
		touchType  = (buf[0]>>5)&0x03;
		touchState = (buf[0]>>4)&0x01;
#else // MTSI_VERSION
		touchType  = (buf[0]>>6)&0x03;
		touchState = (buf[0]>>4)&0x03;
#endif //MTSI_VERSION
		reportID = (buf[0]&0x0F);
		posX = ((buf[1]& 0x0F) << (8)) +  buf[2];
		posY = (((buf[1]& 0xF0) >> 4) << (8)) +  buf[3];
#if MTSI_VERSION > 0x06 
		if(touchType == 2 ) //touch key
		{
			keyID = reportID;
		}
		else
		{
			keyID = 0;
		}
		width = buf[4];
		strength = buf[5];
#else // MTSI_VERSION
		strength = buf[4]; 
#endif //MTSI_VERSION

		touchID = reportID-1;

		if(touchID > MELFAS_MAX_TOUCH-1)
		{
#if DEBUG_PRINT
		printk(KERN_ERR "melfas_ts_work_func: Touch ID: %d\n",  touchID);
#endif //DEBUG_PRINT
			return ;
		}
		
		if(touchType == TOUCH_SCREEN)
		{
			g_Mtouch_info[touchID].posX= posX;
			g_Mtouch_info[touchID].posY= posY;
			g_Mtouch_info[touchID].width= width;			

			if(touchState)
				g_Mtouch_info[touchID].strength= strength;
			else
				g_Mtouch_info[touchID].strength = 0;

			for(i=0; i<MELFAS_MAX_TOUCH; i++)
			{
/////////////////////////////// for 3x4 input method //////////////////////////////////////////////////////////////////////
				if(g_Mtouch_info[i].reportID-1 >=1) // press interrupt
				{
					if(i==0 && g_Mtouch_info[1].strength != 1)
					{
						if((g_Mtouch_info[2].reportID-1 != g_Mtouch_info[0].reportID-1)&&(g_Mtouch_info[2].reportID-1 != 0))// no release with finger id change
						{
				//			if(g_Mtouch_info[1].reportID ==0)
							{
								input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[2].posX);	
								input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[2].posY);
								input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
								input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[2].width);
								input_mt_sync(ts->input_dev);
								input_sync(ts->input_dev);

							//	printk("[TSP] [%d] 0 (%d,	%d,	%x)\n", i, g_Mtouch_info[2].posX, g_Mtouch_info[2].posY, g_Mtouch_info[2].width);
								g_Mtouch_info[1].strength = -1;
							}
						}
						else if(g_Mtouch_info[2].reportID-1 != 0) // check x or y jump with same finger id
						{
#if defined(CONFIG_MACH_COOPER_BASE_KOR)					
							if(ABS(g_Mtouch_info[2].posX,g_Mtouch_info[0].posX)> jump_limit_X)
#else
							if(ABS(g_Mtouch_info[2].posX,g_Mtouch_info[0].posX)>180)
#endif
							{
								input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[2].posX);	
								input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[2].posY);
								input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
								input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[2].width);
								input_mt_sync(ts->input_dev);
								input_sync(ts->input_dev);

							//	printk("[TSP] [%d] 0 (%d,	%d,	%x)\n", i, g_Mtouch_info[2].posX, g_Mtouch_info[2].posY, g_Mtouch_info[2].width);
								g_Mtouch_info[1].strength = -1;	
							}
#if defined(CONFIG_MACH_COOPER_BASE_KOR)					
							else if(ABS(g_Mtouch_info[2].posY,g_Mtouch_info[0].posY)>jump_limit_Y)
#else                        
							else if(ABS(g_Mtouch_info[2].posY,g_Mtouch_info[0].posY)>180)
#endif                        
							{
								input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[2].posX);	
								input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[2].posY);
								input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
								input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[2].width);
								input_mt_sync(ts->input_dev);
								input_sync(ts->input_dev);

							//	printk("[TSP] [%d] 0 (%d,	%d,	%x)\n", i, g_Mtouch_info[2].posX, g_Mtouch_info[2].posY, g_Mtouch_info[2].width);
								g_Mtouch_info[1].strength = -1;	
							}
							else // no jump
							{
								if(g_Mtouch_info[i].strength != -2) // force release
									g_Mtouch_info[i].strength = 1;
								else
									g_Mtouch_info[i].strength = -2;
							}
						}
						else // single touch with normal condition
						{
							if(g_Mtouch_info[i].strength != -2) // force release
								g_Mtouch_info[i].strength = 1;
							else
								g_Mtouch_info[i].strength = -2;
						}
					}
					else
					{
						if(g_Mtouch_info[i].strength != -2) // force release
							g_Mtouch_info[i].strength = 1;
						else
							g_Mtouch_info[i].strength = -2;
					}
				}
				else if(g_Mtouch_info[i].reportID-1 ==0) // release interrupt (only first finger)
				{
					if(g_Mtouch_info[i].strength == 1) // prev strength is press
						g_Mtouch_info[i].strength = 0;
					else if(g_Mtouch_info[i].strength == 0 || g_Mtouch_info[i].strength == -2) // release already or force release
						g_Mtouch_info[i].strength = -1;				
				}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
				if(g_Mtouch_info[i].strength== -1)
					continue;

				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength );
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);      				
				input_mt_sync(ts->input_dev);          
#if DEBUG_PRINT
		printk(KERN_ERR "melfas_ts_work_func: Touch ID: %d, State : %d, x: %d, y: %d, z: %d w: %d\n", 
			i, touchState, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, g_Mtouch_info[i].width);
#endif //DEBUG_PRINT
				if(g_Mtouch_info[i].strength == 0)
					g_Mtouch_info[i].strength = -1;
			}
		}
		else if(touchType == TOUCH_KEY)
		{
			if (keyID == 0x1)
				input_report_key(ts->input_dev, KEY_MENU, touchState ? PRESS_KEY : RELEASE_KEY);		
			if (keyID == 0x2)
				input_report_key(ts->input_dev, KEY_BACK, touchState ? PRESS_KEY : RELEASE_KEY);							
/*				
			if (keyID == 0x2)
				input_report_key(ts->input_dev, KEY_HOME, touchState ? PRESS_KEY : RELEASE_KEY);
			if (keyID == 0x3)
				input_report_key(ts->input_dev, KEY_SEARCH, touchState ? PRESS_KEY : RELEASE_KEY);
			if (keyID == 0x4)
				input_report_key(ts->input_dev, KEY_BACK, touchState ? PRESS_KEY : RELEASE_KEY);			
*/				
#if DEBUG_PRINT
			printk(KERN_ERR "melfas_ts_work_func: keyID : %d, touchState: %d\n", keyID, touchState);
#endif //DEBUG_PRINT
		}

		input_sync(ts->input_dev);
	}
			
	enable_irq(ts->client->irq);
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
	struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;
#if 0//DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_irq_handler\n");
#endif //DEBUG_PRINT

	disable_irq_nosync(ts->client->irq);
	schedule_work(&ts->work);

	return IRQ_HANDLED;
}

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts;
	int ret = 0, i; 
#if SET_DOWNLOAD_BY_GPIO
	uint8_t buf[2] = {0,};
#else
	uint8_t buf;

#endif //SET_DOWNLOAD_BY_GPIO

#if DEBUG_PRINT
	printk(KERN_ERR "kim ms : melfas_ts_probe\n");
#endif //DEBUG_PRINT

    gpio_direction_output(TSP_SCL, 1);
    gpio_direction_output(TSP_SDA, 1);    
	gpio_set_value(TSP_INT , 1);     

    melfas_power_control(1);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kmalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "melfas_ts_probe: failed to create a state of melfas-ts\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	INIT_WORK(&ts->work, melfas_ts_work_func);

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts_global = ts;

/*	for(i=0; i<I2C_RETRY_CNT; i++)
	{
		ret = i2c_master_send(ts->client, &buf, 1);

		if(ret >=0)
			break;
		else
			printk(KERN_ERR "melfas_ts_probe: i2c_master_send() failed[%d]\n", ret);
	}*/

//	ret = i2c_master_send(ts->client, &buf, 1);

#if 0  //DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_probe: i2c_master_send() [%d], Add[%d]\n", ret, ts->client->addr);
#endif //DEBUG_PRINT

#if SET_DOWNLOAD_BY_GPIO
    buf[0] = TS_READ_MODULE_VERSION_ADDR;
//    buf[0] = TS_READ_FIRMWARE_VERSION_ADDR;

	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		ret = i2c_master_send(ts->client, buf, 1);
#if DEBUG_PRINT
		printk(KERN_ERR "melfas_ts_work_func : i2c_master_send [%d]\n", ret);
#endif //DEBUG_PRINT
		if (ret >= 0)
		{
			ret = i2c_master_recv(ts->client, buf, 2);
//			ret = i2c_master_recv(ts->client, buf, 1);            
#if DEBUG_PRINT
			printk(KERN_ERR "melfas_ts_work_func : i2c_master_recv [%d]\n", ret);			
#endif //DEBUG_PRINT
			if (ret >= 0)
			{
				break; // i2c success
			}
		}
		else
		{
#if DEBUG_PRINT
			printk(KERN_ERR "melfas_ts_work_func: i2c failed\n");		
#endif //DEBUG_PRINT
//			return;
		}
	}

    printk("[TSP] %s:%d, ver HW=%x, ver SW=%x\n", __func__,__LINE__, buf[0], buf[1] );
//    printk("[TSP] %s:%d, ver SW=%x\n", __func__,__LINE__,  buf[0] );    

	// same module rev. and later firmware

	if (((buf[0] == /*HW_VERSION*/ 0x00 || buf[0] == 0x10) && buf[1] < FW_VERSION) || (buf[1] == 0x00) )
	{
		for(i=0; i<DOWNLOAD_RETRY_CNT;i++)
		{  
			ret = mcsdl_download_binary_data();

			// Check download result 
			if (!ret)
			{
#if DEBUG_PRINT						
				printk(KERN_ERR "SET Download Fail - error code [%d]\n", ret);			
#endif
			}
			else
            {         
#if DEBUG_PRINT			            
    			printk(KERN_ERR "SET Download Succeed.\n");
#endif                
                gpio_direction_output(TSP_SCL, 0);
                gpio_direction_output(TSP_SDA, 0);    
                gpio_set_value(TSP_INT , 0);     
                melfas_power_control(0);
               
                gpio_direction_output(TSP_SCL, 1);
                gpio_direction_output(TSP_SDA, 1);    
                gpio_set_value(TSP_INT , 1);     
                melfas_power_control(1);                

				break;
            }
		}
		
	}	
#endif //SET_DOWNLOAD_BY_GPIO
	
	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		printk(KERN_ERR "melfas_ts_probe: Not enough memory\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	} 

	ts->input_dev->name = "melfas-ts" ;

	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
	

	ts->input_dev->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
	ts->input_dev->keybit[BIT_WORD(KEY_HOME)] |= BIT_MASK(KEY_HOME);
	ts->input_dev->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);		
	ts->input_dev->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);			

//	__set_bit(BTN_TOUCH, ts->input_dev->keybit);
//	__set_bit(EV_ABS,  ts->input_dev->evbit);
//	ts->input_dev->evbit[0] =  BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);	

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TS_MAX_X_COORD, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TS_MAX_Y_COORD, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, TS_MAX_Z_TOUCH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MELFAS_MAX_TOUCH-1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, TS_MAX_W_TOUCH, 0, 0);

//	__set_bit(EV_SYN, ts->input_dev->evbit); 
//	__set_bit(EV_KEY, ts->input_dev->evbit);	
	
	ret = input_register_device(ts->input_dev);	
	if (ret) {
		printk(KERN_ERR "melfas_ts_probe: Failed to register device\n");
		ret = -ENOMEM;
		goto err_input_register_device_failed;
	}

	if (ts->client->irq) {
#if DEBUG_PRINT		
		printk(KERN_ERR "melfas_ts_probe: trying to request irq: %s-%d\n", ts->client->name, ts->client->irq);
#endif //DEBUG_PRINT
		ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, ts->client->name, ts);
		if (ret > 0) {
			printk(KERN_ERR "melfas_ts_probe: Can't allocate irq %d, ret %d\n", ts->client->irq, ret);
			ret = -EBUSY;
			goto err_request_irq;			
		}
	}

//	schedule_work(&ts->work);

	for (i = 0; i < MELFAS_MAX_TOUCH ; i++)  /* _SUPPORT_MULTITOUCH_ */
		g_Mtouch_info[i].strength = -1;	

#if DEBUG_PRINT	
	printk(KERN_ERR "melfas_ts_probe: succeed to register input device\n");
#endif //DEBUG_PRINT

#if CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif //CONFIG_HAS_EARLYSUSPEND

/////////////////////////////// for 3x4 input method //////////////////////////////////////////////////////////////////////
	/* sys fs */
	touch_class = class_create(THIS_MODULE, "touch");
	if (IS_ERR(touch_class))
		pr_err("Failed to create class(touch)!\n");

	firmware_dev = device_create(touch_class, NULL, 0, NULL, "firmware");
	if (IS_ERR(firmware_dev))
		pr_err("Failed to create device(firmware)!\n");

	if (device_create_file(firmware_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_firmware_ret) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ret.attr.name);

#if defined(CONFIG_MACH_JUNO_SKT) || defined(CONFIG_MACH_JUNO_KT)
    set_tsp_dev = device_create(touch_class, NULL, 0, NULL, "set_tsp");    

	if (IS_ERR(set_tsp_dev))
		pr_err("Failed to create device(set_tsp_dev)!\n");

	if (device_create_file(set_tsp_dev, &dev_attr_set_tsp_for_inputmethod) < 0)    
		pr_err("Failed to create device file(%s)!\n", dev_attr_set_tsp_for_inputmethod.attr.name);
#endif    

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if DEBUG_PRINT
	printk(KERN_INFO "melfas_ts_probe: Start touchscreen. name: %s, irq: %d\n", ts->client->name, ts->client->irq);
#endif //DEBUG_PRINT
	return 0;

err_request_irq:
	printk(KERN_ERR "melfas-ts: err_request_irq failed\n");
	free_irq(client->irq, ts);
err_input_register_device_failed:
	printk(KERN_ERR "melfas-ts: err_input_register_device failed\n");
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	printk(KERN_ERR "melfas-ts: err_input_dev_alloc failed\n");
err_alloc_data_failed:
	printk(KERN_ERR "melfas-ts: err_alloc_data failed_\n");	
err_detect_failed:
	printk(KERN_ERR "melfas-ts: err_detect failed\n");
	kfree(ts);
err_check_functionality_failed:
	printk(KERN_ERR "melfas-ts: err_check_functionality failed_\n");

	return ret;
}

/////////////////////////////// for 3x4 input method //////////////////////////////////////////////////////////////////////
int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size)
{
	int ret=-1;
	struct i2c_msg rmsg;
	uint8_t start_reg;

	rmsg.addr = ts_global->client->addr;
	rmsg.flags = 0;//I2C_M_WR;
	rmsg.len = 1;
	rmsg.buf = &start_reg;
	start_reg = reg;
	ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);

	if(ret>=0) {
		rmsg.flags = I2C_M_RD;
		rmsg.len = buf_size;
		rmsg.buf = rbuf;
		ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1 );
	}

	if( ret < 0 )
		{
		printk("[TSP] Error code : %d\n", __LINE__ );
//		printk("[TSP] reset ret=%d\n", tsp_reset( ) );
	}

	return ret;
}
int tsp_i2c_write(u8 reg, unsigned char *rbuf, int buf_size)
{
	int ret=-1;
	struct i2c_msg rmsg;
	unsigned char data[2];

	rmsg.addr = ts_global->client->addr;
	rmsg.flags = 0;
	rmsg.len = 2;
	rmsg.buf = data;
	data[0] = reg;
	data[1] = rbuf[0];
	ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);

	return ret;
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0, i; 
	uint8_t i2c_addr = TS_READ_MODULE_VERSION_ADDR;
	uint8_t buf_tmp[2] = {0};
    uint8_t hw_ver = 0x01;

#if DEBUG_PRINT
	printk("[TSP] %s\n",__func__);
#endif

	buf_tmp[0] = TS_READ_MODULE_VERSION_ADDR;

	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		ret = i2c_master_send(ts_global->client, buf_tmp, 1);
#if DEBUG_PRINT
		printk(KERN_ERR "melfas_ts_work_func : i2c_master_send [%d]\n", ret);
#endif //DEBUG_PRINT
		if (ret >= 0)
		{
			ret = i2c_master_recv(ts_global->client, buf_tmp, 2);
#if DEBUG_PRINT
			printk(KERN_ERR "melfas_ts_work_func : i2c_master_recv [%d]\n", ret);
#endif //DEBUG_PRINT
			if (ret >= 0)
			{
				break; // i2c success
			}
		}
		else
		{
#if DEBUG_PRINT
			printk(KERN_ERR "melfas_ts_work_func: i2c failed\n");       
#endif //DEBUG_PRINT
			return;
		}
	}

	sprintf(buf, "%02X%02X%02X\n",hw_ver, buf_tmp[0], buf_tmp[1]  );

	return sprintf(buf, "%s", buf );
}

/* firmware - update */
static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *after;
	int ret = 0, i; 

	unsigned long value = simple_strtoul(buf, &after, 10);
#if DEBUG_PRINT
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
#endif
	firmware_ret_val = -1;

	if ( value == 1 )
	{
#if DEBUG_PRINT
		printk("[TSP] Firmware update start!!\n" );
#endif

		for(i=0; i<DOWNLOAD_RETRY_CNT; i++)
		{
			ret = mcsdl_download_binary_data();

			// Check download result
			if (!ret)
			{
#if DEBUG_PRINT
				printk(KERN_ERR "SET Download Fail - error code [%d]\n", ret);
#endif
			}
			else
			{
#if DEBUG_PRINT
				printk(KERN_ERR "SET Download Succeed.\n");
#endif
				gpio_direction_output(TSP_SCL, 0);
				gpio_direction_output(TSP_SDA, 0);
				gpio_set_value(TSP_INT , 0);
				melfas_power_control(0);

				gpio_direction_output(TSP_SCL, 1);
				gpio_direction_output(TSP_SDA, 1);
				gpio_set_value(TSP_INT , 1);
				melfas_power_control(1);

				break;
			}
		}

		return size;
	}

	return size;
}

static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
#if DEBUG_PRINT
	printk("[TSP] %s!\n", __func__);
#endif

	return sprintf(buf, "%d", firmware_ret_val );
}

static ssize_t firmware_ret_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
#if DEBUG_PRINT
	printk("[TSP] %s, operate nothing!\n", __func__);
#endif

	return size;
}

#if defined(CONFIG_MACH_JUNO_SKT) || defined(CONFIG_MACH_JUNO_KT)
static ssize_t set_tsp_for_inputmethod_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	printk("[TSP] %s.. jump_limit_X = %d! jump_limit_Y = %d!\n", __func__, jump_limit_X, jump_limit_Y);

    if(jump_limit_X == 180 && jump_limit_Y == 180)
        *buf= '0';
    else
        *buf= '1';        

	return 0;
}

static ssize_t set_tsp_for_inputmethod_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{	
    if(*buf == '1' && jump_limit_X == 180 && jump_limit_Y == 180 )
	{
        jump_limit_X = 70;
		jump_limit_Y = 47;
		
	}
    else if (*buf == '0' && jump_limit_X == 70 && jump_limit_Y == 47)
	{
        jump_limit_X = jump_limit_Y = 180;
	}

	printk("[TSP] %s.. jump_limit_X = %d! jump_limit_Y = %d!\n", __func__, jump_limit_X, jump_limit_Y);    
    
	return 1;
}
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	printk("[TSP] %s+\n", __func__ );

	disable_irq(client->irq);
    
	ret = cancel_work_sync(&ts->work);
	if (ret) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);

	ret = i2c_smbus_write_byte_data(client, 0x01, 0x00); /* deep sleep */
	
	if (ret < 0)
		printk(KERN_ERR "melfas_ts_suspend: i2c_smbus_write_byte_data failed\n");

    melfas_power_control(0);

    gpio_direction_output(TSP_SCL, 0);
    gpio_direction_output(TSP_SDA, 0);    
    gpio_set_value(TSP_INT , 0);     
    
	printk("[TSP] %s-\n", __func__ );
	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	printk("[TSP] %s+\n", __func__ );    

    gpio_direction_output(TSP_SCL, 1);
    gpio_direction_output(TSP_SDA, 1);    
	gpio_set_value(TSP_INT , 1);     

    melfas_power_control(1);

	melfas_init_panel(ts);
	enable_irq(client->irq); // scl wave

	printk("[TSP] %s-\n", __func__ );            

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id melfas_ts_id[] = {
	{ MELFAS_TS_NAME, 0 },
	{ }
};

static struct i2c_driver melfas_ts_driver = {
	.driver		= {
		.name	= MELFAS_TS_NAME,
	},
	.id_table		= melfas_ts_id,
	.probe		= melfas_ts_probe,
	.remove		= __devexit_p (melfas_ts_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend		= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
};

static int __devinit melfas_ts_init(void)
{
	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
}

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
