/* drviers/media/tdmb/tdmb_drv.c
 *
 * TDMB_DRV for Linux
 *
 * klaatu, Copyright (c) 2009 Samsung Eclectronics
 * 			http://www.samsung.com/
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <mach/gpio.h>
//#include <mach/mux.h>
#include <asm/io.h>

#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/time.h>
#include <linux/timer.h>
//#include <linux/i2c/twl4030.h>

#include "tdmb.h"

#ifdef CONFIG_TDMB_T3700
#include "INC_INCLUDES.h"
#endif
#ifdef CONFIG_TDMB_FC8050
#include "DMBDrv_wrap_FC8050.h"
#endif

// --------------------------------
#define TDMB_DEBUG

#ifdef TDMB_DEBUG
#define DPRINTK(x...) printk("[TDMB] " x)
#else /* TDMB_DEBUG */
#define DPRINTK(x...)  /* null */
#endif /* TDMB_DEBUG */

static DEFINE_MUTEX(tdmb_int_mutex);

tdmb_type g_TDMBGlobal; 

// --------------------------------
// definition


#ifdef CONFIG_TDMB_T3700
#define TDMB_ID_0x80        0x80
#endif

//// GPIO //////////////
#if defined(CONFIG_MACH_JUNO_SKT) || defined(CONFIG_MACH_JUNO_KT)
#define TDMB_EN     22
#define TDMB_RST    97
#define TDMB_INT    38
#else
#define TDMB_EN     21
#define TDMB_RST    97
#define TDMB_INT    108
#endif

#define GPIO_TDMB_EN	 TDMB_EN
#define GPIO_TDMB_RST	TDMB_RST
#define GPIO_TDMB_INT	TDMB_INT
#define IRQ_TDMB_INT	MSM_GPIO_TO_INT(TDMB_INT)
#define GPIO_LEVEL_LOW   0 
#define GPIO_LEVEL_HIGH  1
////////////////////////


////////////////////////
// TODO
extern unsigned int *ts_head;
extern unsigned int *ts_tail;
extern char *ts_buffer;
extern unsigned int ts_size;

int TDMB_AddDataToRing(unsigned char* pData, unsigned long dwDataSize);

extern unsigned char _tdmb_make_result(unsigned char byCmd, unsigned short byDataLength, unsigned char* pbyData);

#ifdef CONFIG_TDMB_T3700
INC_UINT8  g_acStreamBuff[INC_INTERRUPT_SIZE+188];
#define INTERRUPT_SIZE INC_INTERRUPT_SIZE
extern INC_CHANNEL_INFO    g_currChInfo;
#endif
#ifdef CONFIG_TDMB_FC8050
#define INTERRUPT_SIZE 188*20
#define MSCBuff_Size 1024  // check each chipset 
#endif

#ifdef TDMB_TS_DUMP_TO_FILE
#define TDMB_DUMP_FILE_TS_NAME_1  "/mnt/sdcard/dmb/tv_dump1.ts"
#define TDMB_DUMP_FILE_TS_NAME_2  "/mnt/sdcard/dmb/tv_dump2.ts"
#define TDMB_DUMP_FILE_MP2_NAME_1  "/mnt/sdcard/dmb/radio_dump1.mp2"
#define TDMB_DUMP_FILE_MP2_NAME_2  "/mnt/sdcard/dmb/radio_dump2.mp2"
static int TDMB_TS_DUMP_FILE_NUM = 1;
static int TDMB_MP2_DUMP_FILE_NUM = 1;
void TDMB_write_to_file(void *_data, int size, int type);
#endif

#define TS_PACKET_SIZE 188

unsigned char MSCBuff[MSCBuff_Size];
unsigned char TSBuff[INTERRUPT_SIZE * 2];
extern int bfirst;
extern int TSBuffpos;
extern int MSCBuffpos;
extern int mp2len;
static const int bitRateTable[2][16] = { {0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, 0},  /* MPEG1 for id=1*/ 
                                                           {0,  8,  16, 24, 32, 40, 48,  56,   64,   80,  96,  112, 128, 144, 160, 0} };/* MPEG2 for id=0 */
static int __AddDataTS(unsigned char* pData, unsigned long dwDataSize)
{
    int j = 0;
    int maxi = 0;
    if(bfirst)
    {
        printk("!!!!!!!!!!!!! first sync dwDataSize = %ld !!!!!!!!!!!!\n", dwDataSize);

        for(j=0;j<dwDataSize;j++)
        {
            if(pData[j]==0x47)
            {                
                printk("!!!!!!!!!!!!! first sync j = %d !!!!!!!!!!!!\n",j);
                _tdmb_make_result(DMB_TS_PACKET_RESYNC, sizeof(int), &j);
                maxi = (dwDataSize - j) / TS_PACKET_SIZE;
                TSBuffpos = (dwDataSize - j) % TS_PACKET_SIZE;
                TDMB_AddDataToRing(&pData[j], maxi * TS_PACKET_SIZE);
                if(TSBuffpos > 0)
                    memcpy(TSBuff, &pData[j + maxi * TS_PACKET_SIZE], TSBuffpos);
                bfirst = 0;
                return 0;
            }
        }
    }
    else
    {
        maxi = (dwDataSize) / TS_PACKET_SIZE;

        if(TSBuffpos > 0)
        {
            if(pData[TS_PACKET_SIZE - TSBuffpos] != 0x47)
            {
                printk("!!!!!!!!!!!!! error 0x%x,0x%x!!!!!!!!!!!!\n",pData[TS_PACKET_SIZE - TSBuffpos], pData[TS_PACKET_SIZE - TSBuffpos + 1]);                    
                memset(TSBuff, 0, INTERRUPT_SIZE * 2);
                TSBuffpos =0;
                bfirst =1;
                return -1;
            }

            memcpy(&TSBuff[TSBuffpos], pData, TS_PACKET_SIZE-TSBuffpos);
            TDMB_AddDataToRing(TSBuff, TS_PACKET_SIZE);
            TDMB_AddDataToRing(&pData[TS_PACKET_SIZE - TSBuffpos], dwDataSize - TS_PACKET_SIZE);
            memcpy(TSBuff, &pData[dwDataSize-TSBuffpos], TSBuffpos);
        }
        else
        {
            if(pData[0] != 0x47)
            {
                printk("!!!!!!!!!!!!! error 0x%x,0x%x!!!!!!!!!!!!\n",pData[0], pData[1]);                    
                memset(TSBuff, 0, INTERRUPT_SIZE * 2);
                TSBuffpos =0;
                bfirst =1;
                return -1;
            }

            TDMB_AddDataToRing(pData, dwDataSize);
        }
    }
    return 0;
}                       
int MP2_GetPacketLength(unsigned char *pkt)
{
    int id, layer_index, bitrate_index, fs_index, samplerate,protection;
    int bitrate, length;
                                              
    id = (pkt[1]>>3) &0x01; /* 1: ISO/IEC 11172-3, 0:ISO/IEC 13818-3 */
    layer_index = (pkt[1]>>1)&0x03; /* 2 */
    protection = pkt[1]&0x1;
    if (protection != 0)
    {
        //QTV_F("protection_bit is *NOT* 0");
    }
    bitrate_index = (pkt[2]>>4);
    fs_index = (pkt[2]>>2)&0x3; /* 1 */

    if(pkt[0]==0xff && (pkt[1]>>4)==0xf) /* sync word check */
    {
        if ( (bitrate_index > 0 && bitrate_index < 15) && (layer_index==2) && (fs_index ==1) )
        {
            if (id ==1 && layer_index ==2) /* Fs==48 KHz*/
            {
                bitrate=1000*bitRateTable[0][bitrate_index];
                samplerate=48000;
            }
            else if (id==0 && layer_index ==2) /* Fs=24 KHz */
            {
                bitrate=1000*bitRateTable[1][bitrate_index];
                samplerate=24000;
            }
            else
                return -1;
        }
        else
            return -1;
    }
    else
        return -1;

    if ( (pkt[2]&0x02)!=0) /* padding bit */
    {
        return -1;            
    }
    
    length = (144*bitrate)/(samplerate);
    
    return length;
}

static int __AddDataMSC(unsigned char* pData, unsigned long dwDataSize, int SubChID)
{
    int j,readpos =0;
    unsigned char pOutAddr[188];
    static int first=1;
    int remainbyte = 0;
    if(bfirst)
    {
        for(j=0;j<dwDataSize-4;j++)
        {
            if(pData[j]==0xFF && (pData[j+1]>>4==0xF))
            {                
                mp2len = MP2_GetPacketLength(&pData[j]);
                printk("!!!!!!!!!!!!! first sync mp2len= %d !!!!!!!!!!!!\n",mp2len);
                if(mp2len <=0 || mp2len >MSCBuff_Size )
                    return -1;
                memcpy(MSCBuff, &pData[j], dwDataSize-j);
                MSCBuffpos = dwDataSize-j;
                bfirst = 0;
                first =1;
                return 0;
            }
        }
    }
    else
    {
        if(mp2len <=0 || mp2len >MSCBuff_Size )
        {
            MSCBuffpos =0;
            bfirst =1;
            return -1;
        }   

        remainbyte = dwDataSize;
        if(mp2len-MSCBuffpos>=dwDataSize)
        {
            memcpy(MSCBuff+MSCBuffpos, pData,  dwDataSize);
            MSCBuffpos += dwDataSize;
            remainbyte = 0;
        }
        else if(mp2len-MSCBuffpos>0)
        {
            memcpy(MSCBuff+MSCBuffpos, pData,  mp2len-MSCBuffpos);
            remainbyte = dwDataSize - (mp2len -MSCBuffpos);
            MSCBuffpos = mp2len;
        }

        if(MSCBuffpos==mp2len)
        {
            while(MSCBuffpos>readpos)
            {
                //memset(pOutAddr, 0, 188);
                if(first)
                {
                    pOutAddr[0]=0xDF;
                    pOutAddr[1]=0xDF;
                    pOutAddr[2]= (SubChID<<2);
                    pOutAddr[2] |=(((MSCBuffpos>>3)>>8)&0x03);
                    pOutAddr[3] = (MSCBuffpos>>3)&0xFF;
                    if(!(MSCBuff[0]==0xFF && (MSCBuff[1]>>4==0xF)))
                    {
                        printk("!!!!!!!!!!!!! error 0x%x,0x%x!!!!!!!!!!!!\n",MSCBuff[0], MSCBuff[1]);                    
                        memset(MSCBuff, 0, MSCBuff_Size);
                        MSCBuffpos =0;
                        bfirst =1;
                        return -1;
                    }
                    memcpy(pOutAddr+4, MSCBuff, 184);
                    readpos = 184;
                    first =0;
                }
                else
                {
                    pOutAddr[0]=0xDF;
                    pOutAddr[1]=0xD0;
                    if(MSCBuffpos-readpos>=184)
                    {
                        memcpy(pOutAddr+4, MSCBuff+readpos, 184);
                        readpos +=184;
                    }
                    else
                    {
                        memcpy(pOutAddr+4, MSCBuff+readpos, MSCBuffpos-readpos);
                        readpos +=(MSCBuffpos-readpos);
                    }

                }
                TDMB_AddDataToRing(pOutAddr, 188);
            }
            first =1;
            MSCBuffpos =0;
            if(remainbyte>0)
            {
                memcpy(MSCBuff, pData+dwDataSize-remainbyte, remainbyte);
                MSCBuffpos = remainbyte;                
            }
        }
        else if(MSCBuffpos > mp2len)
        {
            printk("!!!!!!!!!!!!!Error MSCBuffpos=%d, mp2len =%d!!!!!!!!!!!!\n",MSCBuffpos, mp2len);                    
            memset(MSCBuff, 0, MSCBuff_Size);
            MSCBuffpos =0;
            bfirst =1;
            return -1;
        }
    }
    return 0;
}
#ifdef CONFIG_TDMB_T3700
void tdmb_data_restore(void)
    {
    INC_UINT16 ulRemainLength = INC_INTERRUPT_SIZE;
    INC_UINT16 unIndex = 0;
    INC_UINT16 unSPISize= 0xFFF;
    
    memset(g_acStreamBuff, 0, sizeof(g_acStreamBuff));

    while(ulRemainLength)
    {
        if(ulRemainLength >= unSPISize){

            INC_CMD_READ_BURST(TDMB_I2C_ID80, APB_STREAM_BASE, &g_acStreamBuff[unIndex*unSPISize], unSPISize);
            unIndex++;
            ulRemainLength -= unSPISize;
            
        }
        else
        {
            INC_CMD_READ_BURST(TDMB_I2C_ID80, APB_STREAM_BASE, &g_acStreamBuff[unIndex*unSPISize], ulRemainLength);
            ulRemainLength = 0;
        }
    }

    if ( g_currChInfo.ucServiceType == 0x18 )
    {
     //TDMB_AddDataToRing(g_acStreamBuff, INC_INTERRUPT_SIZE);
        __AddDataTS(g_acStreamBuff, INC_INTERRUPT_SIZE);
#ifdef TDMB_TS_DUMP_TO_FILE
        TDMB_write_to_file((void*)g_acStreamBuff, INC_INTERRUPT_SIZE, 1);
#endif
    }
    else
    {
        INC_UINT16 i;
        INC_UINT16 maxi = INC_INTERRUPT_SIZE/188;
        unsigned char* pData = (unsigned char*)g_acStreamBuff;
        for ( i = 0 ; i < maxi ; i++ )
        {
            __AddDataMSC(pData, 188, g_currChInfo.ucSubChID);
#ifdef TDMB_TS_DUMP_TO_FILE
            TDMB_write_to_file((void*)pData, 188, 0);
#endif
            pData += 188;
        }
    }
}
#elif defined(CONFIG_TDMB_FC8050)
void tdmb_data_restore(unsigned char* pData, unsigned long dwDataSize, unsigned char ucSubChannel, unsigned char ucSvType)
{
    if ( ucSvType == 0x18 )
    {
        __AddDataTS(pData, dwDataSize);
    }
    else
    {
        unsigned short i;
        unsigned short maxi = dwDataSize/188;
        
        DPRINTK("tdmb_data_restore dwDataSize(%d) maxi(%d)\r\n", dwDataSize, maxi);
        for ( i = 0 ; i < maxi ; i++ )
        {
            __AddDataMSC(pData, 188, ucSubChannel);
            pData += 188;
        }
    }

}
#endif
void tdmb_work_function(void)
{
//	DPRINTK("tdmb_work_function \r\n");

#ifdef CONFIG_TDMB_T3700
    tdmb_data_restore();
#elif defined(CONFIG_TDMB_FC8050)
    DMBDrv_ISR();
#endif

    return;

}

static struct workqueue_struct *tdmb_workqueue=0;
DECLARE_WORK(tdmb_work, tdmb_work_function);


int IsTDMBPowerOn(void) 
{ 
    return g_TDMBGlobal.b_isTDMB_Enable; 
} 

static irqreturn_t TDMB_irq_handler(int irq, void *dev_id)
{
	int ret = 0;

//	DPRINTK("TDMB_irq_handler \r\n");

	if(!tdmb_workqueue)
	{
		DPRINTK("tdmb_workqueue doesn't exist!\n");
		tdmb_workqueue=create_singlethread_workqueue("ktdmbd");

		if(!tdmb_workqueue)
		{
			DPRINTK("failed to create_workqueue\n");
			return TDMB_ERROR;
		}
	}

	ret=queue_work(tdmb_workqueue, &tdmb_work);
	if(ret==0)
	{
		DPRINTK("failed in queue_work\n");
	}
/*    
	else
	{
		DPRINTK("suceeded in queue_work\n");
	}
*/
	return IRQ_HANDLED;
}

int TDMB_drv_Init(void)
{
	int dRet = TDMB_ERROR;
   //define tdmb workqueue
	DPRINTK("Work Queue Init ! \r\n");

	tdmb_workqueue = create_singlethread_workqueue("ktdmbd");

	if(!tdmb_workqueue)
	{
		DPRINTK("failed create_singlethread_workqueue\n");
		return TDMB_ERROR;
	}

//	set_irq_type(IRQ_TDMB_INT,IRQ_TYPE_EDGE_FALLING); // TODO
	dRet = request_irq(IRQ_TDMB_INT, TDMB_irq_handler, IRQF_DISABLED | IRQF_TRIGGER_FALLING, "TDMB", NULL);

	if(dRet < 0)
	{
		DPRINTK("request_irq failed !! \r\n");
		return TDMB_ERROR;
	}
	
	return TDMB_SUCCESS;
}

static void __TDMB_drv_PowerOn(void)// power on
{
	DPRINTK("TDMB_drv_PowerOn\r\n");
    int rc_en = 0;
    int rc_rst = 0;
    int rc_int = 0;

#if defined(CONFIG_TDMB_T3700)
  // TDMB Reset
#if 1 

#if 1
    // TDMB Reset
    gpio_request(GPIO_TDMB_EN,"tdmb" );
    gpio_direction_output(GPIO_TDMB_EN, GPIO_LEVEL_HIGH);
    msleep(10);
    DPRINTK("TDMB_drv_PowerOn !! 1\r\n");

    gpio_request(GPIO_TDMB_RST,"tdmb" );
    gpio_direction_output(GPIO_TDMB_RST, GPIO_LEVEL_LOW);
    msleep(2);
    DPRINTK("TDMB_drv_PowerOn !! 3\r\n");

    gpio_set_value(GPIO_TDMB_RST, GPIO_LEVEL_HIGH);    
    msleep(10);
    DPRINTK("TDMB_drv_PowerOn !! 5\r\n");
#else
    gpio_configure(TDMB_EN, GPIOF_DRIVE_OUTPUT);
    gpio_configure(TDMB_RST, GPIOF_DRIVE_OUTPUT);
    gpio_configure(TDMB_INT, GPIOF_INPUT);  

    rc_en = gpio_tlmm_config(GPIO_CFG(GPIO_TDMB_EN, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);
    rc_rst = gpio_tlmm_config(GPIO_CFG(GPIO_TDMB_RST, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);
    rc_int = gpio_tlmm_config(GPIO_CFG(GPIO_TDMB_INT, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);

    gpio_set_value( GPIO_TDMB_EN , GPIO_LEVEL_HIGH );
    msleep(10);

    gpio_set_value( GPIO_TDMB_RST , GPIO_LEVEL_LOW );
    msleep(2);

    gpio_set_value( GPIO_TDMB_RST , GPIO_LEVEL_HIGH );
    msleep(10);
#endif
#else
    gpio_request(GPIO_TDMB_EN,"tdmb" );
    gpio_direction_output(GPIO_TDMB_EN, GPIO_LEVEL_HIGH);
    msleep(10);
    gpio_request(GPIO_TDMB_RST,"tdmb" );
    gpio_direction_output(GPIO_TDMB_RST, GPIO_LEVEL_LOW);
    msleep(2);
    gpio_set_value(GPIO_TDMB_RST, GPIO_LEVEL_HIGH);    
    msleep(10);
#endif
#elif defined(CONFIG_TDMB_FC8050)
#if 1
    gpio_configure(TDMB_EN, GPIOF_DRIVE_OUTPUT);
    gpio_configure(TDMB_RST, GPIOF_DRIVE_OUTPUT);
    gpio_configure(TDMB_INT, GPIOF_INPUT);	


    gpio_tlmm_config(GPIO_CFG(GPIO_TDMB_EN, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);
    gpio_tlmm_config(GPIO_CFG(GPIO_TDMB_RST, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),GPIO_ENABLE);
    gpio_tlmm_config(GPIO_CFG(GPIO_TDMB_INT, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),GPIO_ENABLE);

    gpio_set_value( GPIO_TDMB_EN , GPIO_LEVEL_HIGH );
    msleep(5);

    gpio_set_value( GPIO_TDMB_RST , GPIO_LEVEL_LOW );
    msleep(1);

    gpio_set_value( GPIO_TDMB_RST , GPIO_LEVEL_HIGH );
    msleep(1);

#endif  
#if 0
  gpio_request(GPIO_TDMB_EN,"tdmb" );
  gpio_direction_output(GPIO_TDMB_EN, GPIO_LEVEL_HIGH);
  msleep(5);

	DPRINTK("TDMB_drv_PowerOn 1\r\n");

  gpio_request(GPIO_TDMB_RST,"tdmb" );
  gpio_direction_output(GPIO_TDMB_RST, GPIO_LEVEL_LOW);
  msleep(1);

	DPRINTK("TDMB_drv_PowerOn 2\r\n");
  
  gpio_set_value(GPIO_TDMB_RST, GPIO_LEVEL_HIGH);    
  msleep(1);
  
	DPRINTK("TDMB_drv_PowerOn 3\r\n");
  
#endif
#endif	

}

static void __TDMB_drv_PowerOff(void)
{
	DPRINTK("__TDMB_drv_PowerOff OK\r\n");

#ifdef CONFIG_TDMB_T3700
  gpio_set_value(GPIO_TDMB_RST, GPIO_LEVEL_LOW);    
	msleep(10);
  gpio_set_value(GPIO_TDMB_EN, GPIO_LEVEL_LOW);
	msleep(10);
	
#elif defined(CONFIG_TDMB_FC8050)
	gpio_set_value( GPIO_TDMB_RST , GPIO_LEVEL_LOW );
  msleep(1);

	gpio_set_value( GPIO_TDMB_EN , GPIO_LEVEL_LOW );
  msleep(1);  
#endif

}

void TDMBDrv_PowerInit(void)
{
    __TDMB_drv_PowerOn();
    __TDMB_drv_PowerOff();
}


int TDMB_PowerOn(void)
{
#ifdef CONFIG_TDMB_SPI	
    spi_open();
#endif    

    DPRINTK("call TDMB_PowerOn ! \n");
    __TDMB_drv_PowerOn();// power on by gpio
    TDMB_drv_Init();

#ifdef CONFIG_TDMB_T3700
    if(INC_SUCCESS != INTERFACE_INIT(TDMB_ID_0x80))
    {
        printk("[TDMB] tdmb power on failed\n");
        TDMB_PowerOff();
        return FALSE;
    }
    else
    {
        g_TDMBGlobal.b_isTDMB_Enable = 1; 
        printk("[TDMB] tdmb power on success\n");
        return TRUE;
    }
#elif defined(CONFIG_TDMB_FC8050)
    if(DMBDrv_init() == TDMB_FAIL)
    {
        printk("[TDMB] tdmb power on failed\n");
        TDMB_PowerOff();
        return FALSE;
    }
    else
    {
        g_TDMBGlobal.b_isTDMB_Enable = 1; 
        printk("[TDMB] tdmb power on success\n");
        return TRUE;
    }

#endif

}

void TDMB_PowerOff(void)
{
	DPRINTK("call TDMB_PowerOff ! \n");
	g_TDMBGlobal.b_isTDMB_Enable = 0; 
#ifdef CONFIG_TDMB_T3700
    INC_STOP(TDMB_ID_0x80);
#elif defined(CONFIG_TDMB_FC8050)
    DMBDrv_DeInit();
#endif

// disable ISR
	free_irq(IRQ_TDMB_INT, NULL);

	// flush workqueue
	flush_workqueue(tdmb_workqueue);
	destroy_workqueue(tdmb_workqueue);
	tdmb_workqueue = 0;

	ts_size = 0;

  __TDMB_drv_PowerOff();

}


#if 0 //stream dump 
#define MAX_SAVE_TS_SIZE 1024*1024
static char gSaveTempTs[MAX_SAVE_TS_SIZE] = {0,};
static int gSaveTempTsIdx = 0;
//#define TDMB_FROM_FILE_BIN "/system/usr/Test.TS"
//FILE *fp1, *fp2;
#endif


int TDMB_AddDataToRing(unsigned char* pData, unsigned long dwDataSize)
{
	int ret = 0;
	unsigned int size;
	unsigned int head;
	unsigned int tail;
	unsigned int dist;
	unsigned int temp_size;

    if ( ts_size == 0 )
    {
        return ret;
    }

	size = dwDataSize;
	head = *ts_head;
	tail = *ts_tail;

	if(size > ts_size )
	{
		DPRINTK(" Error - size too large \n");
	}
	else
	{
		if( head >= tail )
			dist = head-tail;
		else
			dist = ts_size+head-tail;

		if( (ts_size-dist) < size)
		{
			DPRINTK(" too small space is left in Ring Buffer!!\n");
		}
		else
		{
			if( head+size <= ts_size )
			{
				memcpy( (ts_buffer+head), (char*)pData, size);

				head += size;
				if(head == ts_size)
					head = 0;
			}
			else
			{
				temp_size = ts_size-head;
				temp_size = (temp_size/DMB_TS_SIZE)*DMB_TS_SIZE;

				if(temp_size>0)
				{
					memcpy( (ts_buffer+head), (char*)pData, temp_size);
				}
				memcpy( ts_buffer, (char*)(pData+temp_size), size-temp_size);
				head = size-temp_size;
			}
			*ts_head = head;
		}
	}
    return ret; 
}

#ifdef TDMB_TS_DUMP_TO_FILE
void TDMB_write_to_file(void *_data, int size, int type)
{
    int writelen = -1;
    mm_segment_t oldfs;
    struct file *fp = NULL;
    struct file *fp2 = NULL;

    oldfs = get_fs();
    set_fs(KERNEL_DS);

    if(type)
    {
        if(TDMB_TS_DUMP_FILE_NUM == 1)
            fp=filp_open(TDMB_DUMP_FILE_TS_NAME_1, O_CREAT|O_APPEND |O_RDWR, 0777);
        else 
            fp=filp_open(TDMB_DUMP_FILE_TS_NAME_2, O_CREAT|O_APPEND |O_RDWR, 0777);
    }
    else
    {
        if(TDMB_MP2_DUMP_FILE_NUM == 1)
            fp=filp_open(TDMB_DUMP_FILE_MP2_NAME_1, O_CREAT|O_APPEND |O_RDWR, 0777);
        else 
            fp=filp_open(TDMB_DUMP_FILE_MP2_NAME_2, O_CREAT|O_APPEND |O_RDWR, 0777);
    }
    
    if(IS_ERR(fp)) {
        DPRINTK("[jeaman] IS_ERR(fp)" );
        set_fs(oldfs);        
        filp_close(fp,NULL);
        return;
    }

    // file size check. If file size is over 10M, change saving file
    writelen=fp->f_op->write(fp, _data, size, &fp->f_pos);   

    if(type)
    {
        if(fp->f_pos > 10000000) {
            if(TDMB_TS_DUMP_FILE_NUM == 1) {
                TDMB_TS_DUMP_FILE_NUM = 2;
                fp2=filp_open(TDMB_DUMP_FILE_TS_NAME_2, O_CREAT|O_TRUNC |O_RDWR, 0777);
                filp_close(fp2,NULL);
            } else {
                TDMB_TS_DUMP_FILE_NUM = 1;
                fp2=filp_open(TDMB_DUMP_FILE_TS_NAME_1, O_CREAT|O_TRUNC |O_RDWR, 0777);
                filp_close(fp2,NULL);
            }
        }
    }
    else
    {
        if(fp->f_pos > 10000000) {
            if(TDMB_MP2_DUMP_FILE_NUM == 1) {
                TDMB_MP2_DUMP_FILE_NUM = 2;
                fp2=filp_open(TDMB_DUMP_FILE_MP2_NAME_2, O_CREAT|O_TRUNC |O_RDWR, 0777);
                filp_close(fp2,NULL);
            } else {
                TDMB_MP2_DUMP_FILE_NUM = 1;
                fp2=filp_open(TDMB_DUMP_FILE_MP2_NAME_1, O_CREAT|O_TRUNC |O_RDWR, 0777);
                filp_close(fp2,NULL);
            }
        }
    }

    set_fs(oldfs);

    filp_close(fp,NULL);

    return;
}
#endif //TDMB_TS_DUMP_TO_FILE


#ifdef TDMB_FROM_FILE // ########################################################################################

#define TDMB_FROM_FILE_BIN "/system/usr/tdmb/Test.TS"
#define TIME_STEP	(10*HZ/100) // 0.1sec

struct file *fp_tff = NULL;
struct timer_list tff_timer;

void TDMB_drv_FromFile_Open(void);
void tfftimer_exit(void);
void tff_work_function(void);
void tfftimer_registertimer( struct timer_list *timer, unsigned long timeover);

DECLARE_WORK(tdmb_work_fromfile, tff_work_function);

void TDMB_PowerOn_FromFile(void)
{
	TDMB_drv_FromFile_Open();
}

void TDMB_PowerOff_FromFile(void)
{
	g_TDMBGlobal.b_isTDMB_Enable = 0; 

	DPRINTK("%s\n",__FUNCTION__);
	tfftimer_exit();
}
int tff_read_from_file_to_dest(char *dest, int size)
{
	int ret=0; 
	int temp_size;
	int data_received=0;
	mm_segment_t oldfs;

	temp_size= size;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	do{
		temp_size -= ret;
		ret = fp_tff->f_op->read(fp_tff, dest, temp_size, &fp_tff->f_pos);
		DPRINTK("---> file read [ret:%d] [f_pos:%d] \n", ret, fp_tff->f_pos);
		if(ret < temp_size)
		{
			DPRINTK(" file from the first \n");
			fp_tff->f_op->llseek(fp_tff, 0, SEEK_SET);
		}
		data_received += ret;
	}while(ret < temp_size);
	
	set_fs(oldfs);

	return ret;
}

void tff_work_function(void)
{
	int ret = 0;
	mm_segment_t oldfs;
	unsigned int size;
	unsigned int head;
	unsigned int tail;
	unsigned int dist;
	unsigned int temp_size;

	DPRINTK("%s  \n", __FUNCTION__);

	size = DMB_TS_SIZE*40;
	head = *ts_head;
	tail = *ts_tail;

	DPRINTK("entering head:%d,tail:%d size:%d,ps_size:%d\n",head,tail,size,ts_size);

	//
	if(size > ts_size )
	{
		DPRINTK(" Error - size too large \n");
	}
	else
	{
		if( head >= tail )
			dist = head-tail;
		else
			dist = ts_size+head-tail;

		DPRINTK("dist: %x\n", dist);

		if( (ts_size-dist) < size)
		{
			DPRINTK(" too small space is left in Ring Buffer!!\n");
		}
		else
		{

			//
			if( head+size <= ts_size )
			{
				ret=tff_read_from_file_to_dest( (ts_buffer+head), size);

				head += ret;
				if(head == ts_size)
					head = 0;
			}
			else
			{
				temp_size = ts_size-head;
				temp_size = (temp_size/DMB_TS_SIZE)*DMB_TS_SIZE;

				if(temp_size>0)
				{
					ret=tff_read_from_file_to_dest( (ts_buffer+head), temp_size );
					temp_size=ret;
				}

				ret=tff_read_from_file_to_dest( ts_buffer, size-temp_size);

				head = size-temp_size;
			}

			DPRINTK("< data > %x, %x, %x, %x \n",
						*(ts_buffer+ *ts_head),
						*(ts_buffer+ *ts_head +1),
						*(ts_buffer+ *ts_head +2),
						*(ts_buffer+ *ts_head +3) );

			DPRINTK("exiting - head : %d\n",head);
			*ts_head = head;
		}
	}

	tfftimer_registertimer(&tff_timer, TIME_STEP);
}

void tfftimer_timeover(void)
{
	int ret = 0;
	mm_segment_t oldfs;

	DPRINTK("%s\n",__FUNCTION__);

	ret=queue_work(tdmb_workqueue, &tdmb_work_fromfile);
	if(ret==0)
	{
		DPRINTK("failed in queue_work\n");
	}
	else
	{
		DPRINTK("suceeded in queue_work\n");
	}

	//tfftimer_registertimer(&tff_timer, TIME_STEP);
}

void tfftimer_registertimer(struct timer_list *timer, unsigned long timeover)
{
	DPRINTK("%s\n",__FUNCTION__);
	init_timer(timer);
	timer->expires = get_jiffies_64() + timeover;
	timer->function = tfftimer_timeover;
	add_timer(timer);
}

int tfftimer_init(void)
{
	DPRINTK("%s\n",__FUNCTION__);
	tfftimer_registertimer(&tff_timer, TIME_STEP);

	return 0;
}

void tfftimer_exit(void)
{
	DPRINTK("%s\n",__FUNCTION__);
	del_timer(&tff_timer);
}

void TDMB_drv_FromFile_Open(void)
{
	int ret = 0;
	mm_segment_t oldfs;

	DPRINTK("%s\n",__FUNCTION__);
	DPRINTK(" ##############################\n" );

	// open file
	fp_tff = filp_open(TDMB_FROM_FILE_BIN, O_RDONLY, 0);
	DPRINTK("fp_tff : %x\n", fp_tff);

    //fp_tff->f_pos = 3750600 - (3750600%188 + 3750600/188 - 188*10); // for test of boundary cond.

	// work queue init
	if(!tdmb_workqueue)
	{
		DPRINTK("tdmb_workqueue doesn't exist!\n");
		tdmb_workqueue=create_singlethread_workqueue("ktdmbd");

		if(!tdmb_workqueue)
		{
			DPRINTK("failed to create_workqueue\n");
			return TDMB_ERROR;
		}
	}

	// timer init
	tfftimer_init();
}

#endif /* TDMB_FROM_FILE  */ // ###########################################################
