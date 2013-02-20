/* drivers/media/tdmb/tdmb.c
 *
 *  TDMB Driver for Linux
 *
 *  klaatu, Copyright (c) 2009 Samsung Electronics
 *  		http://www.samsung.com/
 *
 */

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>

#include <linux/types.h>
#include <linux/fcntl.h>

// for delay(sleep)
#include <linux/delay.h>

// for mutex
#include <linux/mutex.h>

//using copy to user
#include <asm/uaccess.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include <linux/workqueue.h>
#include <linux/irq.h>
#include <asm/mach/irq.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>

#include <asm/io.h>
//#include <asm/arch/regs-gpio.h>

//#include <plat/regs-gpio.h>
//#include <plat/gpio-cfg.h>
//#include <plat/gpio-bank-j4.h>
#include <mach/gpio.h>

#include "tdmb.h"


#ifdef CONFIG_TDMB_T3700
#include "INC_INCLUDES.h"
#endif
#ifdef CONFIG_TDMB_FC8050
#include "DMBDrv_wrap_FC8050.h"
#endif

#define TDMB_DEBUG

#ifdef TDMB_DEBUG
#define DPRINTK(x...) printk("[TDMB] " x)
#else /* TDMB_DEBUG */
#define DPRINTK(x...)  /* null */
#endif /* TDMB_DEBUG */

#define TDMB_DEV_NAME "tdmb"
#define TDMB_DEV_MAJOR 225
#define TDMB_DEV_MINOR 0
#define TDMB_PRE_MALLOC 1

// global variables --------------------------------

static struct class *tdmb_class;
static int tdmb_major;

// ring buffer
char * TS_RING = NULL;
unsigned int *ts_head = NULL;
unsigned int *ts_tail = NULL ;
char *ts_buffer = NULL ;
unsigned int ts_size = 0;//NULL;

unsigned int *cmd_head = NULL;
unsigned int *cmd_tail = NULL ;
char *cmd_buffer = NULL ;
unsigned int cmd_size = 0;//NULL;

#ifdef CONFIG_TDMB_T3700 
extern int UpdateEnsembleInfo(EnsembleInfoType* ensembleInfo, unsigned long freq);
#if 1 // T3700 new version
ST_SUBCH_INFO* g_pStChInfo = NULL;
#else
INC_CHANNEL_INFO* g_pStChInfo = NULL;
#endif
#endif
// extern -----------------------------------------
extern void TDMBDrv_PowerInit(void);

extern tdmb_type g_TDMBGlobal;


#ifdef CONFIG_TDMB_SPI	
struct spi_device *spi_dmb;
static int tdmbspi_probe(struct spi_device *spi)
{
	//	struct tdmb_spi *test;

	spi_dmb = spi;

	printk("tdmbspi_probe() \n");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	return 0;
}


static int __devexit tdmbspi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver tdmbspi_driver = {
	.driver = {
		.name	= "tdmbspi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= tdmbspi_probe,
	.remove		= __devexit_p(tdmbspi_remove),
};

int tdmbspi_init(void)
{
	printk("This is test program for S3C64XX's SPI Driver\n");

	return spi_register_driver(&tdmbspi_driver);
}

void tdmbspi_exit(void)
{
	spi_unregister_driver(&tdmbspi_driver);
}

void spi_open(void)
{
	printk("spi_open()!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

void spi_close(void)
{
	printk("spi_close()!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}
#endif
#ifdef CONFIG_TDMB_EBI
#include <linux/io.h>
#define TDMB_BASE_ADDR_PHYS 0x98000000
void* addr_TDMB_CS4_V;

int tdmb_ebi2_init(void)
{
    addr_TDMB_CS4_V = ioremap(TDMB_BASE_ADDR_PHYS, PAGE_SIZE);
    printk("TDMB EBI2 Init addr_TDMB_CS4_V(0x%x)\n", addr_TDMB_CS4_V);
    return 0;
}
#endif

int tdmb_open(struct inode *inode, struct file *filp)
{
	DPRINTK("tdmb_open! \r\n");

	return 0;
}

int tdmb_read(struct inode *inode, struct file *filp)
{
	DPRINTK("tdmb_read! \r\n");

	return 0;
}


#ifdef TDMB_FROM_FILE
extern void tfftimer_exit(void);
#endif

int tdmb_release(struct inode *inode, struct file *filp)
{
	DPRINTK("tdmb_release! \r\n");

#ifdef TDMB_FROM_FILE
	tfftimer_exit();
#endif
    // For tdmb_release() without TDMB POWER OFF (App abnormal -> kernal panic)
    if(IsTDMBPowerOn())
    {
        TDMB_PowerOff();
    }

#if TDMB_PRE_MALLOC
    ts_size = 0;
    cmd_size = 0;
#else
	if(TS_RING != 0)
	{
		kfree(TS_RING);
		TS_RING = 0;
        ts_size = 0;
        cmd_size = 0;
	}
#endif    
	return 0;
}

#if TDMB_PRE_MALLOC
int tdmb_makeRingBuffer()
{
	size_t size = TDMB_RING_BUFFER_MAPPING_SIZE;

	// size should aligned in PAGE_SIZE
	if(size % PAGE_SIZE) // klaatu hard coding
		size = size + size % PAGE_SIZE;
	
	TS_RING = kmalloc(size, GFP_KERNEL);
    DPRINTK("RING Buff Create OK\n");
}

#endif

int tdmb_mmap(struct file *filp, struct vm_area_struct *vma)
{
	size_t size;
	unsigned long pfn;
	
	DPRINTK(" %s \n", __FUNCTION__);

	vma->vm_flags |= VM_RESERVED;
	size = vma->vm_end - vma->vm_start;
	DPRINTK("size given : %x\n",size);


#if TDMB_PRE_MALLOC
    size = TDMB_RING_BUFFER_MAPPING_SIZE;
    if(!TS_RING)
    {
        DPRINTK("RING Buff ReAlloc !!\n",size);
#endif
	// size should aligned in PAGE_SIZE
	if(size % PAGE_SIZE) // klaatu hard coding
		size = size + size % PAGE_SIZE;
	
	TS_RING = kmalloc(size, GFP_KERNEL);
#if TDMB_PRE_MALLOC
    }
#endif
	pfn = virt_to_phys(TS_RING) >> PAGE_SHIFT;

	DPRINTK("vm_start:%x,TS_RING:%x,size:%x,prot:%x,pfn:%x\n", 
			vma->vm_start, TS_RING, size, vma->vm_page_prot,pfn);

	if(remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot))
		return -EAGAIN;

	DPRINTK(" succeeded \n");

	ts_head   = TS_RING;
	ts_tail   = TS_RING + 4;
	ts_buffer = TS_RING + 8;

	*ts_head = 0;
	*ts_tail = 0;

	ts_size = size-8; // klaatu hard coding
	ts_size = (ts_size/DMB_TS_SIZE)*DMB_TS_SIZE - 30*DMB_TS_SIZE;

	DPRINTK("ts_head : %x, ts_tail : %x, ts_buffer : %x,ts_size : %x \n", 
			ts_head, ts_tail, ts_buffer, ts_size);
	 
	cmd_buffer = ts_buffer + ts_size + 8;
	cmd_head   = cmd_buffer - 8;
	cmd_tail   = cmd_buffer - 4;

	*cmd_head = 0;
	*cmd_tail = 0;

	cmd_size = 30*DMB_TS_SIZE - 8; // klaatu hard coding

	DPRINTK("cmd_head : %x, cmd_tail : %x, cmd_buffer : %x,cmd_size : %x \n", 
			cmd_head, cmd_tail, cmd_buffer, cmd_size);

	 
	return 0;
}


int _tdmb_cmd_update(unsigned char* byCmdsHeader, unsigned char byCmdsHeaderSize, unsigned char* byCmds, unsigned short bySize)
{
	unsigned int size;
	unsigned int head;
	unsigned int tail;
	unsigned int dist;
	unsigned int temp_size;
	unsigned int dataSize;	

	if(bySize > cmd_size )
	{
		DPRINTK(" Error - cmd size too large \n");
		return FALSE;
	}

	head = *cmd_head;
	tail = *cmd_tail;
    size = cmd_size;
    dataSize = bySize + byCmdsHeaderSize;

	if( head >= tail )
		dist = head-tail;
	else
		dist = size + head-tail;
	
	if( size - dist <= dataSize )
	{
		DPRINTK(" too small space is left in Command Ring Buffer!!\n");
		return FALSE;
	}

    DPRINTK(" Error - %x head %d tail %d\n", cmd_buffer, head, tail);

    if( head+dataSize <= size )
    {
        memcpy( (cmd_buffer+head), (char*)byCmdsHeader, byCmdsHeaderSize); 
        memcpy( (cmd_buffer+head+byCmdsHeaderSize), (char*)byCmds, size);   
        head += dataSize;
        if(head == size)
            head = 0;
    }
    else
    {
        temp_size = size-head;
        if ( temp_size < byCmdsHeaderSize )
        {
            memcpy( (cmd_buffer+head), (char*)byCmdsHeader, temp_size);
            memcpy( (cmd_buffer), (char*)byCmdsHeader+temp_size, byCmdsHeaderSize-temp_size);
            head = byCmdsHeaderSize-temp_size;
        }
        else 
        {
            memcpy( (cmd_buffer+head), (char*)byCmdsHeader, byCmdsHeaderSize);
            head += byCmdsHeaderSize;
            if(head == size)
                head = 0;
        }
        temp_size = size-head;
        memcpy( (cmd_buffer+head), (char*)byCmds, temp_size);
        head = dataSize-temp_size;        
        memcpy( cmd_buffer, (char*)(byCmds+temp_size), head);
    }
    *cmd_head = head;
	return TRUE ;
}

unsigned char _tdmb_make_result(unsigned char byCmd, unsigned short byDataLength, unsigned char* pbyData)
{
	unsigned char byCmds[256] = { 0, } ;
	
	byCmds[0] = TDMB_CMD_START_FLAG ;
	byCmds[1] = byCmd ;
	byCmds[2] = (byDataLength>>8)&0xff ;
	byCmds[3] = byDataLength&0xff ;

#if 0	
	if (byDataLength > 0)
	{
		if (pbyData == NULL)
		{
		    // to error 
			return FALSE;
	    }
        memcpy(byCmds + 8, pbyData, byDataLength) ;
	}
#endif	
	_tdmb_cmd_update( byCmds, 4 , pbyData,  byDataLength ) ;
	
	return TRUE ;
}


int UpdateEnsembleInfo(EnsembleInfoType* ensembleInfo, unsigned long freq)
{
    int i;
    int nSubChIdx = 0;
    int j, nCnt;
    const char * ensembleName = NULL;

    DPRINTK("UpdateEnsembleInfo - freq(%d)\r\n", freq);                                        

#ifdef CONFIG_TDMB_T3700 
    INC_CHANNEL_INFO* pINC_SubChInfo;

    if (INTERFACE_GETDMB_CNT() + INTERFACE_GETDAB_CNT() > 0)
    {
        ensembleInfo->TotalSubChNumber = 0;
        ensembleName = (char *)INTERFACE_GETENSEMBLE_LABEL(TDMB_I2C_ID80); //TDMB_T3700

        if (ensembleName)
        {
            strncpy((char *)ensembleInfo->EnsembleLabelCharField, (char *)ensembleName, ENSEMBLE_LABEL_SIZE_MAX);
        }
        ensembleInfo->EnsembleFrequency = freq;

        for ( i=0 ; i<2 ; i++ )
        {
            nCnt = (i==0)?INTERFACE_GETDMB_CNT():INTERFACE_GETDAB_CNT();

            for ( j=0 ; j<nCnt ; j++, nSubChIdx++ )
            {
                pINC_SubChInfo = (i==0)?INTERFACE_GETDB_DMB(j):INTERFACE_GETDB_DAB(j);
                ensembleInfo->SubChInfo[nSubChIdx].SubChID      = pINC_SubChInfo->ucSubChID;
                ensembleInfo->SubChInfo[nSubChIdx].StartAddress = pINC_SubChInfo->uiStarAddr;
                ensembleInfo->SubChInfo[nSubChIdx].TMId         = pINC_SubChInfo->uiTmID;
                ensembleInfo->SubChInfo[nSubChIdx].Type         = pINC_SubChInfo->ucServiceType;
                ensembleInfo->SubChInfo[nSubChIdx].ServiceID    = pINC_SubChInfo->ulServiceID;
                memcpy(ensembleInfo->SubChInfo[nSubChIdx].ServiceLabel, pINC_SubChInfo->aucLabel, SERVICE_LABEL_SIZE_MAX);
            }
        }
    }

    ensembleInfo->TotalSubChNumber = nSubChIdx;
    
#elif defined(CONFIG_TDMB_FC8050)
    SubChInfoTypeDB* pFCI_SubChInfo;

    if (DMBDrv_GetDMBSubChCnt() + DMBDrv_GetDABSubChCnt() > 0)
    {
        ensembleInfo->TotalSubChNumber = 0;
        ensembleName = (char *)DMBDrv_GetEnsembleLabel();
        if (ensembleName)
        {
            strncpy((char *)ensembleInfo->EnsembleLabelCharField, (char *)ensembleName, ENSEMBLE_LABEL_SIZE_MAX);
        }
        ensembleInfo->EnsembleFrequency = freq;

        DPRINTK("UpdateEnsembleInfo - ensembleName(%s), \r\n", ensembleName);                                        

        for ( i=0 ; i<2 ; i++ )
        {
            nCnt = (i==0)?DMBDrv_GetDMBSubChCnt() : DMBDrv_GetDABSubChCnt();
            for ( j=0 ; j<nCnt ; j++, nSubChIdx++ )
            {
                pFCI_SubChInfo = (i==0)?DMBDrv_GetFICDMB(j) : DMBDrv_GetFICDAB(j);

                ensembleInfo->EnsembleID                        = pFCI_SubChInfo->uiEnsembleID;
                ensembleInfo->SubChInfo[nSubChIdx].SubChID      = pFCI_SubChInfo->ucSubchID;
                ensembleInfo->SubChInfo[nSubChIdx].StartAddress = pFCI_SubChInfo->uiStartAddress;
                ensembleInfo->SubChInfo[nSubChIdx].TMId         = pFCI_SubChInfo->ucTMId;
                ensembleInfo->SubChInfo[nSubChIdx].Type         = pFCI_SubChInfo->ucServiceType;
                ensembleInfo->SubChInfo[nSubChIdx].ServiceID    = pFCI_SubChInfo->ulServiceID;
                if(i==0)
                {
                    memcpy(ensembleInfo->SubChInfo[nSubChIdx].ServiceLabel, (char*)DMBDrv_GetSubChDMBLabel(j), SERVICE_LABEL_SIZE_MAX);
                }
                else
                {
                    memcpy(ensembleInfo->SubChInfo[nSubChIdx].ServiceLabel, (char*)DMBDrv_GetSubChDABLabel(j), SERVICE_LABEL_SIZE_MAX);
                }
                DPRINTK("UpdateEnsembleInfo - EnsembleID(%d) \r\n", ensembleInfo->EnsembleID);                                        
                DPRINTK("UpdateEnsembleInfo - SubChID(0x%x), StartAddress(0x%x) \r\n", ensembleInfo->SubChInfo[nSubChIdx].SubChID, ensembleInfo->SubChInfo[nSubChIdx].StartAddress);                
                DPRINTK("UpdateEnsembleInfo - TMId(0x%x), Type(0x%x) \r\n", ensembleInfo->SubChInfo[nSubChIdx].TMId, ensembleInfo->SubChInfo[nSubChIdx].Type);                
                DPRINTK("UpdateEnsembleInfo - ServiceID(0x%x) \r\n", ensembleInfo->SubChInfo[nSubChIdx].ServiceID);                                
                DPRINTK("UpdateEnsembleInfo - ServiceLabel(%s), \r\n", ensembleInfo->SubChInfo[nSubChIdx].ServiceLabel);                                                
            }
        }
    }        
    ensembleInfo->TotalSubChNumber = nSubChIdx;
    
#endif
    return nSubChIdx;
}

static int tdmb_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret =0;
    unsigned long ulFreq = 0;
    unsigned char subChID = 0, svcType = 0;
    
    DPRINTK("call tdmb_ioctl : %d \r\n", cmd);

    if(!IsTDMBPowerOn())
    {
        if ( cmd == IOCTL_TDMB_POWER_OFF )
        {
            DPRINTK("%d cmd : current state poweroff \r\n", cmd);
            return TRUE;
        }
        else if ( cmd > IOCTL_TDMB_POWER_ON )
        {
            DPRINTK("error %d cmd : current state poweroff \r\n", cmd);
            return FALSE;
        }
    }
    else
    {
        if ( cmd == IOCTL_TDMB_POWER_ON )
        {
            DPRINTK("%d cmd : current state poweron \r\n", cmd);
            return TRUE;
        }
    }
    
    switch(cmd)
    {
        case IOCTL_TDMB_GET_DATA_BUFFSIZE :
            DPRINTK("IOCTL_TDMB_GET_DATA_BUFFSIZE %d \r\n", ts_size);
            ret = copy_to_user( (unsigned int*)arg, &ts_size, sizeof(unsigned int));
            break;
            
        case IOCTL_TDMB_GET_CMD_BUFFSIZE :
            DPRINTK("IOCTL_TDMB_GET_CMD_BUFFSIZE %d \r\n", cmd_size);
            ret = copy_to_user( (unsigned int*)arg, &cmd_size, sizeof(unsigned int));
            break;
            
        case IOCTL_TDMB_POWER_ON:
            DPRINTK("IOCTL_TDMB_POWER_ON \n");
            ret = TDMB_PowerOn();
#ifdef CONFIG_TDMB_T3700 
            if ( g_pStChInfo == NULL )
            {
                g_pStChInfo = vmalloc (sizeof(ST_SUBCH_INFO));
                if ( g_pStChInfo == NULL )
                {
                    TDMB_PowerOff();
                    ret = FALSE;
                    printk("tdmb vmalloc error \n");
                }
            }
#endif
            DPRINTK("IOCTL_TDMB_POWER_ON 2 \n");
            break;

        case IOCTL_TDMB_POWER_OFF:
#ifdef CONFIG_TDMB_T3700 
            if ( g_pStChInfo != NULL )
            {
                vfree(g_pStChInfo);
                g_pStChInfo = NULL;
            }
#endif      
            DPRINTK("IOCTL_TDMB_POWER_OFF \r\n");
            TDMB_PowerOff();
            ret = TRUE;
            break;


        case IOCTL_TDMB_SCAN_FREQ_ASYNC:
            {
                unsigned long FIG_Frequency;
                FIG_Frequency = arg;
                DPRINTK("IOCTL_TDMB_SCAN_FREQ_ASYNC \r\n");
#if defined(CONFIG_TDMB_T3700)
                ulFreq = arg / 1000;
                if(INTERFACE_SCAN(TDMB_I2C_ID80, ulFreq) == INC_SUCCESS)
                {
                    //TODO Scan good code ....
                    ret = TRUE;
                }

#elif defined(CONFIG_TDMB_FC8050)
                ulFreq = arg / 1000;
                if(DMBDrv_ScanCh(ulFreq)==TDMB_SUCCESS)
                {
                    //TODO Scan good code ....
                    ret = TRUE;
                }
#endif              
                if ( ret == TRUE )
                {
                    EnsembleInfoType* pEnsembleInfo = vmalloc(sizeof(EnsembleInfoType));
                    if (pEnsembleInfo != NULL )
                    {
                        memset((char*)pEnsembleInfo, 0x00, sizeof(EnsembleInfoType));
                        UpdateEnsembleInfo(pEnsembleInfo, FIG_Frequency);
                        _tdmb_make_result( DMB_FIC_RESULT_DONE, sizeof(EnsembleInfoType), pEnsembleInfo);
                        vfree(pEnsembleInfo);
                    }
                }
                else
                {
                    _tdmb_make_result( DMB_FIC_RESULT_FAIL, sizeof(unsigned long), &FIG_Frequency);                 
                }
            }
            break;

        case IOCTL_TDMB_SCAN_FREQ_SYNC:
            {
                EnsembleInfoType* pEnsembleInfo = (EnsembleInfoType*)arg;
                unsigned long FIG_Frequency = pEnsembleInfo->EnsembleFrequency;

                DPRINTK("IOCTL_TDMB_SCAN_FREQ_SYNC - %d \r\n", FIG_Frequency);
#if defined(CONFIG_TDMB_T3700)
                ulFreq = pEnsembleInfo->EnsembleFrequency / 1000;
                if(INTERFACE_SCAN(TDMB_I2C_ID80, ulFreq) == INC_SUCCESS)
                {
                    //TODO Scan good code ....
                    ret = TRUE;
                }    
#elif defined(CONFIG_TDMB_FC8050)
                ulFreq = pEnsembleInfo->EnsembleFrequency / 1000;
                if(DMBDrv_ScanCh(ulFreq)==TDMB_SUCCESS)
                {
                    DPRINTK("DMBDrv_ScanCh Success \r\n");                                        
                    //TODO Scan good code ....
                    ret = TRUE;
                }
                else
                {
                    DPRINTK("DMBDrv_ScanCh Fail \r\n");                                        
                }
#endif              
                if ( ret == TRUE )
                {
                    EnsembleInfoType* pTempEnsembleInfo = vmalloc(sizeof(EnsembleInfoType));
                    if (pTempEnsembleInfo != NULL )
                    {
                        memset((char*)pTempEnsembleInfo, 0x00, sizeof(EnsembleInfoType));
                        UpdateEnsembleInfo(pTempEnsembleInfo, FIG_Frequency);
                        copy_to_user( (EnsembleInfoType*)arg, pTempEnsembleInfo, sizeof(EnsembleInfoType));
                        vfree(pTempEnsembleInfo);
                    }
                }
                else
                {
                    //
                }
            }
            break;

        case IOCTL_TDMB_SCANSTOP:
            DPRINTK("IOCTL_TDMB_SCANSTOP \r\n");
#if defined(CONFIG_TDMB_T3700)
            ret = FALSE; //temp
#elif defined(CONFIG_TDMB_FC8050)
            ret = FALSE; //temp            
#endif          
            break;          

        case IOCTL_TDMB_ASSIGN_CH :
            DPRINTK("IOCTL_TDMB_ASSIGN_CH %d\r\n", arg);
#if defined(CONFIG_TDMB_T3700)
#if 1 // T3700 new version
            if ( g_pStChInfo != NULL )
            {
                INC_UINT8 reErr;            
				
                g_pStChInfo->nSetCnt = 1;
                g_pStChInfo->astSubChInfo[0].ulRFFreq = arg/1000 ;
                g_pStChInfo->astSubChInfo[0].ucSubChID = arg%1000 ;
                g_pStChInfo->astSubChInfo[0].ucServiceType = 0x0;             
                if (g_pStChInfo->astSubChInfo[0].ucSubChID >= 64 )
                {
                    g_pStChInfo->astSubChInfo[0].ucSubChID -= 64;
                    g_pStChInfo->astSubChInfo[0].ucServiceType  = 0x18;
                }            
	            
	            if((reErr = INTERFACE_START(TDMB_I2C_ID80, g_pStChInfo)) == INC_SUCCESS)
                {
                    //TODO Ensemble  good code ....
                    ret = TRUE;
                }
                else if (reErr == INC_RETRY )
                {
                    int temp_ts_size = ts_size;
                    DPRINTK("IOCTL_TDMB_ASSIGN_CH retry\r\n");
                    TDMB_PowerOff();
                    TDMB_PowerOn();
                    ts_size = temp_ts_size;
		            if(INTERFACE_START(TDMB_I2C_ID80, g_pStChInfo) == INC_SUCCESS)
                    {
                        ret = TRUE;
                    }               
                }
            }
#else
            {   
                INC_UINT8 reErr;            
         
                g_pStChInfo->ulRFFreq       = arg/1000 ;
                g_pStChInfo->ucSubChID      = arg%1000;
                g_pStChInfo->ucServiceType  = 0x0;
                if (g_pStChInfo->ucSubChID >= 64 )
                {
                    g_pStChInfo->ucSubChID -= 64;
                    g_pStChInfo->ucServiceType  = 0x18;
                }            
                if((reErr = INTERFACE_START(T3700_I2C_ID80, g_pStChInfo)) == INC_SUCCESS)
                {
                    //TODO Ensemble  good code ....
                    ret = TRUE;
                }
                else if (reErr == INC_RETRY )
                {
                    int temp_ts_size = ts_size;
                    TDMB_PowerOff();
                    TDMB_PowerOn();
                    ts_size = temp_ts_size;
                    if(INTERFACE_START(T3700_I2C_ID80, g_pStChInfo) == INC_SUCCESS)
                    {
                        ret = TRUE;
                    }               
                }
            }
#endif
#elif defined(CONFIG_TDMB_FC8050)
            {
                ulFreq  = arg / 1000;
                subChID = arg%1000;
                svcType = 0x0;
                DPRINTK("IOCTL_TDMB_ASSIGN_CH ulFreq:%d, subChID:%d, svcType:%d\r\n", ulFreq, subChID, svcType);
                
                if (subChID >= 64 )
                {
                    subChID -= 64;
                    svcType  = 0x18;
                }            
                if(DMBDrv_SetCh(ulFreq, subChID, svcType) == 1)
                {
                    DPRINTK("DMBDrv_SetCh Success \r\n");                                        
                    ret = TRUE;
                }
                else
                {
                    DPRINTK("DMBDrv_SetCh Fail \r\n");                                                       
                }
            }
#endif          
            break;          

        case IOCTL_TDMB_GET_DM :
            {
                tdmb_dm dmBuff;          
#if defined(CONFIG_TDMB_T3700)
                extern INC_UINT8 INC_GET_SAMSUNG_ANT_LEVEL(INC_UINT8 ucI2CID);

#if 1 // T3700 new version
			    dmBuff.rssi = INC_GET_RSSI(TDMB_I2C_ID80/*TDMB_I2C_ID80*/);
			    dmBuff.BER = INC_GET_SAMSUNG_BER(TDMB_I2C_ID80/*TDMB_I2C_ID80*/);
                dmBuff.PER = 0;
			    dmBuff.antenna = INC_GET_SAMSUNG_ANT_LEVEL(TDMB_I2C_ID80/*TDMB_I2C_ID80*/);
#else
                dmBuff.rssi = INC_GET_RSSI(T3700_I2C_ID80/*T3700_I2C_ID80*/);
                dmBuff.BER = INC_GET_SAMSUNG_BER(T3700_I2C_ID80/*T3700_I2C_ID80*/);
                dmBuff.PER = 0;
                dmBuff.antenna = INC_GET_SAMSUNG_ANT_LEVEL(T3700_I2C_ID80/*T3700_I2C_ID80*/);
#endif
#elif defined(CONFIG_TDMB_FC8050)
                dmBuff.antenna = DMBDrv_GetAntLevel();
                dmBuff.rssi = DMBDrv_GetRSSI();
                dmBuff.BER = DMBDrv_GetBER();
                dmBuff.PER = 0;
#endif
                ret = copy_to_user( (tdmb_dm*)arg, &dmBuff, sizeof(tdmb_dm));
                DPRINTK("rssi %d, ber %d, ANT %d\r\n", dmBuff.rssi, dmBuff.BER, dmBuff.antenna);
                // to do...

            }
            break;
            
        case IOCTL_TDMB_ASSIGN_CH_TEST :
            printk("IOCTL_TDMB_ASSIGN_CH_TEST %d\r\n", arg);
#if defined(CONFIG_TDMB_T3700)
#if 1 // T3700 new version
            if ( g_pStChInfo != NULL )
            {
                INC_UINT8 reErr;            
				
                g_pStChInfo->nSetCnt = 1;
                g_pStChInfo->astSubChInfo[0].ulRFFreq = arg/1000 ;
                g_pStChInfo->astSubChInfo[0].ucSubChID = arg%1000 ;
                g_pStChInfo->astSubChInfo[0].ucServiceType = 0x0;             
                if (g_pStChInfo->astSubChInfo[0].ucSubChID >= 64 )
                {
                    g_pStChInfo->astSubChInfo[0].ucSubChID -= 64;
                    g_pStChInfo->astSubChInfo[0].ucServiceType  = 0x18;
                }            
	            
	            if((reErr = INTERFACE_START_TEST(TDMB_I2C_ID80, g_pStChInfo)) == INC_SUCCESS)
                {
                    //TODO Ensemble  good code ....
                    ret = TRUE;
                }
                else if (reErr == INC_RETRY )
                {
                    int temp_ts_size = ts_size;
                    DPRINTK("IOCTL_TDMB_ASSIGN_CH retry\r\n");
                    TDMB_PowerOff();
                    TDMB_PowerOn();
                    ts_size = temp_ts_size;
		            if(INTERFACE_START_TEST(TDMB_I2C_ID80, g_pStChInfo) == INC_SUCCESS)
                    {
                        ret = TRUE;
                    }               
                }
            }
#else
            {   
                INC_UINT8 reErr;            
         
                g_pStChInfo->ulRFFreq       = arg/1000 ;
                g_pStChInfo->ucSubChID      = arg%1000;
                g_pStChInfo->ucServiceType  = 0x0;
                if (g_pStChInfo->ucSubChID >= 64 )
                {
                    g_pStChInfo->ucSubChID -= 64;
                    g_pStChInfo->ucServiceType  = 0x18;
                }            
                if((reErr = INTERFACE_START_TEST(T3700_I2C_ID80, g_pStChInfo)) == INC_SUCCESS)
                {
                    //TODO Ensemble  good code ....
                    ret = TRUE;
                }
                else if (reErr == INC_RETRY )
                {
                    int temp_ts_size = ts_size;
                    TDMB_PowerOff();
                    TDMB_PowerOn();
                    ts_size = temp_ts_size;
                    if(INTERFACE_START_TEST(T3700_I2C_ID80, g_pStChInfo) == INC_SUCCESS)
                    {
                        ret = TRUE;
                    }               
                }
            }
#endif

#elif defined(CONFIG_TDMB_FC8050)
            {
                ulFreq  = arg / 1000;
                subChID = arg%1000;
                svcType = 0x0;
                DPRINTK("IOCTL_TDMB_ASSIGN_CH ulFreq:%d, subChID:%d, svcType:%d\r\n", ulFreq, subChID, svcType);
                
                if (subChID >= 64 )
                {
                    subChID -= 64;
                    svcType  = 0x18;
                }            
                if(DMBDrv_SetCh(ulFreq, subChID, svcType) == 1)
                {
                    ret = TRUE;
                }
            }            
#endif

            break;              
            
    }
    return ret;
}

static struct file_operations tdmb_ctl_fops = {
	owner:		THIS_MODULE,
	open:		tdmb_open,
	read:		tdmb_read,
	ioctl:		tdmb_ioctl,
	mmap:		tdmb_mmap,
	release:	tdmb_release,
	llseek:		no_llseek,
};

int tdmb_probe(struct platform_device *pdev)
{
	int ret;
	struct device *tdmb_dev_t;

	DPRINTK("call tdmb_probe\r\n");

	ret=register_chrdev(TDMB_DEV_MAJOR, TDMB_DEV_NAME, &tdmb_ctl_fops);
	if(ret < 0)
	{
		DPRINTK("register_chrdev(TDMB_DEV) failed !\r\n");
	}
	
	tdmb_class = class_create(THIS_MODULE, TDMB_DEV_NAME);
	if(IS_ERR(tdmb_class))
	{
		unregister_chrdev(TDMB_DEV_MAJOR, TDMB_DEV_NAME);
		class_destroy(tdmb_class);
		DPRINTK("class_create failed !\r\n");
		return -EFAULT;
	}

	tdmb_major = TDMB_DEV_MAJOR;
	tdmb_dev_t = device_create(tdmb_class, NULL, MKDEV(tdmb_major,0), NULL, TDMB_DEV_NAME);
	if(IS_ERR(tdmb_dev_t))
	{
		DPRINTK("device_create failed !\r\n");
		return -EFAULT;
	}

#if defined(CONFIG_TDMB_SPI)
	tdmbspi_init(); 
#endif    
#ifdef CONFIG_TDMB_EBI
  tdmb_ebi2_init();
#endif
  TDMBDrv_PowerInit();
  
#if TDMB_PRE_MALLOC
  tdmb_makeRingBuffer();
#endif
	return 0;
}

int tdmb_remove(struct platform_device *pdev)
{
	DPRINTK("Call tdmb_remove! \r\n");

	return 0;
}

int tdmb_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	DPRINTK("Call tdmb_suspend! \r\n");

	return 0;
}

int tdmb_resume(struct platform_device *pdev, pm_message_t mesg)
{
	DPRINTK("Call tdmb_resume! \r\n");

	return 0;
}

static struct platform_device *tdmbdrv_device;

static struct platform_driver tdmb_driver = {
	.probe	= tdmb_probe,
	.remove	= tdmb_remove,
	.suspend	= tdmb_suspend,
	.resume	=	tdmb_resume,
	.driver	= {
			.owner	= THIS_MODULE,
			.name	= "tdmb"
	},
};


static int __init tdmb_init(void)
{
	int ret;

	DPRINTK("<klaatu TDMB> module init\n");
	ret = platform_driver_register(&tdmb_driver);
	if(ret)
	{
		return ret;
	}

	DPRINTK("platform_driver_register! \r\n");
	tdmbdrv_device = platform_device_register_simple("tdmb",-1,NULL,0);
	if(IS_ERR(tdmbdrv_device))
	{
			DPRINTK("platform_device_register! \r\n");
			return PTR_ERR(tdmbdrv_device);
	}

	g_TDMBGlobal.b_isTDMB_Enable = 0;

	return 0;
}

static void __exit tdmb_exit(void)
{
	DPRINTK("<klaatu TDMB> module exit\n");
#if TDMB_PRE_MALLOC
	if(TS_RING != 0)
	{
		kfree(TS_RING);
		TS_RING = 0;
	}
#endif   
	unregister_chrdev(TDMB_DEV_MAJOR,"tdmb");
	device_destroy(tdmb_class, MKDEV(tdmb_major,0));
	class_destroy(tdmb_class);

	platform_device_unregister(tdmbdrv_device);
	platform_driver_unregister(&tdmb_driver);
}

module_init(tdmb_init);
module_exit(tdmb_exit);

MODULE_AUTHOR("Samsung");
MODULE_DESCRIPTION("TDMB Driver(T3700)");
MODULE_LICENSE("GPL v2");

