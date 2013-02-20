ifeq ($(CONFIG_MACH_COOPER_BASE_KOR),y)
EXTRA_CFLAGS += -Dlinux -DLINUX -DBDC -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2 \
    -DDHD_GPL -DDHD_SCHED -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS -DDHD_BCMEVENTS      \
    -DSHOW_EVENTS -DANDROID_SPECIFIC -DENABLE_DEEP_SLEEP

################ OPTIONAL FEATURES ############################################################
#TO ENALBLE OPTIONAL FEATURES UNCOMMENT THE CORRESPONDING FLAGS

# For Samsung Aries only.
EXTRA_CFLAGS += -DCUSTOMER_HW_SAMSUNG

# For MACID management
EXTRA_CFLAGS += -DREAD_MACADDR

# For 5G scan home channel time. (default=45 -> 60)
ifeq ($(CONFIG_MACH_JUNO_SKT),y)  
EXTRA_CFLAGS += -DSCAN_5G_HOMECHANNEL_TIME  
EXTRA_CFLAGS += -DSUPPORT_5G_BANDWIDTH
else ifeq ($(CONFIG_MACH_JUNO_KT),y)  
EXTRA_CFLAGS += -DSCAN_5G_HOMECHANNEL_TIME  
EXTRA_CFLAGS += -DSUPPORT_5G_BANDWIDTH
endif

# For AUTH time patch
EXTRA_CFLAGS += -DAUTH_TIME_PATCH

# Check if this is required. SDIO_ISR_THREAD is mutually exclusive with OOB_INTR_ONLY
#EXTRA_CFLAGS += -DSDIO_ISR_THREAD

# For OOB only
EXTRA_CFLAGS += -DOOB_INTR_ONLY

# For HW_OOB
#EXTRA_CFLAGS += -DHW_OOB

# FOR SOFTAP
EXTRA_CFLAGS += -DSOFTAP

# For Debug
#EXTRA_CFLAGS += -DDHD_DEBUG -DSRCBASE=\"$(src)/src\"

# HOST WAKEUP
EXTRA_CFLAGS += -DBCM_HOSTWAKE

# ROAM_TEST_CODE
#EXTRA_CFLAGS += -DROAM_TEST_CODE

# CONTROL ROAMMING
#EXTRA_CFLAGS += -DCONTROL_ROAMMING

# PRINTF FW LOG
#EXTRA_CFLAGS += -DPRINT_FW_LOG
#EXTRA_CFLAGS += -DROAM_DEBUG

# OKC
EXTRA_CFLAGS += -DUSE_OKC

# Keep Alive
EXTRA_CFLAGS += -DUSE_KEEP_ALIVE

#STATIC MEMORY ALLOCATION FEATURE
ifeq ($(CONFIG_BROADCOM_WIFI_RESERVED_MEM),y)
    EXTRA_CFLAGS += -DDHD_USE_STATIC_BUF
endif

EXTRA_CFLAGS += -DEMBEDDED_PLATFORM

EXTRA_CFLAGS += -DARP_OFFLOAD_SUPPORT
EXTRA_CFLAGS += -DPKT_FILTER_SUPPORT
#Disable TPC feature for FCC 
#EXTRA_CFLAGS += -DFCC_CERT

# Scan Method - ESCAN
#EXTRA_CFLAGS += -DWL_IW_USE_ISCAN
EXTRA_CFLAGS += -DWL_IW_USE_ESCAN
#Disable PowerSave mode for OTA or certification test
#EXTRA_CFLAGS += -DBCMDISABLE_PM

EXTRA_CFLAGS += -DCONFIG_TARGET_LOCALE_KOR
###############################################################################################

EXTRA_CFLAGS += -I$(src)/src/include/
EXTRA_CFLAGS += -I$(src)/src/dhd/sys/
EXTRA_CFLAGS += -I$(src)/src/dongle/
EXTRA_CFLAGS += -I$(src)/src/bcmsdio/sys/
EXTRA_CFLAGS += -I$(src)/src/wl/sys/
EXTRA_CFLAGS += -I$(src)/src/shared/

EXTRA_LDFLAGS += --strip-debug

KBUILD_CFLAGS += -I$(LINUXDIR)/include -I$(shell pwd)


#obj-$(CONFIG_BROADCOM_WIFI)-m	+= dhd.o
obj-m	+= dhd.o

dhd-y := src/dhd/sys/dhd_linux.o \
         src/dhd/sys/dhd_common.o \
         src/dhd/sys/dhd_cdc.o \
         src/dhd/sys/dhd_linux_sched.o\
         src/dhd/sys/dhd_sdio.o \
         src/dhd/sys/dhd_custom_gpio.o \
         src/shared/aiutils.o \
         src/shared/bcmutils.o \
         src/shared/bcmwifi.o \
         src/shared/hndpmu.o \
         src/shared/linux_osl.o \
         src/shared/sbutils.o \
         src/shared/siutils.o \
         src/bcmsdio/sys/bcmsdh_sdmmc.o \
         src/bcmsdio/sys/bcmsdh.o \
         src/bcmsdio/sys/bcmsdh_linux.o \
         src/bcmsdio/sys/bcmsdh_sdmmc_linux.o \
         src/wl/sys/wl_iw.o
all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules

clean: 
	rm -rf *.o *.ko *.mod.c *~ .*.cmd Module.symvers modules.order .tmp_versions	\
	 	 src/dhd/sys/dhd_linux.o \
         src/dhd/sys/dhd_common*.o \
         src/dhd/sys/dhd_cdc.o \
         src/dhd/sys/dhd_linux_sched.o\
         src/dhd/sys/dhd_sdio.o \
         src/dhd/sys/dhd_custom_gpio.o \
         src/shared/aiutils.o \
         src/shared/bcmutils.o \
         src/shared/bcmwifi.o \
         src/shared/hndpmu.o \
         src/shared/linux_osl.o \
         src/shared/sbutils.o \
         src/shared/siutils.o \
         src/bcmsdio/sys/bcmsdh_sdmmc.o \
         src/bcmsdio/sys/bcmsdh.o \
         src/bcmsdio/sys/bcmsdh_linux.o \
         src/bcmsdio/sys/bcmsdh_sdmmc_linux.o \
         src/wl/sys/wl_iw*.o \
         src/dhd/sys/.dhd_linux.o.cmd \
         src/dhd/sys/.dhd_common*.o.cmd \
         src/dhd/sys/.dhd_cdc.o.cmd \
         src/dhd/sys/.dhd_linux_sched.o.cmd \
         src/dhd/sys/.dhd_sdio.o.cmd \
         src/dhd/sys/.dhd_custom_gpio.o.cmd \
         src/shared/.aiutils.o.cmd \
         src/shared/.bcmutils.o.cmd \
         src/shared/.bcmwifi.o.cmd \
         src/shared/.hndpmu.o.cmd \
         src/shared/.linux_osl.o.cmd \
         src/shared/.sbutils.o.cmd \
         src/shared/.siutils.o.cmd \
         src/bcmsdio/sys/.bcmsdh_sdmmc.o.cmd \
         src/bcmsdio/sys/.bcmsdh.o.cmd \
         src/bcmsdio/sys/.bcmsdh_linux.o.cmd \
         src/bcmsdio/sys/.bcmsdh_sdmmc_linux.o.cmd \
         src/wl/sys/.wl_iw*.o.cmd

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install
		
else

# bcm4329
DHDCFLAGS = -DLINUX -DBCMDRIVER -DBCMDONGLEHOST -DDHDTHREAD -DBCMWPA2         \
	-DUNRELEASEDCHIP -Dlinux -DDHD_SDALIGN=64 -DMAX_HDR_READ=64           \
	-DDHD_FIRSTREAD=64 -DDHD_GPL -DDHD_SCHED -DBDC -DTOE -DDHD_BCMEVENTS  \
	-DSHOW_EVENTS -DBCMSDIO -DDHD_GPL -DBCMLXSDMMC -DBCMPLATFORM_BUS      \
	-Wall -Wstrict-prototypes -Werror -DOOB_INTR_ONLY -DCUSTOMER_HW2      \
	-DDHD_USE_STATIC_BUF -DMMC_SDIO_ABORT -DDHD_DEBUG_TRAP -DSOFTAP       \
	-DEMBEDDED_PLATFORM -DARP_OFFLOAD_SUPPORT -DPKT_FILTER_SUPPORT        \
	-DGET_CUSTOM_MAC_ENABLE -DSET_RANDOM_MAC_SOFTAP -DCSCAN -DHW_OOB      \
	-Idrivers/net/wireless/bcm4329 -Idrivers/net/wireless/bcm4329/include

DHDOFILES = dhd_linux.o linux_osl.o bcmutils.o dhd_common.o dhd_custom_gpio.o \
	wl_iw.o siutils.o sbutils.o aiutils.o hndpmu.o bcmwifi.o dhd_sdio.o   \
	dhd_linux_sched.o dhd_cdc.o bcmsdh_sdmmc.o bcmsdh.o bcmsdh_linux.o    \
	bcmsdh_sdmmc_linux.o

obj-$(CONFIG_BCM4329) += bcm4329.o
bcm4329-objs += $(DHDOFILES)
EXTRA_CFLAGS = $(DHDCFLAGS)
EXTRA_LDFLAGS += --strip-debug

endif
