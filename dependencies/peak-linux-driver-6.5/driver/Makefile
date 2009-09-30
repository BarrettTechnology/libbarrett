#****************************************************************************
# Copyright (C) 2001-2007  PEAK System-Technik GmbH
#
# linux@peak-system.com
# www.peak-system.com
#
# Maintainer(s): Klaus Hitschler (klaus.hitschler@gmx.de)
# Contributions: Grant Edwards (gedwards@rivatek.com)
#****************************************************************************

#****************************************************************************
#
# Makefile for Peak CAN interface drivers (ISA, PC/104, PCI, DONGLE, USB, PCCARD)
#
# $Id: Makefile 517 2007-07-09 09:40:42Z edouard $
#
#****************************************************************************

#****************************************************************************
# targets:
# depend		creates dependencies from included header files - not kernel 2.6
# all			does a standard make of the driver
# clean			cleans up, base for a forced new make
# mrproper		make a total clean - not kernel 2.6
# install		run as root, installs the drivers
# message		show info about build environment
#
# make options:
# DBG=DEBUG                 or NO_DEBUG    		  switch debug on and switches Makefile verbosity on
# KERNEL_LOCATION=...       where your kernel sources are located if not standard location (cross-compile)
# MOD=NO_MODVERSIONS		or MODVERSIONS 		  compile for modversions use
# PAR=NO_PARPORT_SUBSYSTEM	or PARPORT_SUBSYSTEM  compile for use without parport subsystem und pcan-dongle
# USB=NO_USB_SUPPORT 		or USB_SUPPORT    	  compile for use with USB and pcan-usb
# PCI=NO_PCI_SUPPORT        or PCI_SUPPORT        compile for use with PCAN-PCI
# DNG=NO_DONGLE_SUPPORT     or DONGLE_SUPPORT     compile for use with PCAN-DONGLE
# ISA=NO_ISA_SUPPORT        or ISA_SUPPORT        compile for use with PCAN-ISA or PCAN-104
# PCC=NO_PCCARD_SUPPORT     or PCCARD_SUPPORT     compile for use with PCAN-PCCARD
# NET=NO_NETDEV_SUPPORT     or NETDEV_SUPPORT     compile for use as CAN network device (AF_CAN)
# VERBOSE=0                 or 1                  switch Makefile verbosity on
# RT=XENOMAI                or RTAI or NO_RT      compile for use on XENOMAI or RTAI RTOS
#
# please the first time do a make clean; make
#

#****************************************************************************
# please modify only here if your kernel path is elsewhere located
#
KERNEL_LOCATION = /usr/src/linux

#****************************************************************************
# removed old kernel source path retrival for support of kernels < 2.2.18!
# old KERNSRC = /lib/modules/$(VERSION)/build
#
KERNSRC := $(KERNEL_LOCATION)

#****************************************************************************
# whole makefile is based on availability of version.h file
# do a forced stop if it is not available
#
HASVERSION_H := $(shell if test -f $(KERNEL_LOCATION)/include/linux/version.h ; then echo yes ; else echo no; fi )

ifeq ($(HASVERSION_H),no)
$(error "Can't find $(KERNEL_LOCATION)/include/linux/version.h !")
else

#****************************************************************************
# removed old version retrival for better cross compile support
# old VERSION := $(shell uname -r)
#
VERSION := $(shell cpp -dM -I$(KERNEL_LOCATION)/include $(KERNEL_LOCATION)/include/linux/version.h \
            | grep UTS_RELEASE | sed -e 's;[^"]*"\(.*\)";\1;g')

ifeq ($(VERSION),)
VERSION := $(shell cpp -dM -I$(KERNEL_LOCATION)/include $(KERNEL_LOCATION)/include/linux/utsrelease.h \
            | grep UTS_RELEASE | sed -e 's;[^"]*"\(.*\)";\1;g')
endif

#****************************************************************************
# where the driver should be installed - change here for cross install
#
INSTALL_LOCATION = /lib/modules/$(VERSION)/misc

#****************************************************************************
# get extracted kernel VERSION and PATCHLEVEL for comparison
# decide to use KBUILD for kernels greater 2.6.0
#
KVERSION    := $(shell echo $(VERSION) | sed -e 's;\([1-9]\)\..*;\1;g' )
KPATCHLEVEL := $(shell echo $(VERSION) | sed -e 's;[1-9]\.\([0-9]\{0,3\}\)\..*;\1;g' )
SUBLEVEL    := $(shell echo $(VERSION) | sed -e 's;[1-9]\.[0-9]\{0,3\}\.\([0-9]\{0,3\}\).*;\1;g' )
USEKBUILD   := $(shell if [ $(KVERSION) -gt 2 ] || [ $(KVERSION) -eq 2 ] && [ $(KPATCHLEVEL) -ge 6 ]  ; \
                 then echo "yes" ; else echo "no"  ; fi)

#****************************************************************************
# some common switches and defines
#
DBG     = NO_DEBUG
MOD     = MODVERSIONS
PAR     = PARPORT_SUBSYSTEM
USB     = USB_SUPPORT
PCI     = PCI_SUPPORT
DNG     = DONGLE_SUPPORT
ISA     = ISA_SUPPORT
PCC     = PCCARD_SUPPORT
NET     = NO_NETDEV_SUPPORT
RT      = NO_RT

VERBOSE = 0

SRC     = src
TEMP    = /tmp
TMPWKD  = $(TEMP)/.pcan-workdir # tricky stuff to get around missing one-time shell command evaluation
IWD     = $(shell cat $(TMPWKD))
PWD     = $(shell pwd)          # called from inside IDEs it differs from the used working directory

PARMS   = type=isa,sp        # only for installer: please modify the install parameters according to your configuration

#****************************************************************************
# switch make verbosity on
#
ifeq ($(DBG),DEBUG)
VERBOSE = 1
endif

ifeq ($(RT), XENOMAI)
#****************************************************************************
# Define flags for XENOMAI installation only
#
USB = NO_USB_SUPPORT
PCC = NO_PCCARD_SUPPORT

SKIN = xeno
RT_DIR          ?= /usr/xenomai
RT_CONFIG       ?= $(RT_DIR)/bin/xeno-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir)
RT_CFLAGS       ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

#****************************************************************************

ifeq ($(RT), RTAI)
#****************************************************************************
# Define flags for RTAI installation only
#
USB = NO_USB_SUPPORT
PCC = NO_PCCARD_SUPPORT

SKIN = lxrt
RT_DIR          ?= /usr/realtime
RT_CONFIG       ?= $(RT_DIR)/bin/rtai-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir)
RT_CFLAGS       ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

#****************************************************************************
# preparation what to build or what to KBUILD
#
pcan-objs := $(SRC)/pcan_main.o $(SRC)/pcan_fops.o $(SRC)/pcan_fifo.o $(SRC)/pcan_filter.o 
pcan-objs += $(SRC)/pcan_parse.o $(SRC)/pcan_sja1000.o $(SRC)/pcan_common.o

ifeq ($(PCI),PCI_SUPPORT)
pcan-objs += $(SRC)/pcan_pci.o
endif

ifeq ($(ISA),ISA_SUPPORT)
pcan-objs += $(SRC)/pcan_isa.o
endif

ifeq ($(DNG),DONGLE_SUPPORT)
pcan-objs += $(SRC)/pcan_dongle.o
endif

ifeq ($(PCC),PCCARD_SUPPORT)
pcan-objs += $(SRC)/pcan_pccard_kernel.o $(SRC)/pcan_pccard.o 
endif

ifeq ($(USB),USB_SUPPORT)
pcan-objs += $(SRC)/pcan_usb_kernel.o $(SRC)/pcan_usb.o 
endif

ifeq ($(NET),NETDEV_SUPPORT)
pcan-objs += $(SRC)/pcan_netdev.o
endif

ifeq ($(USEKBUILD),yes) # <<<<< USEKBUILD >>>>>>

#****************************************************************************
# for local installation only: sometimes /etc/modprobe.conf.local is used
#
MODCONF = /etc/modprobe.conf
MODINST = "install pcan /sbin/modprobe --ignore-install pcan; /usr/local/bin/pcan_make_devices 2"

#****************************************************************************
# what's the target
#
TARGET = pcan.ko
obj-m := pcan.o

#****************************************************************************
# add flags to standard flags
#
CPPFLAGS += -I$(IWD) -D$(DBG) -D$(MOD) -D$(PAR) -D$(USB) -D$(PCI) -D$(ISA) -D$(DNG) -D$(PCC) -D$(NET) -D$(RT) $(RT_CFLAGS)

EXTRA_CFLAGS := $(CPPFLAGS)

#****************************************************************************
# do it in another way for kernels less than 2.6.5
#
USESUBDIRS  := $(shell if [ $(KVERSION) -eq 2 ] && [ $(KPATCHLEVEL) -eq 6 ] && [ $(SUBLEVEL) -lt 5 ]  ; \
                 then echo "yes" ; else echo "no"  ; fi)

ifeq ($(USESUBDIRS),yes)
CMDLINE = $(MAKE) -C $(KERNSRC) SUBDIRS=$(PWD) V=$(VERBOSE) modules
else
CMDLINE = $(MAKE) -C $(KERNSRC) M=$(PWD)       V=$(VERBOSE) modules
endif # <<<<< USESUBDIRS >>>>>>

#****************************************************************************
# do it
#
all : message
	pwd > $(TMPWKD)
	$(CMDLINE)

else # <<<<< USEKBUILD >>>>>>

#****************************************************************************
# additional common switches and defines
#
CC      = gcc
LD      = ld
INC     = $(KERNSRC)/include

DEP     = .depend

#****************************************************************************
# for local installation only: sometimes /etc/modules.conf.local is used
#
MODCONF = /etc/modules.conf
MODINST = "post-install pcan /usr/local/bin/pcan_make_devices 2"

#****************************************************************************
# what's the target
#
TARGET = pcan.o

#****************************************************************************
# compile flags
#
CFLAGS  = -O2 -D__KERNEL__ -DMODULE -Wall -I$(INC) -I. -D$(DBG) -D$(MOD) -D$(PAR) -D$(USB) -D$(PCI) -D$(ISA) -D$(DNG) -D$(PCC) -D$(NET) -D$(RT) $(RT_CFLAGS)

#****************************************************************************
# do it
#
all: message $(TARGET)

$(TARGET)  : $(pcan-objs)
	$(LD) -r $^ -o $@

#********** catch include file depencies ************************************
ifeq ($(DEP),$(wildcard $(DEP)))

depend:
	makedepend -f$(DEP) -- $(CFLAGS) -- $(addsuffix .c, $(basename $(pcan-objs))) -I$(INC)

include $(DEP)

else

depend:
	touch $(DEP)
	makedepend -f$(DEP) -- $(CFLAGS) -- $(addsuffix .c, $(basename $(pcan-objs))) -I$(INC)
endif

endif # <<<<< USEKBUILD >>>>>>

#********** clean all for a rebuild *****************************************
clean:
	rm -f $(SRC)/*o $(SRC)/*~ $(SRC)/.*cmd *o *ko *~ .*cmd  pcan.mod.c

#********** make totally clean **********************************************
ifneq ($(USEKBUILD),yes) # <<<<< USEKBUILD >>>>>>
mrproper: clean
	rm -f $(DEP)
endif # <<<<< USEKBUILD >>>>>>

#********** this entry is reserved for root access only *********************
install:
	@if test -d $(INSTALL_LOCATION) ; then echo "info: $(INSTALL_LOCATION) exists."; else mkdir $(INSTALL_LOCATION); fi
	cp $(TARGET) $(INSTALL_LOCATION)/$(TARGET)
	cp pcan_make_devices /usr/local/bin/pcan_make_devices
	if test -z $(shell grep -l pcan $(MODCONF)); then\
		cp $(MODCONF) $(MODCONF)~;\
		echo ""                                               >> $(MODCONF);\
		echo "# pcan - automatic made entry, begin --------"  >> $(MODCONF);\
		echo "# options pcan $(PARMS)   # see documentation"  >> $(MODCONF);\
		echo $(MODINST)                                       >> $(MODCONF);\
		echo "# pcan - automatic made entry, end ----------"  >> $(MODCONF);\
		echo ""                                               >> $(MODCONF);\
	fi
	/sbin/depmod
	cp pcan.h /usr/include/pcan.h
	chmod 644 /usr/include/pcan.h

#********** informations during build of driver *****************************
.PHONY : message
message:
	@ echo "***"
	@ echo "*** Host machine kernel version=$(shell uname -r), Driver kernel version=$(VERSION), Path to kernel sources=$(KERNSRC), use KBUILD=$(USEKBUILD)"
	@ echo "***"

endif  # <<<<< HASVERSION_H >>>>>>

# DO NOT DELETE

