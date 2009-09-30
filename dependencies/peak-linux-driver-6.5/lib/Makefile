#****************************************************************************
# Copyright (C) 2001-2007  PEAK System-Technik GmbH
#
# linux@peak-system.com
# www.peak-system.com
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
# Maintainer(s): Klaus Hitschler (klaus.hitschler@gmx.de)
#****************************************************************************

#****************************************************************************
#
# Makefile - Makefile for the shared library libpcan.so.x.x
#
# $Id: Makefile 517 2007-07-09 09:40:42Z edouard $
#
#****************************************************************************

SRC     = src
INC     = -I. -I../driver
FILES   = $(SRC)/libpcan.c
DBG     = -g
RT      = NO_RT

ifeq ($(RT), XENOMAI)
#****************************************************************************
# Define flags for XENOMAI installation only
#
USB = NO_USB_SUPPORT
PCC = NO_PCCARD_SUPPORT

INC     = -I. -I../driver -I/usr/xenomai/include

SKIN = xeno
RT_DIR          ?= /usr/xenomai
RT_CONFIG       ?= $(RT_DIR)/bin/xeno-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir) -Wl,-rpath $(shell $(RT_CONFIG) --library-dir)
RT_CFLAGS       ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

ifeq ($(RT), RTAI)
#****************************************************************************
# Define flags for RTAI installation only
#
USB = NO_USB_SUPPORT
PCC = NO_PCCARD_SUPPORT

INC     = -I. -I../driver -I/usr/realtime/include

SKIN = lxrt
RT_DIR          ?= /usr/realtime
RT_CONFIG       ?= $(RT_DIR)/bin/rtai-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir) -Wl,-rpath $(shell $(RT_CONFIG) --library-dir)
RT_CFLAGS       ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

ifeq ($(HOSTTYPE),x86_64)
  LIBPATH = /usr/lib64
else
  LIBPATH = /usr/lib
endif

LDNAME  = libpcan.so
SONAME  = $(LDNAME).0
TARGET  = $(SONAME).6

ifneq ($(RT), NO_RT)
  CFLAGS = -fPIC -shared -O2 -Wall -Wl,-soname,$(SONAME) -lc $(INC) -D$(RT) $(RT_CFLAGS) -L$(RT_LIB_DIR) -lrtdm
else
  CFLAGS = -fPIC -shared -O2 -Wall -Wl,-soname,$(SONAME) -lc $(INC) -D$(RT)
endif

all:    $(TARGET)

$(TARGET) : $(FILES)
	# ln -sf ../driver/pcan.h pcan.h
	$(CC) $(FILES) $(CFLAGS) -o $(TARGET)
	ln -sf $(TARGET) $(LDNAME)

clean:
	rm -f $(SRC)/*~ $(SRC)/*.o *~ *.so.*

#********** this entry is reserved for root access only *********************
install:
	cp $(TARGET) $(LIBPATH)/$(TARGET)
	ln -sf $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME)
	ln -sf $(LIBPATH)/$(SONAME) $(LIBPATH)/$(LDNAME)
	cp libpcan.h /usr/include/libpcan.h
	chmod 644 /usr/include/libpcan.h
	/sbin/ldconfig
  
  

