#****************************************************************************
# Copyright (C) 2001-2006  PEAK System-Technik GmbH
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
# Makefile - Makefile for receivetest and transmittest programs
#
# $Id: Makefile 518 2007-08-08 07:40:31Z edouard $
#
#****************************************************************************

SRC     = src
INC     = -I. -I../lib -I../driver
RT      = NO_RT


ifeq ($(RT), XENOMAI)
SKIN = xeno
### Xenomai directory, xeno-config and library directory ###########
RT_DIR          ?= /usr/xenomai
RT_CONFIG       ?= $(RT_DIR)/bin/xeno-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir) -Wl,-rpath $(shell $(RT_CONFIG) --library-dir)
### User space application compile options #########################
USERAPP_LIBS      ?= -lnative -lpcan
USERAPP_LDFLAGS   ?= $(shell $(RT_CONFIG) --$(SKIN)-ldflags) -L$(RT_LIB_DIR)
USERAPP_CFLAGS    ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

ifeq ($(RT), RTAI)
SKIN = lxrt
### Rtai directory, rtai-config and library directory ###########
RT_DIR          ?= /usr/realtime
RT_CONFIG       ?= $(RT_DIR)/bin/rtai-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir) -Wl,-rpath $(shell $(RT_CONFIG) --library-dir)
### User space application compile options #########################
USERAPP_LIBS      ?= -llxrt -lpcan
USERAPP_LDFLAGS   ?= $(shell $(RT_CONFIG) --$(SKIN)-ldflags) -L$(RT_LIB_DIR)
USERAPP_CFLAGS    ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

ifeq ($(RT), NO_RT)
  USERAPP_LIBS = -lpcan
endif

ifeq ($(HOSTTYPE),x86_64)
  LDLIBS  = -L../lib -L/lib64 -L/usr/lib64 -L/usr/local/lib64
else
  LDLIBS  = -L../lib -L/lib -L/usr/lib -L/usr/local/lib
endif

ifneq ($(RT), NO_RT)
DBGFLAGS   =
else
DBGFLAGS   = -g
endif

ifeq ($(DBG), DEBUG)
CFLAGS  = $(DBGFLAGS) $(INC) $(LDLIBS)
else
CFLAGS  = $(INC) $(LDLIBS)
endif


TARGET1 = receivetest
FILES1  = $(SRC)/$(TARGET1).c $(SRC)/common.c

TARGET2 = transmitest
FILES2  = $(SRC)/$(TARGET2).cpp $(SRC)/common.c $(SRC)/parser.cpp

TARGET3 = bitratetest
FILES3  = $(SRC)/$(TARGET3).c $(SRC)/common.c

TARGET4 = filtertest
FILES4  = $(SRC)/$(TARGET4).cpp $(SRC)/common.c

all:    $(TARGET1) $(TARGET2) $(TARGET3) $(TARGET4)

$(TARGET1): $(FILES1)
	$(CC) $(FILES1) $(CFLAGS) -o $(TARGET1) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)

$(TARGET2): $(FILES2)
	$(CC) $(FILES2) $(CFLAGS) -o $(TARGET2) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -lstdc++ -D$(RT)
	
$(TARGET3): $(FILES3)
	$(CC) $(FILES3) $(CFLAGS) -o $(TARGET3) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -D$(RT)

$(TARGET4): $(FILES4)
	$(CC) $(FILES4) $(CFLAGS) -o $(TARGET4) $(USERAPP_CFLAGS) $(USERAPP_LDFLAGS) $(USERAPP_LIBS) -lstdc++ -D$(RT)
	

clean:
	rm -f $(SRC)/*~ $(SRC)/*.o *~ $(TARGET1) $(TARGET2) $(TARGET3) $(TARGET4)
	
install:
	cp $(TARGET1) /usr/local/bin
	cp $(TARGET2) /usr/local/bin
	cp $(TARGET3) /usr/local/bin
	cp $(TARGET4) /usr/local/bin



