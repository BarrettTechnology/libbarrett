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
# Makefile - global Makefile for all components
#
# $Id: Makefile 369 2006-03-30 20:52:10Z khitschler $
#
#****************************************************************************

#****************************************************************************
# MACROS AND DEFINES

define make-all
@cd driver; make depend; make; cd ../lib; make; cd ../test; make; cd ..
endef

define make-clean
@cd driver; make clean; cd ../lib; make clean; cd ../test; make clean; cd ..
endef

define make-install
@cd driver; make install; cd ../lib; make install; cd ../test; make install; cd ..
endef

define make-rpminstall
@cd driver; make rpminstall; cd ../lib; make rpminstall; cd ../test; make rpminstall; cd ..
endef


#****************************************************************************
# DO IT
all :
	$(make-all)

clean:
	$(make-clean)

install:
	$(make-install)
  
# end


# DO NOT DELETE
