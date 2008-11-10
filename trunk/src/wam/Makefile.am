# The libtool library to make
noinst_LTLIBRARIES = libbtwam.la

# Header files to include in the package
pkginclude_HEADERS = wam.h

# Source files to complile
# Note - remove os.h dep from sys.h ...
AM_CPPFLAGS = -I../os -I../bus -I../wambot -I../kinematics -I../gravity -I../trajectory -I../log -I../control -I../spline -I../profile
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
libbtwam_la_SOURCES = wam.c
