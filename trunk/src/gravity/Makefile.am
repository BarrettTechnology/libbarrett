# The libtool library to make
noinst_LTLIBRARIES = libbtgravity.la

# Header files to include in the package
pkginclude_HEADERS = gravity.h

# Source files to complile
AM_CPPFLAGS = -I../os -I../sys -I../bot -I../kinematics -I../gsl
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
libbtgravity_la_SOURCES = gravity.c
