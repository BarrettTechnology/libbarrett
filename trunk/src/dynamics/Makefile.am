# The libtool library to make
noinst_LTLIBRARIES = libbtdynamics.la

# Header files to include in the package
pkginclude_HEADERS = dynamics.h

# Source files to complile
AM_CPPFLAGS = -I../gsl -I../kinematics
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
libbtdynamics_la_SOURCES = dynamics.c
