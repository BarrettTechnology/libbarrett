# The libtool library to make
noinst_LTLIBRARIES = libbtcontrol.la

# Header files to include in the package
pkginclude_HEADERS = control.h control_joint_legacy.h control_joint.h

# Source files to complile
AM_CPPFLAGS = -I../gsl -I../trajectory -I../dynamics
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
libbtcontrol_la_SOURCES = control_joint_legacy.c control_joint.c
