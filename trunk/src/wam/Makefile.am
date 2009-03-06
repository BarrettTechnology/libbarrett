# The libtool library to make
noinst_LTLIBRARIES = libbtwam.la

# Header files to include in the package
pkginclude_HEADERS = wam.h wam_custom.h wam_internal.h wam_gw_server.h

# Source files to complile
# Note - remove os.h dep from sys.h ...
AM_CPPFLAGS = -I../gsl -I../os -I../bus -I../wambot -I../kinematics -I../dynamics -I../gravity -I../refgen -I../log -I../control -I../spline -I../profile
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
libbtwam_la_SOURCES = wam.c wam_gw_server.c
