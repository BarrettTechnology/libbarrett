# The libtool library to make
noinst_LTLIBRARIES = libbtpath.la

# Header files to include in the package
pkginclude_HEADERS = path.h path_discrete.h

# Source files to complile
AM_CPPFLAGS = -I../gsl -I../trajectory
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
libbtpath_la_SOURCES = path.c path_discrete.c
