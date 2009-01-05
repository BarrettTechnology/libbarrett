# The libtool library to make
noinst_LTLIBRARIES = libbtrefgen.la

# Header files to include in the package
pkginclude_HEADERS = refgen.h refgen_move.h refgen_teachplay_const.h

# Source files to complile
AM_CPPFLAGS = -I../spline -I../profile -I../log
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror
libbtrefgen_la_SOURCES = refgen_move.c refgen_teachplay_const.c
