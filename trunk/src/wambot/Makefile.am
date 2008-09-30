# The libtool library to make
noinst_LTLIBRARIES = libbtwambot.la

# Header files to include in the package
pkginclude_HEADERS = wambot.h

# Source files to complile
# Note - remove os.h dep from sys.h ...
AM_CPPFLAGS = -I../bus -I../os
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
libbtwambot_la_SOURCES = wambot.c
