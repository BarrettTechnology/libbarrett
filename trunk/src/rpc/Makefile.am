# The libtool library to make
noinst_LTLIBRARIES = libbtrpc.la

# Header files to include in the package
pkginclude_HEADERS = rpc.h rpc_tcpjson.h

# Source files to complile
#AM_CPPFLAGS = -I../spline -I../profile -I../log
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror
libbtrpc_la_SOURCES = rpc.c rpc_tcpjson.c
