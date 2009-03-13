# The libtool library to make
noinst_LTLIBRARIES = libbtrpc.la

# Header files to include in the package
pkginclude_HEADERS = rpc.h rpc_tcpjson.h

# Source files to complile
#AM_CPPFLAGS = -I../spline -I../profile -I../log
# Note: -Wno-variadic-macros can be used in GCC 4.2.4, but not GCC 3.4.6. UGH!
#AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-variadic-macros
AM_CFLAGS = -std=c99 -pedantic-errors -Wall -Werror
libbtrpc_la_SOURCES = rpc.c rpc_tcpjson.c
