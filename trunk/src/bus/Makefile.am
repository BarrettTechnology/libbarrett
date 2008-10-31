if BT_BUS_ENABLE

# The libtool library to make
noinst_LTLIBRARIES = libbtbus.la

# Header files to include in the package
pkginclude_HEADERS = bus.h

# Source files to complile
AM_CPPFLAGS = -I../os -D@BT_BUS_CANTYPE@
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
libbtbus_la_SOURCES = bus.c can_peakesd.c can.h
libbtbus_la_LIBADD = @BT_BUS_LIBS@

endif
