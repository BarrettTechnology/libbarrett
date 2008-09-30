# The libtool library to make
noinst_LTLIBRARIES = libbtos.la

# Header files to include in the package
pkginclude_HEADERS = os.h

# Source files to complile
# Note - can't use ansi, pedantic because of xenomai, usleep, etc
libbtos_la_SOURCES = os.c

# External libraries
libbtos_la_CFLAGS = -Wall -Werror
libbtos_la_CPPFLAGS = @BT_OS_CPP@
libbtos_la_LDFLAGS = @BT_OS_LDFLAGS@
libbtos_la_LIBADD = @BT_OS_LIBS@
