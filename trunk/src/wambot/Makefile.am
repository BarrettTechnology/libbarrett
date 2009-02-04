# The libtool library to make
noinst_LTLIBRARIES = libbtwambot.la

# What to compile?
if BT_WAMBOT_PHYSICAL
CPP_PHYS = -I../bus -I../os -I../gsl
HEADER_PHYS = wambot_phys.h
SOURCE_PHYS = wambot_phys.c
endif

if BT_WAMBOT_SIMULATED
CPP_SIM = -I../os
HEADER_SIM = wambot_sim.h
SOURCE_SIM = wambot_sim.c
endif

# Header files to include in the package
pkginclude_HEADERS = wambot.h $(HEADER_PHYS) $(HEADER_SIM)

# Source files to complile
# Note - remove os.h dep from bus.h ...
AM_CPPFLAGS = $(CPP_PHYS) $(CPP_SIM)
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
libbtwambot_la_SOURCES = wambot.c $(SOURCE_PHYS) $(SOURCE_SIM)
