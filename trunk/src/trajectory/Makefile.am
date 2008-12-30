# The libtool library to make
noinst_LTLIBRARIES = libbttrajectory.la

# Header files to include in the package
pkginclude_HEADERS = trajectory.h trajectory_move.h trajectory_teachplay.h

# Source files to complile
AM_CPPFLAGS = -I../spline -I../profile -I../log
AM_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror
libbttrajectory_la_SOURCES = trajectory_move.c trajectory_teachplay.c
