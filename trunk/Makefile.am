# Let's try something different.

####### LIBBT

# The library to make 
lib_LTLIBRARIES = src/libbt.la

# Any source files?
src_libbt_la_SOURCES =

if BT_BUS_ENABLE
BT_BUS_LIB = src/libbtbus.la
endif

src_libbt_la_LIBADD = \
  $(BT_BUS_LIB) \
  src/libbtcontrol.la \
  src/libbtdynamics.la \
  src/libbtgravity.la \
  src/libbtgsl.la \
  src/libbtinterp.la \
  src/libbtkinematics.la \
  src/libbtlog.la \
  src/libbtos.la \
  src/libbtprofile.la \
  src/libbtrefgen.la \
  src/libbtrpc.la \
  src/libbtspline.la \
  src/libbtwam.la \
  src/libbtwambot.la

# All the libraries to not be installed ...
noinst_LTLIBRARIES =
pkginclude_HEADERS =

######## BT_BUS

if BT_BUS_ENABLE

# The libtool library to make
noinst_LTLIBRARIES += src/libbtbus.la

# Header files to include in the package
pkginclude_HEADERS += src/bus/bus.h

# Source files to complile
src_libbtbus_la_CPPFLAGS = -Isrc/os -D@BT_BUS_CANTYPE@
src_libbtbus_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtbus_la_SOURCES = src/bus/bus.c \
                          src/bus/can_peakesd.c \
                          src/bus/can.h
src_libbtbus_la_LIBADD = @BT_BUS_LIBS@

endif

####### BT_CONTROL

# The libtool library to make
noinst_LTLIBRARIES += src/libbtcontrol.la

# Header files to include in the package
pkginclude_HEADERS += src/control/control.h \
                      src/control/control_joint_legacy.h \
                      src/control/control_joint.h \
                      src/control/control_cartesian_xyz.h

# Source files to complile
src_libbtcontrol_la_CPPFLAGS = -Isrc/gsl \
                               -Isrc/trajectory \
                               -Isrc/dynamics \
                               -Isrc/kinematics
src_libbtcontrol_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtcontrol_la_SOURCES = src/control/control_joint_legacy.c \
                              src/control/control_joint.c \
                              src/control/control_cartesian_xyz.c

####### BT_DYNAMICS

# The libtool library to make
noinst_LTLIBRARIES += src/libbtdynamics.la

# Header files to include in the package
pkginclude_HEADERS += src/dynamics/dynamics.h

# Source files to complile
src_libbtdynamics_la_CPPFLAGS = -Isrc/gsl \
                                -Isrc/kinematics
src_libbtdynamics_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtdynamics_la_SOURCES = src/dynamics/dynamics.c

####### BT_GRAVITY

# The libtool library to make
noinst_LTLIBRARIES += src/libbtgravity.la

# Header files to include in the package
pkginclude_HEADERS += src/gravity/gravity.h

# Source files to complile
src_libbtgravity_la_CPPFLAGS = -Isrc/os \
                               -Isrc/sys \
                               -Isrc/bot \
                               -Isrc/kinematics \
                               -Isrc/gsl
src_libbtgravity_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtgravity_la_SOURCES = src/gravity/gravity.c

####### BT_GSL

# The libtool library to make
noinst_LTLIBRARIES += src/libbtgsl.la

# Header files to include in the package
pkginclude_HEADERS += src/gsl/gsl.h

# Source files to complile
src_libbtgsl_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtgsl_la_SOURCES = src/gsl/gsl.c

####### BT_INTERP

# The libtool library to make
noinst_LTLIBRARIES += src/libbtinterp.la

# Header files to include in the package
pkginclude_HEADERS += src/interp/interp.h

# Source files to complile
src_libbtinterp_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtinterp_la_SOURCES = src/interp/interp.c

####### BT_KINEMATICS

# The libtool library to make
noinst_LTLIBRARIES += src/libbtkinematics.la

# Header files to include in the package
pkginclude_HEADERS += src/kinematics/kinematics.h

# Source files to complile
src_libbtkinematics_la_CPPFLAGS = -Isrc/gsl
src_libbtkinematics_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtkinematics_la_SOURCES = src/kinematics/kinematics.c

####### BT_LOG

# The libtool library to make
noinst_LTLIBRARIES += src/libbtlog.la

# Header files to include in the package
pkginclude_HEADERS += src/log/log.h

# Source files to complile
src_libbtlog_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtlog_la_SOURCES = src/log/log.c

####### BT_OS

noinst_LTLIBRARIES += src/libbtos.la

pkginclude_HEADERS += src/os/os.h

src_libbtos_la_SOURCES = src/os/os.c

src_libbtos_la_CFLAGS = -Wall -Werror
src_libbtos_la_CPPFLAGS = @BT_OS_CPP@
src_libbtos_la_LDFLAGS = @BT_OS_LDFLAGS@
src_libbtos_la_LIBADD = @BT_OS_LIBS@

####### BT_PROFILE

# The libtool library to make
noinst_LTLIBRARIES += src/libbtprofile.la

# Header files to include in the package
pkginclude_HEADERS += src/profile/profile.h

# Source files to complile
src_libbtprofile_la_CPPFLAGS = -Isrc/interp
src_libbtprofile_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror
src_libbtprofile_la_SOURCES = src/profile/profile.c

####### BT_REFGEN

# The libtool library to make
noinst_LTLIBRARIES += src/libbtrefgen.la

# Header files to include in the package
pkginclude_HEADERS += src/refgen/refgen.h \
                      src/refgen/refgen_move.h \
                      src/refgen/refgen_teachplay_const.h \
                      src/refgen/refgen_teachplay.h

# Source files to complile
src_libbtrefgen_la_CPPFLAGS = -Isrc/spline \
                              -Isrc/profile \
                              -Isrc/log
src_libbtrefgen_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror
src_libbtrefgen_la_SOURCES = src/refgen/refgen_move.c \
                             src/refgen/refgen_teachplay_const.c \
                             src/refgen/refgen_teachplay.c

####### BT_RPC

# The libtool library to make
noinst_LTLIBRARIES += src/libbtrpc.la

# Header files to include in the package
pkginclude_HEADERS += src/rpc/rpc.h \
                      src/rpc/rpc_tcpjson.h

# Source files to complile
# Note: -Wno-variadic-macros can be used in GCC 4.2.4, but not GCC 3.4.6. UGH!
#src_libbtrpc_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-variadic-macros
src_libbtrpc_la_CFLAGS = -std=c99 -pedantic-errors -Wall -Werror
src_libbtrpc_la_SOURCES = src/rpc/rpc.c \
                          src/rpc/rpc_tcpjson.c

####### BT_SPLINE

# The libtool library to make
noinst_LTLIBRARIES += src/libbtspline.la

# Header files to include in the package
pkginclude_HEADERS += src/spline/spline.h

# Source files to complile
src_libbtspline_la_CPPFLAGS = -Isrc/interp
src_libbtspline_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror
src_libbtspline_la_SOURCES = src/spline/spline.c

####### BT_WAM

# The libtool library to make
noinst_LTLIBRARIES += src/libbtwam.la

# Header files to include in the package
pkginclude_HEADERS += src/wam/wam.h \
                      src/wam/wam_custom.h \
                      src/wam/wam_internal.h \
                      src/wam/wam_rpc.h

# Source files to complile
# Note - remove os.h dep from sys.h ...
src_libbtwam_la_CPPFLAGS = -Isrc/gsl \
                           -Isrc/os \
                           -Isrc/bus \
                           -Isrc/wambot \
                           -Isrc/kinematics \
                           -Isrc/dynamics \
                           -Isrc/gravity \
                           -Isrc/refgen \
                           -Isrc/log \
                           -Isrc/control \
                           -Isrc/spline \
                           -Isrc/profile \
                           -Isrc/rpc
# Note: -Wno-variadic-macros can be used in GCC 4.2.4, but not GCC 3.4.6. UGH!
#src_libbtwam_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long -Wno-variadic-macros
src_libbtwam_la_CFLAGS = -std=c99 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtwam_la_SOURCES = src/wam/wam.c \
                          src/wam/wam_rpc.c

####### BT_WAMBOT

# The libtool library to make
noinst_LTLIBRARIES += src/libbtwambot.la

# What to compile?
if BT_WAMBOT_PHYSICAL
CPP_PHYS = -Isrc/bus -Isrc/os -Isrc/gsl
HEADER_PHYS = src/wambot/wambot_phys.h
SOURCE_PHYS = src/wambot/wambot_phys.c
endif

if BT_WAMBOT_SIMULATED
CPP_SIM = -Isrc/os
HEADER_SIM = src/wambot/wambot_sim.h
SOURCE_SIM = src/wambot/wambot_sim.c
endif

# Header files to include in the package
pkginclude_HEADERS += src/wambot/wambot.h \
                      $(HEADER_PHYS) \
                      $(HEADER_SIM)

# Source files to complile
# Note - remove os.h dep from bus.h ...
src_libbtwambot_la_CPPFLAGS = $(CPP_PHYS) $(CPP_SIM)
src_libbtwambot_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtwambot_la_SOURCES = src/wambot/wambot.c \
                             $(SOURCE_PHYS) \
                             $(SOURCE_SIM)

##############################################

# Also, make the legacy stuff

lib_LTLIBRARIES += legacy/libbtlegacy.la

pkginclude_HEADERS += legacy/wam/wam_legacy.h

legacy_libbtlegacy_la_CPPFLAGS = -Isrc/gsl \
                                 -Isrc/os \
                                 -Isrc/bus \
                                 -Isrc/wambot \
                                 -Isrc/kinematics \
                                 -Isrc/dynamics \
                                 -Isrc/gravity \
                                 -Isrc/refgen \
                                 -Isrc/log \
                                 -Isrc/control \
                                 -Isrc/spline \
                                 -Isrc/profile \
                                 -Isrc/rpc \
                                 -Isrc/wam
# Damn variadic macros ...
legacy_libbtlegacy_la_CFLAGS = -std=c99 -pedantic-errors -Wall -Werror -Wno-long-long
legacy_libbtlegacy_la_SOURCES = legacy/wam/wam_legacy.c

##############################################

# Make the libwam library itself
SUBDIRS = . bindings/python
