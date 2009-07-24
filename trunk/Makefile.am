# Things set by ./configure:
# Variables:
#  - @BT_OS_CPP@
#  - @BT_OS_LDFLAGS@
#  - @BT_OS_LIBS@
#  - @BT_BUS_CANTYPE@
#  - @BT_BUS_LIBS@
# Contitionals:
#  - BT_ASYNC_ONLY
#  - BT_WAMBOT_PHYSICAL
#  - BT_WAMBOT_SIMULATED
#  - BT_BUS_ENABLE



####### LIBBARRETT

# The library to make 
lib_LTLIBRARIES = src/libbarrett.la

# Library dependencies
if BT_WAM_LOCAL
libbarrett_lib_deps = -lgsl -lgslcblas -lconfig
else
libbarrett_lib_deps =
endif

# All the libraries to not be installed ...
src_libbarrett_la_SOURCES = 
src_libbarrett_la_LIBADD = $(libbarrett_lib_deps)

noinst_LTLIBRARIES =
pkginclude_HEADERS =

############################################

if BT_WAM_LOCAL

######## BT_BUS

if BT_BUS_ENABLE

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtbus.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtbus.la

# Header files to include in the package
pkginclude_HEADERS += src/bus/bus.h \
                      src/bus/bus_can.h

# Source files to complile
src_libbtbus_la_CPPFLAGS = -Isrc/os -D@BT_BUS_CANTYPE@
src_libbtbus_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtbus_la_SOURCES = src/bus/bus.c \
                          src/bus/bus_can.c
src_libbtbus_la_LIBADD = @BT_BUS_LIBS@

endif

####### BT_CONTROL

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtcontrol.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtcontrol.la

# Header files to include in the package
pkginclude_HEADERS += src/control/control.h \
                      src/control/control_joint_legacy.h \
                      src/control/control_joint.h \
                      src/control/control_cartesian_xyz.h \
                      src/control/control_cartesian_xyz_q.h

# Source files to complile
src_libbtcontrol_la_CPPFLAGS = -Isrc/gsl \
                               -Isrc/trajectory \
                               -Isrc/dynamics \
                               -Isrc/kinematics
src_libbtcontrol_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtcontrol_la_SOURCES = src/control/control_joint_legacy.c \
                              src/control/control_joint.c \
                              src/control/control_cartesian_xyz.c \
                              src/control/control_cartesian_xyz_q.c

####### BT_DYNAMICS

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtdynamics.la

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

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtgravity.la

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

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtgsl.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtgsl.la

# Header files to include in the package
pkginclude_HEADERS += src/gsl/gsl.h

# Source files to complile
src_libbtgsl_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtgsl_la_SOURCES = src/gsl/gsl.c

####### BT_INTERP

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtinterp.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtinterp.la

# Header files to include in the package
pkginclude_HEADERS += src/interp/interp.h

# Source files to complile
src_libbtinterp_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtinterp_la_SOURCES = src/interp/interp.c

####### BT_KINEMATICS

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtkinematics.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtkinematics.la

# Header files to include in the package
pkginclude_HEADERS += src/kinematics/kinematics.h

# Source files to complile
src_libbtkinematics_la_CPPFLAGS = -Isrc/gsl
src_libbtkinematics_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtkinematics_la_SOURCES = src/kinematics/kinematics.c

####### BT_LOG

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtlog.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtlog.la

# Header files to include in the package
pkginclude_HEADERS += src/log/log.h

# Source files to complile
src_libbtlog_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtlog_la_SOURCES = src/log/log.c

####### BT_OS

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtos.la

noinst_LTLIBRARIES += src/libbtos.la

pkginclude_HEADERS += src/os/os.h

src_libbtos_la_SOURCES = src/os/os.c

src_libbtos_la_CFLAGS = -Wall -Werror
src_libbtos_la_CPPFLAGS = @BT_OS_CPP@
src_libbtos_la_LDFLAGS = @BT_OS_LDFLAGS@
src_libbtos_la_LIBADD = @BT_OS_LIBS@

####### BT_PROFILE

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtprofile.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtprofile.la

# Header files to include in the package
pkginclude_HEADERS += src/profile/profile.h

# Source files to complile
src_libbtprofile_la_CPPFLAGS = -Isrc/interp
src_libbtprofile_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror
src_libbtprofile_la_SOURCES = src/profile/profile.c

####### BT_REFGEN

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtrefgen.la

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

####### BT_SPLINE

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtspline.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtspline.la

# Header files to include in the package
pkginclude_HEADERS += src/spline/spline.h

# Source files to complile
src_libbtspline_la_CPPFLAGS = -Isrc/interp
src_libbtspline_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror
src_libbtspline_la_SOURCES = src/spline/spline.c

####### BT_WAMBOT

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtwambot.la

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
src_libbtwambot_la_SOURCES = $(SOURCE_PHYS) \
                             $(SOURCE_SIM)

#######

endif
                             
############################################

####### BT_DISCOVER

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtdiscover.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtdiscover.la

# Header files to include in the package
pkginclude_HEADERS += src/discover/discover.h

# Source files to complile
src_libbtdiscover_la_CPPFLAGS =
src_libbtdiscover_la_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtdiscover_la_SOURCES = src/discover/discover.c

####### BT_FILE

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtfile.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtfile.la

# Header files to include in the package
pkginclude_HEADERS += src/file/file.h \
                      src/file/file_rpc.h

# Source files to complile
src_libbtfile_la_CPPFLAGS = -Isrc/rpc
# Damn variadic macros!
src_libbtfile_la_CFLAGS = -std=c99 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtfile_la_SOURCES = src/file/file.c \
                           src/file/file_rpc.c

if BT_WAM_LOCAL
else

src_libbtfile_la_CPPFLAGS += -DASYNC_ONLY

endif

####### BT_RPC

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtrpc.la

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
src_libbtrpc_la_LIBADD = -ljson

####### BT_WAM

# Add this library to libbt
src_libbarrett_la_LIBADD += src/libbtwam.la

# The libtool library to make
noinst_LTLIBRARIES += src/libbtwam.la

# Header files to include in the package
pkginclude_HEADERS += src/wam/wam.h \
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
# Note: c99 doesn't include popen (!!)
#src_libbtwam_la_CFLAGS = -std=c99 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtwam_la_CFLAGS = -std=gnu99 -pedantic-errors -Wall -Werror -Wno-long-long
src_libbtwam_la_SOURCES = src/wam/wam.c \
                          src/wam/wam_rpc.c

if BT_WAM_LOCAL

pkginclude_HEADERS += src/wam/wam_custom.h \
                      src/wam/wam_local.h

src_libbtwam_la_SOURCES += src/wam/wam_local.c

else

src_libbtwam_la_CPPFLAGS += -DASYNC_ONLY

endif

##############################################

bin_PROGRAMS = programs/bt-wam-demo/bt-wam-demo \
               programs/bt-wam-gw/bt-wam-gw \
               programs/bt-wam-list/bt-wam-list

# It sucks that I have to include all of this crap ...
programs_bt_wam_calibrate_bt_wam_calibrate_SOURCES = programs/bt-wam-calibrate/bt-wam-calibrate.c
programs_bt_wam_calibrate_bt_wam_calibrate_CPPFLAGS = -Isrc/gsl \
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
                                                      
programs_bt_wam_demo_bt_wam_demo_SOURCES = programs/bt-wam-demo/bt-wam-demo.c
programs_bt_wam_demo_bt_wam_demo_CPPFLAGS = -Isrc/wam
programs_bt_wam_demo_bt_wam_demo_CFLAGS = -std=gnu89 -pedantic-errors -Wall -Werror -Wno-long-long
programs_bt_wam_demo_bt_wam_demo_LDADD = src/libbarrett.la -lcurses $(libbarrett_lib_deps)

programs_bt_wam_gw_bt_wam_gw_SOURCES = programs/bt-wam-gw/bt-wam-gw.c
programs_bt_wam_gw_bt_wam_gw_CPPFLAGS = -Isrc/rpc -Isrc/wam -Isrc/file
programs_bt_wam_gw_bt_wam_gw_CFLAGS = -std=c99 -pedantic-errors -Wall -Werror -Wno-long-long
programs_bt_wam_gw_bt_wam_gw_LDADD = src/libbarrett.la $(libbarrett_lib_deps)

programs_bt_wam_list_bt_wam_list_SOURCES = programs/bt-wam-list/bt-wam-list.c
programs_bt_wam_list_bt_wam_list_CPPFLAGS = -Isrc/wam
programs_bt_wam_list_bt_wam_list_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
programs_bt_wam_list_bt_wam_list_LDADD = src/libbarrett.la $(libbarrett_lib_deps)

dist_bin_SCRIPTS = programs/bt-wam-py-demo/bt-wam-py-demo

###########

if BT_WAM_LOCAL

bin_PROGRAMS += programs/bt-wam-calibrate/bt-wam-calibrate \
                programs/bt-bus-util/bt-bus-util

# Damn variadic macros!
programs_bt_wam_calibrate_bt_wam_calibrate_CFLAGS = -std=c99 -pedantic-errors -Wall -Werror -Wno-long-long
programs_bt_wam_calibrate_bt_wam_calibrate_LDADD = src/libbarrett.la -lcurses $(libbarrett_lib_deps)

programs_bt_bus_util_bt_bus_util_SOURCES = programs/bt-bus-util/bt-bus-util.c
programs_bt_bus_util_bt_bus_util_CPPFLAGS = -Isrc/bus -Isrc/os
programs_bt_bus_util_bt_bus_util_CFLAGS = -std=c89 -pedantic-errors -Wall -Werror -Wno-long-long
programs_bt_bus_util_bt_bus_util_LDADD = src/libbarrett.la -lcurses $(libbarrett_lib_deps)

endif

##############################################

# Some more rules
include pkg/debian.mk

##############################################

# Also, make the bindings
SUBDIRS = . bindings/python
