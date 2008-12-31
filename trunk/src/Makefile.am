# The library to make 
lib_LTLIBRARIES = libbt.la

# Any source files?
libbt_la_SOURCES =

if BT_BUS_ENABLE
BT_BUS_LIB = bus/libbtbus.la
endif

# List extra libtool objects (.lo) or libtool libraries (.la) to add to the library
libbt_la_LIBADD = \
  os/libbtos.la \
  $(BT_BUS_LIB) \
  wambot/libbtwambot.la \
  kinematics/libbtkinematics.la \
  gravity/libbtgravity.la \
  interp/libbtinterp.la \
  refgen/libbtrefgen.la \
  log/libbtlog.la \
  wam/libbtwam.la \
  gsl/libbtgsl.la \
  control/libbtcontrol.la \
  spline/libbtspline.la \
  profile/libbtprofile.la

# Subdirectories to make
SUBDIRS = os bus wambot kinematics gravity interp refgen log wam gsl control spline profile

