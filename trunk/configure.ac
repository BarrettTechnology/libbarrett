AC_INIT([libbt], [0.1], [cd@barrett.com])
AC_CONFIG_HEADERS([config.h])


# Note, these are automake warnings, not compiler ones
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

# Information on the package

### STEP 0: REQUIRED PACKAGES ###
AC_CHECK_HEADER([gsl/gsl_math.h],[],[AC_MSG_ERROR([
   LibBT depends on the GNU Scientific Library (GSL),
   available at http://www.gnu.org/software/gsl/.
])])
AC_CHECK_HEADER([libconfig.h],[],[AC_MSG_ERROR([
   LibBT depends on libconfig,
   available at http://www.hyperrealm.com/libconfig/.
])])

### STEP 1: REAL-TIME OS ###
# Choose a Real-Time operating system (for libbt/os)
AC_ARG_WITH([xenomai],
   [AS_HELP_STRING([--with-xenomai],[Xenomai Real-Time Support])],
   [],[with_xenomai=no])
AC_ARG_WITH([rtai],
   [AS_HELP_STRING([--with-rtai],[RTAI Real-Time Support])],
   [],[with_rtai=no])

# Note - -D_GNU_SOURCE for pthread_mutexattr_settype in pthread.h
#        See: http://ipozgaj.blogspot.com/2006/08/posix-threads-and-manual-pages-rant.html
bt_os_cpp=""
bt_os_ldflags=""
bt_os_libs=""
AS_IF(
   [test "x$with_xenomai" != xno],
   [
      AC_MSG_CHECKING([for Xenomai xeno-config script])
      AS_IF([test "x$with_xenomai" = xyes],[xeno_config="/usr/xenomai/bin/xeno-config"],[xeno_config="$with_xenomai"])
      AC_MSG_RESULT([$xeno_config])
      AC_MSG_CHECKING([for Xenomai version])
      xeno_version=`$xeno_config --version`
      AC_MSG_RESULT([$xeno_version])
      xeno_cppflags=`$xeno_config --xeno-cflags`
      xeno_libs=`$xeno_config --xeno-ldflags`
      # Set up Xenomai
      bt_os_cpp="$bt_os_cpp -DRTSYS_XENOMAI $xeno_cppflags"
      bt_os_ldflags="$bt_os_ldflags -rdynamic"
      bt_os_libs="$bt_os_libs $xeno_libs -lnative"
   ],
   [test "$with_rtai" = yes],
   [
      # Set up RTAI
      bt_os_cpp="$bt_os_cpp -D_GNU_SOURCE -DRTSYS_RTAI -I/usr/realtime/include"
      bt_os_libs="$bt_os_libs -L/usr/realtime/lib -llxrt -lpthread"
   ],
   [
      # No RTSYS Selected
      bt_os_cpp="$bt_os_cpp -D_GNU_SOURCE -DRTSYS_NONE"
      bt_os_libs="-lpthread"
   ]
)
AC_SUBST([BT_OS_CPP],["$bt_os_cpp"])
AC_SUBST([BT_OS_LDFLAGS],["$bt_os_ldflags"])
AC_SUBST([BT_OS_LIBS],["$bt_os_libs"])


### STEP 2: WAMBOT PHYSICAL AND/OR SIMULATION ###
# Choose a physical/simulator option (between libbt/wambot_phys and libbt/wambot_sim)

# This sets the variables:
#   with_wambot_physical
#   with_wambot_simulated
AC_ARG_WITH([wambot_physical],
   [AS_HELP_STRING([--with-wambot-physical],[Physical WAM support (Def:yes)])],
   [],[with_wambot_physical=yes])
AC_ARG_WITH([wambot_simulated],
   [AS_HELP_STRING([--with-wambot-simulated],[Simulated WAM support (Def:yes)])],
   [],[with_wambot_simulated=yes])

# Make sure the user picks at least one
AS_IF(
   [test "$with_wambot_physical" = yes],[],
   [test "$with_wambot_simulated" = yes],[],
   [AC_MSG_FAILURE([at least one of --with-wambot-physical or --with-wambot-simulated required])]
)

# Set Automake Conditional
AM_CONDITIONAL([BT_WAMBOT_PHYSICAL],[test "$with_wambot_physical" = yes])
AM_CONDITIONAL([BT_WAMBOT_SIMULATED],[test "$with_wambot_simulated" = yes])

# Only compile bt_bus of physical is set
AM_CONDITIONAL([BT_BUS_ENABLE],[test "$with_wambot_physical" = yes])


### STEP 3: CAN CARD / INTERFACE ###
# Choose a CAN card/interface (for libbt/bus)
# This sets the varibles:
#   with_can_esd
#   with_can_peak_pci
#   with_can_peak_isa

AC_ARG_WITH([can-esd],
   [AS_HELP_STRING([--with-can-esd],[ESD CAN (Def:no)])],
   [],[with_can_esd=no])
AC_ARG_WITH([can-peak-pci],
   [AS_HELP_STRING([--with-can-peak-pci],[Peak CAN / PCI Bus (Def:no)])],
   [],[with_can_peak_pci=no])
AC_ARG_WITH([can-peak-isa],
   [AS_HELP_STRING([--with-can-peak-isa],[Peak CAN / ISA Bus (Def:no)])],
   [],[with_can_peak_isa=no])

AS_IF(
   # It doesn't matter; bt_bus it won't even compile.
   [! test "$with_wambot_physical" = yes],[],
   [test "$with_can_esd" = yes],
   [
      AC_SUBST([BT_SYS_CANTYPE],[CANTYPE_ESD])
      AC_SUBST([BT_SYS_LIBS],["-lntcan"])
   ],
   [test "$with_can_peak_pci" = yes],
   [
      AC_SUBST([BT_SYS_CANTYPE],[CANTYPE_PEAKPCI])
      AC_SUBST([BT_SYS_LIBS],["-lpcan"])
   ],
   [test "$with_can_peak_isa" = yes],
   [
      AC_SUBST([BT_SYS_CANTYPE],[CANTYPE_PEAKISA])
      AC_SUBST([BT_SYS_LIBS],["-lpcan"])
   ],
   [AC_MSG_FAILURE([one of --with-can-esd, --with-can-peak-pci, --with-can-peak-isa, or --without-wambot-physical required])]
)

# Checks for programs
AC_PROG_CC
AC_PROG_LIBTOOL

# Checks for libraries

# Woo!
AC_CONFIG_FILES([
 Makefile
 src/Makefile
 src/os/Makefile
 src/bus/Makefile
 src/wambot/Makefile
 src/kinematics/Makefile
 src/gravity/Makefile
 src/interp/Makefile
 src/trajectory/Makefile
 src/log/Makefile
 src/wam/Makefile
 src/gsl/Makefile
 src/control/Makefile
])
AC_OUTPUT
