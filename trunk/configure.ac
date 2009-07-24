AC_INIT([libbarrett], [0.1], [cd@barrett.com])
AC_CONFIG_HEADERS([config.h])

# Note, these are automake warnings, not compiler ones
AM_INIT_AUTOMAKE([-Wall -Werror foreign])


# First, grab input from the user into $with_xxx variables
AC_ARG_WITH([async-only],
   [AS_HELP_STRING([--with-async-only],[Async Only Support])],
   [],[with_async_only=no])

AC_ARG_WITH([xenomai],
   [AS_HELP_STRING([--with-xenomai],[Xenomai Real-Time Support])],
   [],[with_xenomai=no])
AC_ARG_WITH([rtai],
   [AS_HELP_STRING([--with-rtai],[RTAI Real-Time Support])],
   [],[with_rtai=no])
AC_ARG_WITH([nortsys],
   [AS_HELP_STRING([--with-nortsys],[No Real-Time System Supported])],
   [],[with_nortsys=no])

AC_ARG_WITH([wambot-physical],
   [AS_HELP_STRING([--with-wambot-physical],[Physical WAM support (Def:yes)])],
   [],[with_wambot_physical=yes])
AC_ARG_WITH([wambot-simulated],
   [AS_HELP_STRING([--with-wambot-simulated],[Simulated WAM support (Def:no)])],
   [],[with_wambot_simulated=no])
   
AC_ARG_WITH([can-esd],
   [AS_HELP_STRING([--with-can-esd],[ESD CAN (Def:no)])],
   [],[with_can_esd=no])
AC_ARG_WITH([can-peak-pci],
   [AS_HELP_STRING([--with-can-peak-pci],[Peak CAN / PCI Bus (Def:no)])],
   [],[with_can_peak_pci=no])
AC_ARG_WITH([can-peak-isa],
   [AS_HELP_STRING([--with-can-peak-isa],[Peak CAN / ISA Bus (Def:no)])],
   [],[with_can_peak_isa=no])

# Initialize config variables
bt_os_cpp=""
bt_os_ldflags=""
bt_os_libs=""
bt_bus_cantype=""
bt_bus_libs=""

# Make sure the user picks at least one
AS_IF(
   [test "x$with_async_only" != xno],
   [],
   [
      ### STEP 0: REQUIRED PACKAGES (if not async only) ###
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
      # Note - -D_GNU_SOURCE for pthread_mutexattr_settype in pthread.h
      #        See: http://ipozgaj.blogspot.com/2006/08/posix-threads-and-manual-pages-rant.html
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
         [test "x$with_rtai" != xno],
         [
            # Set up RTAI
            bt_os_cpp="$bt_os_cpp -D_GNU_SOURCE -DRTSYS_RTAI -I/usr/realtime/include"
            bt_os_libs="$bt_os_libs -L/usr/realtime/lib -llxrt -lpthread"
         ],
         [test "x$with_nortsys" != xno],
         [
            # No RTSYS Selected
            bt_os_cpp="$bt_os_cpp -D_GNU_SOURCE -DRTSYS_NONE"
            bt_os_libs="-lpthread"
         ],
         [AC_MSG_FAILURE([one of --with-xenomai, --with-rtai, --with-nortsys, or --with-async-only./ required])]
      )

      ### STEP 2: WAMBOT PHYSICAL AND/OR SIMULATION ###
      # Choose a physical/simulator option (between libbt/wambot_phys and libbt/wambot_sim)
      # This sets the variables:
      #   with_wambot_physical
      #   with_wambot_simulated

      # Make sure the user picks at least one
      AS_IF(
         [test "$with_wambot_physical" = yes],[],
         [test "$with_wambot_simulated" = yes],[],
         [AC_MSG_FAILURE([at least one of --with-wambot-physical or --with-wambot-simulated required])]
      )

      ### STEP 3: CAN CARD / INTERFACE ###
      # Choose a CAN card/interface (for libbt/bus)
      # This sets the varibles:
      #   with_can_esd
      #   with_can_peak_pci
      #   with_can_peak_isa
      AS_IF(
         # It doesn't matter; bt_bus it won't even compile.
         [! test "$with_wambot_physical" = yes],[],
         [test "$with_can_esd" = yes],
         [
            bt_bus_cantype="CANTYPE_ESD"
            bt_bus_libs="-lntcan"
         ],
         [test "$with_can_peak_pci" = yes],
         [
            bt_bus_cantype="CANTYPE_PEAKPCI"
            bt_bus_libs="-lpcan"
         ],
         [test "$with_can_peak_isa" = yes],
         [
            bt_bus_cantype="CANTYPE_PEAKISA"
            bt_bus_libs="-lpcan"
         ],
         [AC_MSG_FAILURE([one of --with-can-esd, --with-can-peak-pci, --with-can-peak-isa, or --without-wambot-physical required])]
      )
   ]
)

# Check Python
AM_PATH_PYTHON([2.4])
AC_MSG_CHECKING([for Python include directory])
pyincdir=`$PYTHON -c "from distutils import sysconfig; print sysconfig.get_python_inc()" 2>/dev/null`
AC_MSG_RESULT([$pyincdir])
AC_SUBST([BT_PYINCDIR],["$pyincdir"])


# Do substitutions
AC_SUBST([BT_OS_CPP],["$bt_os_cpp"])
AC_SUBST([BT_OS_LDFLAGS],["$bt_os_ldflags"])
AC_SUBST([BT_OS_LIBS],["$bt_os_libs"])
AC_SUBST([BT_BUS_CANTYPE],["$bt_bus_cantype"])
AC_SUBST([BT_BUS_LIBS],["$bt_bus_libs"])

# Set Automake Conditionals
AM_CONDITIONAL([BT_WAM_LOCAL],[! test "$with_async_only" = yes])
AM_CONDITIONAL([BT_WAMBOT_PHYSICAL],[test "$with_wambot_physical" = yes])
AM_CONDITIONAL([BT_WAMBOT_SIMULATED],[test "$with_wambot_simulated" = yes])
AM_CONDITIONAL([BT_BUS_ENABLE],[test "$with_wambot_physical" = yes])

# Checks for programs
AC_PROG_CC

AC_PROG_LIBTOOL

# Checks for libraries

# Woo!
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
