/* src/include/xeno_config.h.  Generated from xeno_config.h.in by configure.  */
/* src/include/xeno_config.h.in.  Generated from configure.in by autoheader.  */

/* config */
/* #undef CONFIG_SMP */

/* config */
#define CONFIG_X86_TSC 1

/* config */
/* #undef CONFIG_XENO_ARM_ARCH */

/* config */
/* #undef CONFIG_XENO_ARM_EABI */

/* config */
/* #undef CONFIG_XENO_ARM_HW_DIRECT_TSC */

/* config */
/* #undef CONFIG_XENO_ARM_SA1100 */

/* Build system alias */
#define CONFIG_XENO_BUILD_STRING "i686-pc-linux-gnu"

/* Compiler */
#define CONFIG_XENO_COMPILER "gcc version 4.3.3 (Ubuntu 4.3.3-5ubuntu4) "

/* Host system alias */
#define CONFIG_XENO_HOST_STRING "i686-pc-linux-gnu"

/* config */
/* #undef CONFIG_XENO_POSIX_AUTO_MLOCKALL */

/* config */
/* #undef CONFIG_XENO_PSOS_AUTO_MLOCKALL */

/* config */
#define CONFIG_XENO_REVISION_LEVEL 2

/* config */
/* #undef CONFIG_XENO_UITRON_AUTO_MLOCKALL */

/* config */
#define CONFIG_XENO_VERSION_MAJOR 2

/* config */
#define CONFIG_XENO_VERSION_MINOR 4

/* config */
/* #undef CONFIG_XENO_X86_SEP */

#ifdef __IN_XENO__

/* Define to 1 if you have the <dlfcn.h> header file. */
#define HAVE_DLFCN_H 1

/* Define to 1 if you have the <inttypes.h> header file. */
#define HAVE_INTTYPES_H 1

/* Define to 1 if you have the <memory.h> header file. */
#define HAVE_MEMORY_H 1

/* Define to 1 if you have the <mqueue.h> header file. */
#define HAVE_MQUEUE_H 1

/* config */
/* #undef HAVE_OLD_SETAFFINITY */

/* config */
#define HAVE_RECENT_SETAFFINITY 1

/* Define to 1 if you have the `shm_open' function. */
#define HAVE_SHM_OPEN 1

/* Define to 1 if you have the `shm_unlink' function. */
#define HAVE_SHM_UNLINK 1

/* Define to 1 if you have the <stdint.h> header file. */
#define HAVE_STDINT_H 1

/* Define to 1 if you have the <stdlib.h> header file. */
#define HAVE_STDLIB_H 1

/* Define to 1 if you have the <strings.h> header file. */
#define HAVE_STRINGS_H 1

/* Define to 1 if you have the <string.h> header file. */
#define HAVE_STRING_H 1

/* Define to 1 if you have the <sys/stat.h> header file. */
#define HAVE_SYS_STAT_H 1

/* Define to 1 if you have the <sys/types.h> header file. */
#define HAVE_SYS_TYPES_H 1

/* Define to 1 if you have the <unistd.h> header file. */
#define HAVE_UNISTD_H 1

/* Name of package */
#define PACKAGE "xenomai"

/* Define to the address where bug reports for this package should be sent. */
#define PACKAGE_BUGREPORT "xenomai-help@gna.org"

/* Define to the full name of this package. */
#define PACKAGE_NAME "Xenomai"

/* Define to the full name and version of this package. */
#define PACKAGE_STRING "Xenomai 2.4.2"

/* Define to the one symbol short name of this package. */
#define PACKAGE_TARNAME "xenomai"

/* Define to the version of this package. */
#define PACKAGE_VERSION "2.4.2"

/* Define to 1 if you have the ANSI C header files. */
#define STDC_HEADERS 1

/* Version number of package */
#define VERSION "2.4.2"

/* Define to 1 if `lex' declares `yytext' as a `char *' by default, not a
   `char[]'. */
/* #undef YYTEXT_POINTER */

#endif /* __IN_XENO__ */
