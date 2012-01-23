cmake_minimum_required(VERSION 2.6)

project(libbarrett)
set(libbarrett_VERSION_MAJOR 0)
set(libbarrett_VERSION_MINOR 2)
set(libbarrett_VERSION_PATCH 0)
set(libbarrett_VERSION "${libbarrett_VERSION_MAJOR}.${libbarrett_VERSION_MINOR}.${libbarrett_VERSION_PATCH}")
set(libbarrett_SOVERSION "${libbarrett_VERSION_MAJOR}.${libbarrett_VERSION_MINOR}")

# debug build by default
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

# Fail to link if there are undefined symbols in a shared library
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

# Location of custom Finders
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/")

# Put libbarrett headers in the include path
include_directories("${PROJECT_SOURCE_DIR}/include/")



### Options
option(BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON)
option(NON_REALTIME "Set to ON to avoid building code that depends on a real-time operating system" OFF)
option(OPTIMIZE_FOR_PROCESSOR "Set to ON to build binaries that are optimized for this specific computer and can't be copied to other machines" OFF)
option(WITH_PYTHON "Set to ON to build Python bindings for libbarrett (EXPERIMENTAL)" OFF)


if (OPTIMIZE_FOR_PROCESSOR)
	# TODO(dc): Does this turn on sse2 if supported by processor? What about -mfpmath=sse?
	set(NEW_FLAGS "-march=native -mtune=native")
	
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${NEW_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NEW_FLAGS}")
	
	message(STATUS "OPTIMIZE_FOR_PROCESSOR: ${NEW_FLAGS}")
	set(NEW_FLAGS)  # unset
endif()


### Dependencies

if (NON_REALTIME)
	message(STATUS "NON_REALTIME: Not using Xeonmai")
else()
	## Xenomai
	# TODO(dc): make into a finder?
	set(Xeno_TARGETVERSION "2.6")

	find_program(Xeno_CONFIGEXE "xeno-config" "/usr/xenomai/bin/")
	if (${Xeno_CONFIGEXE} STREQUAL "Xeno_CONFIGEXE-NOTFOUND")
		message(SEND_ERROR "Could not find Xenomai installation with version ${Xeno_TARGETVERSION}. (http://www.xenomai.org/)")
		unset(Xeno_CONFIGEXE CACHE)
	else()
		execute_process(COMMAND ${Xeno_CONFIGEXE} --version OUTPUT_VARIABLE Xeno_VERSION)
		string(STRIP ${Xeno_VERSION} Xeno_VERSION)
		string(SUBSTRING ${Xeno_VERSION} 0 4 Xeno_SHORTVERSION)
		if (NOT (${Xeno_SHORTVERSION} VERSION_EQUAL ${Xeno_TARGETVERSION}))
			message(SEND_ERROR "Found incompatible version of Xenomai. Found: ${Xeno_VERSION}, Need ${Xeno_TARGETVERSION}")
			unset(Xeno_CONFIGEXE CACHE)
			unset(Xeno_VERSION CACHE)
			unset(Xeno_SHORTVERSION CACHE)
		endif()
	
		execute_process(COMMAND ${Xeno_CONFIGEXE} --skin native --cflags OUTPUT_VARIABLE Xeno_CFLAGS)
		string(STRIP ${Xeno_CFLAGS} Xeno_CFLAGS)
		execute_process(COMMAND ${Xeno_CONFIGEXE} --skin native --ldflags OUTPUT_VARIABLE Xeno_LDFLAGS)
		string(STRIP ${Xeno_LDFLAGS} Xeno_LDFLAGS)
	
		message(STATUS "Xeonmai version: ${Xeno_VERSION} (${Xeno_CONFIGEXE})")
	endif()
endif()


## GSL
# TODO(dc): make into a finder?
set(GSL_MINVERSION "1.14")

find_program(GSL_CONFIGEXE "gsl-config")
if (${GSL_CONFIGEXE} STREQUAL "GSL_CONFIGEXE-NOTFOUND")
	message(SEND_ERROR "Could not find GSL installation with version ${GSL_MINVERSION}. (http://www.gnu.org/software/gsl/)")
	unset(GSL_CONFIGEXE CACHE)
else()
	execute_process(COMMAND ${GSL_CONFIGEXE} --version OUTPUT_VARIABLE GSL_VERSION)
	string(STRIP ${GSL_VERSION} GSL_VERSION)
	if (NOT ( (${GSL_VERSION} VERSION_GREATER ${GSL_MINVERSION}) OR (${GSL_VERSION} VERSION_EQUAL ${GSL_MINVERSION})))
		message(SEND_ERROR "Found incompatible version of GSL. Found: ${GSL_VERSION}, Need >=${GSL_MINVERSION}")
		unset(GSL_CONFIGEXE CACHE)
		unset(GSL_VERSION CACHE)
	endif()
	
	execute_process(COMMAND ${GSL_CONFIGEXE} --cflags OUTPUT_VARIABLE GSL_CFLAGS)
	string(STRIP ${GSL_CFLAGS} GSL_CFLAGS)
	execute_process(COMMAND ${GSL_CONFIGEXE} --libs OUTPUT_VARIABLE GSL_LDFLAGS)
	string(STRIP ${GSL_LDFLAGS} GSL_LDFLAGS)
	
	message(STATUS "GSL version: ${GSL_VERSION} (${GSL_CONFIGEXE})")
endif()


## Boost
# Python code to generate version list:
#for minor in range(45, 60):
#	print '"1.%d" "1.%d.0"' % (minor, minor),
set(Boost_ADDITIONAL_VERSIONS "1.45" "1.45.0" "1.46" "1.46.0" "1.47" "1.47.0" "1.48" "1.48.0" "1.49" "1.49.0" "1.50" "1.50.0" "1.51" "1.51.0" "1.52" "1.52.0" "1.53" "1.53.0" "1.54" "1.54.0" "1.55" "1.55.0" "1.56" "1.56.0" "1.57" "1.57.0" "1.58" "1.58.0" "1.59" "1.59.0")
find_package(Boost 1.45.0 REQUIRED thread)
include_directories(${Boost_INCLUDE_DIRS})


## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


## Eigen2
find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})



# Targets
add_subdirectory(src)
if (NON_REALTIME)
	add_subdirectory(programs EXCLUDE_FROM_ALL)
else()
	add_subdirectory(programs)
endif()

add_subdirectory(examples EXCLUDE_FROM_ALL)
add_subdirectory(sandbox EXCLUDE_FROM_ALL)
add_subdirectory(tests EXCLUDE_FROM_ALL)

