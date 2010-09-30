cmake_minimum_required(VERSION 2.6)

project(libbarrett)
set(libbarrett_VERSION_MAJOR 0)
set(libbarrett_VERSION_MINOR 2)
set(libbarrett_VERSION_PATCH 0)
set(libbarrett_VERSION "${libbarrett_VERSION_MAJOR}.${libbarrett_VERSION_MINOR}.${libbarrett_VERSION_PATCH}")
set(libbarrett_SOVERSION "${libbarrett_VERSION_MAJOR}.${libbarrett_VERSION_MINOR}")

# Fail to link if there are undefined symbols in a shared library
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/")
include_directories("${PROJECT_SOURCE_DIR}/library/src/")



### Options
option(BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON)



### Dependencies

## Xenomai
# TODO(dc): make into a finder?
set(Xeno_TARGETVERSION "2.5")

find_program(Xeno_CONFIGEXE "xeno-config" "/usr/xenomai/bin/")
if (${Xeno_CONFIGEXE})
	message(FATAL_ERROR "Could not find Xenomai installation with version ${Xeno_TARGETVERSION}. (http://www.xenomai.org/)")
endif()

execute_process(COMMAND ${Xeno_CONFIGEXE} --version OUTPUT_VARIABLE Xeno_VERSION)
string(STRIP ${Xeno_VERSION} Xeno_VERSION)
string(SUBSTRING ${Xeno_VERSION} 0 4 Xeno_SHORTVERSION)
if (NOT (${Xeno_SHORTVERSION} VERSION_EQUAL ${Xeno_TARGETVERSION}))
	message(FATAL_ERROR "Found incompatible version of Xenomai. Found: ${Xeno_VERSION}, Need ${Xeno_TARGETVERSION}")
endif()

execute_process(COMMAND ${Xeno_CONFIGEXE} --skin native --cflags OUTPUT_VARIABLE Xeno_CFLAGS)
string(STRIP ${Xeno_CFLAGS} Xeno_CFLAGS)
execute_process(COMMAND ${Xeno_CONFIGEXE} --skin native --ldflags OUTPUT_VARIABLE Xeno_LDFLAGS)
string(STRIP ${Xeno_LDFLAGS} Xeno_LDFLAGS)

message(STATUS "Xeonmai version: ${Xeno_VERSION} (${Xeno_CONFIGEXE})")


## GSL
# TODO(dc): make into a finder?
set(GSL_MINVERSION "1.14")

find_program(GSL_CONFIGEXE "gsl-config")
if (${GSL_CONFIGEXE})
	message(FATAL_ERROR "Could not find GSL installation with version ${GSL_MINVERSION}. (http://www.gnu.org/software/gsl/)")
endif()

execute_process(COMMAND ${GSL_CONFIGEXE} --version OUTPUT_VARIABLE GSL_VERSION)
string(STRIP ${GSL_VERSION} GSL_VERSION)
if (NOT ( (${GSL_VERSION} VERSION_GREATER ${GSL_MINVERSION}) OR (${GSL_VERSION} VERSION_EQUAL ${GSL_MINVERSION})))
	message(FATAL_ERROR "Found incompatible version of GSL. Found: ${GSL_VERSION}, Need >=${GSL_MINVERSION}")
endif()

execute_process(COMMAND ${GSL_CONFIGEXE} --cflags OUTPUT_VARIABLE GSL_CFLAGS)
string(STRIP ${GSL_CFLAGS} GSL_CFLAGS)
execute_process(COMMAND ${GSL_CONFIGEXE} --libs OUTPUT_VARIABLE GSL_LDFLAGS)
string(STRIP ${GSL_LDFLAGS} GSL_LDFLAGS)

message(STATUS "GSL version: ${GSL_VERSION} (${GSL_CONFIGEXE})")


## Boost
# Python code to generate version list:
#for minor in range(40, 60):
#	print '"1.%d" "1.%d.0"' % (minor, minor),
set(Boost_ADDITIONAL_VERSIONS "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0" "1.47" "1.47.0" "1.48" "1.48.0" "1.49" "1.49.0" "1.50" "1.50.0" "1.51" "1.51.0" "1.52" "1.52.0" "1.53" "1.53.0" "1.54" "1.54.0" "1.55" "1.55.0" "1.56" "1.56.0" "1.57" "1.57.0" "1.58" "1.58.0" "1.59" "1.59.0")
find_package(Boost 1.40.0 REQUIRED thread)
include_directories(${Boost_INCLUDE_DIRS})
#      add_executable(foo foo.cc)
#      target_link_libraries(foo ${Boost_LIBRARIES})


## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


## Eigen2
find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})



# Targets
add_subdirectory(library)
add_subdirectory(programs)

