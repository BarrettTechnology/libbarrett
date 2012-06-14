cmake_minimum_required(VERSION 2.6)

project(libbarrett)
set(libbarrett_VERSION_MAJOR 1)
set(libbarrett_VERSION_MINOR 0)
set(libbarrett_VERSION_PATCH 1)
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



### Options
option(BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON)
option(NON_REALTIME "Set to ON to avoid building code that depends on a real-time operating system" OFF)
option(OPTIMIZE_FOR_PROCESSOR "Set to ON to build binaries that are optimized for this specific computer and can't be copied to other machines" OFF)
option(WITH_PYTHON "Set to ON to build Python bindings for libbarrett (EXPERIMENTAL)" OFF)
option(INSTALL_EXAMPLES "Set to ON to copy libbarrett example programs to the current user's home folder when the library is installed" ON)

if (OPTIMIZE_FOR_PROCESSOR)
	# TODO(dc): Does this turn on sse2 if supported by processor? What about -mfpmath=sse?
	set(NEW_FLAGS "-march=native -mtune=native")
	
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${NEW_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NEW_FLAGS}")
	
	message(STATUS "OPTIMIZE_FOR_PROCESSOR: ${NEW_FLAGS}")
	set(NEW_FLAGS)  # unset
endif()

# Record the name of the user executing the cmake command
execute_process(COMMAND whoami OUTPUT_VARIABLE current_user OUTPUT_STRIP_TRAILING_WHITESPACE)

set(CONFIG_FILE_OWNER ${current_user} CACHE STRING "When installed, config files will be owned by this user.")
message(STATUS "When installed, config files will be owned by: ${CONFIG_FILE_OWNER}")

set(EXAMPLES_DIR /home/${current_user}/libbarrett_examples CACHE PATH "If installed, example programs will be placed in this directory.")
set(EXAMPLES_DIR_OWNER ${current_user} CACHE STRING "If installed, example programs will be owned by this user.")
if (INSTALL_EXAMPLES)
	message(STATUS "When installed, example code will be placed in: ${EXAMPLES_DIR}")
	message(STATUS "When installed, example code will be owned by: ${EXAMPLES_DIR_OWNER}")
endif()



### Dependencies

# Record these for when the library is installed
set(exported_include_dirs "")
set(exported_link_dirs "")
set(exported_definitions "")
set(exported_libraries "")


## Put libbarrett headers in the include path
include_directories("${PROJECT_SOURCE_DIR}/include/")
set(exported_include_dirs ${exported_include_dirs} ${CMAKE_INSTALL_PREFIX}/include)
set(exported_link_dirs ${exported_link_dirs} ${CMAKE_INSTALL_PREFIX}/lib)


## Xenomai
if (NON_REALTIME)
	message(STATUS "NON_REALTIME: Not using Xeonmai")
else()
	find_package(Xenomai REQUIRED)
	include_directories(${XENOMAI_INCLUDE_DIR})
	add_definitions(${XENOMAI_DEFINITIONS})
	
	set(exported_include_dirs ${exported_include_dirs} ${XENOMAI_INCLUDE_DIR})
	set(exported_definitions ${exported_definitions} ${XENOMAI_DEFINITIONS})
endif()


## GSL
find_package(GSL REQUIRED)
include_directories(${GSL_INCLUDE_DIRS})
set(exported_include_dirs ${exported_include_dirs} ${GSL_INCLUDE_DIRS})


## Boost
# Python snippet to generate version list:
#for minor in range(45, 60):
#	print '"1.%d" "1.%d.0"' % (minor, minor),
set(Boost_ADDITIONAL_VERSIONS "1.45" "1.45.0" "1.46" "1.46.0" "1.47" "1.47.0" "1.48" "1.48.0" "1.49" "1.49.0" "1.50" "1.50.0" "1.51" "1.51.0" "1.52" "1.52.0" "1.53" "1.53.0" "1.54" "1.54.0" "1.55" "1.55.0" "1.56" "1.56.0" "1.57" "1.57.0" "1.58" "1.58.0" "1.59" "1.59.0")
set(boost_components thread)
if (WITH_PYTHON)
	set(boost_components ${boost_components} python)
endif()
find_package(Boost 1.45.0 REQUIRED ${boost_components})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(exported_include_dirs ${exported_include_dirs} ${Boost_INCLUDE_DIRS})
set(exported_link_dirs ${exported_link_dirs} ${Boost_LIBRARY_DIRS})


## Python
if (WITH_PYTHON)
	find_package(PythonLibs REQUIRED)
	include_directories(${PYTHON_INCLUDE_PATH})
endif()


## Eigen2
find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})
set(exported_include_dirs ${exported_include_dirs} ${EIGEN2_INCLUDE_DIR})


## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})



### Main target
# src/CMakeLists.txt sets ${exported_libraries}, so this must be included before
# barrett-config.cmake is generated.
add_subdirectory(src)



### Installation
# The trailing slash in "include/" is important
install(DIRECTORY include/
	DESTINATION include
	PATTERN ".svn" EXCLUDE
)


## A config file that other cmake projects can use to build against libbarrett
configure_file(${PROJECT_SOURCE_DIR}/cmake/barrett-config.cmake.in ${PROJECT_SOURCE_DIR}/cmake/barrett-config.cmake
	ESCAPE_QUOTES @ONLY
)
install(FILES ${PROJECT_SOURCE_DIR}/cmake/barrett-config.cmake
	DESTINATION share/barrett
)


## Installs or updates configuration files
# TODO(dc): Make this path configurable. It's currently hard-coded in:
#     src/products/product_manager.cpp
#     products/zerocal.cpp
#     products/gravitycal.cpp
#     ... others?
set(config_dir /etc/barrett/)
set(install_config_cmd ${PROJECT_SOURCE_DIR}/programs/install_config_files --source=${PROJECT_SOURCE_DIR}/config/ --dest=${config_dir})
add_custom_target(install_config ${install_config_cmd} VERBATIM)

install(DIRECTORY DESTINATION ${config_dir})  # Makes an empty directory
install(CODE "execute_process(COMMAND ${install_config_cmd})")

install(CODE "message(STATUS \"Changing the owner of ${config_dir} to ${CONFIG_FILE_OWNER}...\")")
install(CODE "execute_process(COMMAND chown -R ${CONFIG_FILE_OWNER}:${CONFIG_FILE_OWNER} ${config_dir})")


## Example code
if (INSTALL_EXAMPLES)
	install(DIRECTORY ${PROJECT_SOURCE_DIR}/examples/
		USE_SOURCE_PERMISSIONS
		DESTINATION ${EXAMPLES_DIR}
		FILES_MATCHING
		PATTERN "*.h"
		PATTERN "*.cpp"
		PATTERN "*.py"
		PATTERN "*.txt"
		PATTERN ".svn" EXCLUDE
		PATTERN "CMakeFiles" EXCLUDE
	)
	install(CODE "message(STATUS \"Changing the owner of ${EXAMPLES_DIR} to ${EXAMPLES_DIR_OWNER}...\")")
	install(CODE "execute_process(COMMAND chown -R ${EXAMPLES_DIR_OWNER}:${EXAMPLES_DIR_OWNER} ${EXAMPLES_DIR})")
endif()



### Packaging
add_custom_target(package ${PROJECT_SOURCE_DIR}/programs/build_tgz_package ${libbarrett_VERSION}
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
	VERBATIM
)



### Other targets
if (NON_REALTIME)
	add_subdirectory(programs EXCLUDE_FROM_ALL)
else()
	add_subdirectory(programs)
endif()

# Barrett_DIR allows the examples/CMakeLists.txt to find the correct
# barrett-config.cmake. There may not be an installed version, and, if there
# is, we shouldn't use it. We must add the examples/ directory after
# cmake/barrett-config.cmake has been generated.
set(Barrett_DIR ${PROJECT_SOURCE_DIR}/cmake/)
add_subdirectory(examples EXCLUDE_FROM_ALL)

add_subdirectory(sandbox EXCLUDE_FROM_ALL)
add_subdirectory(tests EXCLUDE_FROM_ALL)
