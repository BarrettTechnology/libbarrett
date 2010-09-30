# Listing sources explicitly allows cmake to notice when a new source file is added.
#file(GLOB_RECURSE barrett_SOURCES "src/barrett/*.cpp" "src/barrett/*.c")
set(libbarrett_SOURCES
	src/barrett/bus/abstract/detail/communications_bus.cpp
	src/barrett/bus/detail/bus_manager.cpp
	src/barrett/bus/detail/can_socket.cpp
	src/barrett/cdlbt/bus/bus.c
	src/barrett/cdlbt/bus/bus_can.c
	src/barrett/cdlbt/calgrav/calgrav.c
	src/barrett/cdlbt/dynamics/dynamics.c
	src/barrett/cdlbt/gsl/gsl.c
	src/barrett/cdlbt/interp/interp.c
	src/barrett/cdlbt/kinematics/kinematics.c
	src/barrett/cdlbt/os/os.c
	src/barrett/cdlbt/profile/profile.c
	src/barrett/cdlbt/spline/spline.c
	src/barrett/cdlbt/wambot/wambot_phys.c
	src/barrett/detail/c_stacktrace.cpp
	src/barrett/detail/exception.cpp
	src/barrett/detail/puck.cpp
	src/barrett/math/detail/trapezoidal_velocity_profile.cpp
	src/barrett/systems/abstract/detail/execution_manager.cpp
	src/barrett/systems/abstract/detail/system.cpp
	src/barrett/systems/detail/real_time_execution_manager.cpp
	src/barrett/thread/detail/null_mutex.cpp
	src/barrett/thread/detail/real_time_mutex.cpp
)

# Defenitions for cdlbt code
add_definitions(-DCANTYPE_SOCKET -DHAVE_INLINE -DHAS_WAM_LOCAL -DRTSYS_XENOMAI)

add_library(libbarrett ${libbarrett_SOURCES})
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(libbarrett ${Boost_LIBRARIES} rtdm rtdk config config++)  #TODO(dc): libconfig finder?
set_target_properties(libbarrett PROPERTIES
	OUTPUT_NAME barrett
	VERSION ${${PROJECT_NAME}_VERSION}
	SOVERSION ${${PROJECT_NAME}_SOVERSION}
	
	COMPILE_FLAGS "${Xeno_CFLAGS} ${GSL_CFLAGS}"
	LINK_FLAGS "${Xeno_LDFLAGS} ${GSL_LDFLAGS}"
#	COMPILE_FLAGS ${Xeno_CFLAGS}
#	LINK_FLAGS ${Xeno_LDFLAGS}
	
	# Allow creating static and shared libs without conflicts
	CLEAN_DIRECT_OUTPUT 1
)

message("${Xeno_CFLAGS} ${Xeno_LDFLAGS} ${GSL_CFLAGS} ${GSL_LDFLAGS}")
