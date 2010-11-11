# Listing sources explicitly allows cmake to notice when a new source file is added.
#file(GLOB_RECURSE barrett_SOURCES "src/barrett/*.cpp" "src/barrett/*.c")
set(barrett_SOURCES
	bus/bus_manager.cpp
	bus/can_socket.cpp
	bus/communications_bus.cpp
	
	cdlbt/calgrav.c
	cdlbt/dynamics.c
	cdlbt/gsl.c
	cdlbt/interp.c
	cdlbt/kinematics.c
	cdlbt/profile.c
	cdlbt/spline.c
	
	math/trapezoidal_velocity_profile.cpp

	products/force_torque_sensor.cpp
	products/hand.cpp
	products/motor_puck.cpp
	products/multi_puck_product.cpp
	products/property_list.cpp
	products/puck.cpp
	products/puck_group.cpp
	products/safety_module.cpp

	systems/execution_manager.cpp
	systems/real_time_execution_manager.cpp
	systems/system.cpp

	thread/null_mutex.cpp
	thread/real_time_mutex.cpp

	exception.cpp
	stl_utils.cpp
)

# Defenitions for cdlbt code
add_definitions(-DCANTYPE_SOCKET -DHAVE_INLINE -DHAS_WAM_LOCAL -DRTSYS_XENOMAI)

add_library(barrett ${barrett_SOURCES})
target_link_libraries(barrett ${Boost_LIBRARIES} rtdm config config++)  #TODO(dc): libconfig finder?
set_target_properties(barrett PROPERTIES
	OUTPUT_NAME barrett
	VERSION ${${PROJECT_NAME}_VERSION}
	SOVERSION ${${PROJECT_NAME}_SOVERSION}
	
	COMPILE_FLAGS "${Xeno_CFLAGS} ${GSL_CFLAGS}"
	LINK_FLAGS "${Xeno_LDFLAGS} ${GSL_LDFLAGS}"
	
	# Allow creating static and shared libs without conflicts
	CLEAN_DIRECT_OUTPUT 1
	
	# Prevent executables from having to link with all of libbarrett's dependencies
	LINK_INTERFACE_LIBRARIES ""
)

