# Always compile these files
set(barrett_SOURCES
	bus/bus_manager.cpp
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
	products/gimbals_hand_controller.cpp
	products/hand.cpp
	products/motor_puck.cpp
	products/multi_puck_product.cpp
	products/product_manager.cpp
	products/property_list.cpp
	products/puck.cpp
	products/puck_group.cpp
	products/safety_module.cpp
	products/tactile_puck.cpp

	systems/execution_manager.cpp
	systems/ramp.cpp
	systems/real_time_execution_manager.cpp
	systems/system.cpp

	thread/null_mutex.cpp

	exception.cpp
	stl_utils.cpp
)

set(OS_dependent_sources
	thread/disable_secondary_mode_warning.cpp
	thread/real_time_mutex.cpp

	os.cpp
)
# Always compile OS-dependent sources
list(APPEND barrett_SOURCES ${OS_dependent_sources})

if (NON_REALTIME)
	# Linux only sources
	list(APPEND barrett_SOURCES
		bus/can_socket-pcan.cpp
	)
else()
	foreach (src_file ${OS_dependent_sources})
		# Define BARRETT_XENOMAI preprocessor symbol
		set_property(
			SOURCE ${src_file}
			APPEND PROPERTY COMPILE_DEFINITIONS BARRETT_XENOMAI
		)
	endforeach()
	
	# Xenomai only sources
	list(APPEND barrett_SOURCES
		bus/can_socket-rtcan.cpp
	)
endif()

# Maybe include Python sources
if (WITH_PYTHON)
	list(APPEND barrett_SOURCES
		python.cpp
		bus/python.cpp
		products/python/namespace.cpp
		products/python/product_manager.cpp
		products/python/puck.cpp
	)
endif()


set(libs ${Boost_LIBRARIES} ${GSL_LIBRARIES} config config++)  #TODO(dc): libconfig finder?
if (NON_REALTIME)
	set(libs ${libs} ${PCAN_LIB})
else()
	set(libs ${libs} ${XENOMAI_LIBRARY_NATIVE} ${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_RTDM})
endif()
if (WITH_PYTHON)
	set(libs ${libs} ${PYTHON_LIBRARIES})
endif()

add_library(barrett ${barrett_SOURCES})
target_link_libraries(barrett ${libs})
set_target_properties(barrett PROPERTIES
	OUTPUT_NAME barrett
	VERSION ${${PROJECT_NAME}_VERSION}
	SOVERSION ${${PROJECT_NAME}_SOVERSION}
	
	# Allow creating static and shared libs without conflicts
	CLEAN_DIRECT_OUTPUT 1
)

# Record the libraries that applications should link against if they want to use libbarrett
set(exported_libraries ${libs} barrett PARENT_SCOPE)


install(TARGETS barrett
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

install(CODE "message(STATUS \"Running ldconfig...\")")
install(CODE "execute_process(COMMAND ldconfig)")
