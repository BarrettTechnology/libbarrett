project(libbarrett_tests)

# TODO(dc): gtest finder

# Listing sources explicitly allows cmake to notice when a new source file is added.
#file(GLOB_RECURSE tests_SOURCES "*.cpp")
set(tests_SOURCES
	log/reader.cpp
	log/real_time_writer.cpp
	log/verify_file_contents.cpp
	log/writer.cpp

	math/first_order_filter.cpp
	math/kinematics.cpp
	math/matrix.cpp
	math/spline.cpp
	math/traits.cpp
	math/utils.cpp
	math/vector.cpp
	
	products/puck.cpp

	systems/abstract/controller.cpp
	systems/abstract/execution_manager.cpp
	systems/abstract/single_io.cpp
	systems/abstract/system.cpp
	systems/callback.cpp
	systems/constant.cpp
	systems/converter.cpp
	systems/first_order_filter.cpp
	systems/gain.cpp
	systems/helpers.cpp
	systems/io_conversion.cpp
	systems/manual_execution_manager.cpp
	systems/pid_controller.cpp
	systems/print_to_stream.cpp
	systems/ramp.cpp
	systems/rate_limiter.cpp
	systems/summer.cpp
	systems/summer-polarity.cpp
	#systems/tool_orientation.cpp
)

add_executable(tests ${tests_SOURCES})
target_link_libraries(tests
	barrett
	gtest_main
	${Boost_LIBRARIES}
	${GSL_LIBRARIES}
	${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_NATIVE}
)
