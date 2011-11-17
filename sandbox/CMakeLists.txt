project(libbarrett_sandbox)

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} ${prog}.cpp)
		target_link_libraries(${prog} barrett ${CURSES_LIBRARIES})
		set_target_properties(${prog} PROPERTIES
			COMPILE_FLAGS "${Xeno_CFLAGS} ${GSL_CFLAGS}"
			LINK_FLAGS "${Xeno_LDFLAGS} ${GSL_LDFLAGS}"
		)
	endforeach()
endfunction()

set(programs
	autohome
	autotensioner
	can_terminal
	can_timing
	ft_persistent_tare
	full_system_test
	gimbals_hand_controller
	hand
	hand_buttons_test
	hand_self_preservation
	high_level_wam
	inverse_dynamics_test
	inverse_dynamics_test_teach_and_play
	inverse_dynamics_test_teach_and_play_accel
	load_ft_cal
#	log_ft_data_with_wam
	log_ft_data
	log_hand_jp
	log_hand_tact
	log_sg_data
	log_temp_data
#	log_velocity
	low_level_wam
	mouse_follow
	puck_terminal
	safety_module
	tactile_test
	teach_with_hand
	two_wams
	vibrate_wam
)

add_programs(${programs})

target_link_libraries(hand_buttons_test parapin)


if (WITH_PYTHON)
	find_package(Boost 1.40.0 REQUIRED python)
	include_directories(${Boost_INCLUDE_DIRS})
	link_directories(${Boost_LIBRARY_DIRS})
	
	find_package(PythonLibs REQUIRED)
	include_directories(${PYTHON_INCLUDE_PATH})
	link_directories(${PYTHON_LIBRARIES})
	
	add_library(btpy SHARED python.cpp)
	target_link_libraries(btpy barrett ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
	set_target_properties(btpy PROPERTIES
		COMPILE_FLAGS "${Xeno_CFLAGS} ${GSL_CFLAGS}"
		LINK_FLAGS "${Xeno_LDFLAGS} ${GSL_LDFLAGS}"
	)
endif()
