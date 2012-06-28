project(libbarrett_sandbox)

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} ${prog}.cpp)
		target_link_libraries(${prog}
			barrett
			${CURSES_LIBRARIES}
			${Boost_LIBRARIES}
			${GSL_LIBRARIES}
			${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_NATIVE}
		)
	endforeach()
endfunction()

set(programs
	autohome
	autotensioner
	can_terminal
	can_timing
	cv_moves
	ft_persistent_tare
	full_system_test
	gimbals_hand_controller
	hand
#	hand_buttons_test
	hand_self_preservation
	high_level_wam
	inverse_dynamics_test
	inverse_dynamics_test_teach_and_play
	inverse_dynamics_test_teach_and_play_accel
	joint_encoder_index_adjustments
	joint_encoder_init
	load_ft_cal
#	log_ft_data_with_wam
	log_ft_data
	log_hand_jp
	log_hand_tact
	log_sg_data
	log_temp_data
#	log_velocity
	low_level_wam
#	mouse_follow
	os_test
	point_to_point_moves
	puck_terminal
	quaternion_interpolation
	read_pendant_state
	record_jp
	safety_module
	tactile_test
	teach_with_hand
	tuning
	two_wams
	vibrate_wam
	voltage_control
	wrist
	wrist_teach_with_hand
)

add_programs(${programs})

#target_link_libraries(hand_buttons_test parapin)


if (WITH_PYTHON)
	add_library(btpy SHARED python.cpp)
	target_link_libraries(btpy
		barrett
		${Boost_LIBRARIES}
		${PYTHON_LIBRARIES}
		${GSL_LIBRARIES}
		${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_NATIVE}
	)
endif()
