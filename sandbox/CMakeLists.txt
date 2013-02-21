cmake_minimum_required(VERSION 2.6)
project(libbarrett_sandbox)


# By default, build programs with compiler optimizations and debugging symbols
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()


## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} ${prog}.cpp)
		target_link_libraries(${prog} ${BARRETT_LIBRARIES})
	endforeach()
endfunction()

add_programs(
	autohome
	can_terminal
#	can_timing
	constrain_to_path
	cv_moves
	ft_persistent_tare
	full_system_test
	gimbals_hand_controller
	hand
#	hand_buttons_test
#	hand_self_preservation
	high_level_wam
	inverse_dynamics_test
	inverse_dynamics_test_teach_and_play
	inverse_dynamics_test_teach_and_play_accel
	joint_encoder_index_adjustments
	joint_encoder_init
	joint_encoder_set_offsets
	load_ft_cal
#	log_ft_data
#	log_hand_jp
#	log_hand_tact
#	log_sg_data
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

#target_link_libraries(hand_buttons_test parapin)
target_link_libraries(full_system_test ${CURSES_LIBRARIES})
target_link_libraries(tactile_test ${CURSES_LIBRARIES})
