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
	bus_manager
	can_terminal
	cartesian_hold
	friction_data
	hand_feedback
	hand_motor_control_stress_test
	haptics
	high_level_wam
	load_ft_cal
	log_ft_data
	log_ft_data_with_wam
	log_sg_data
	log_velocity
	low_level_wam
	master_master
	move_to
	network_haptics
	puck_terminal
	read_safety_state
	tactile_test
	teach
	update_properties
	zero_torque
)

add_programs(${programs})


#set(libs "")
#set(cflags "")
#set(ldflags "")
#add_programs(${programs})

