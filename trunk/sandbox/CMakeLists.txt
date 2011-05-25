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
	can_terminal
	can_timing
	ft_persistent_tare
	full_system_test
	gimbals_hand_controller
	hand
	high_level_wam
	load_ft_cal
#	log_ft_data_with_wam
	log_ft_data
	log_hand_jp
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
)

add_programs(${programs})


#set(libs "")
#set(cflags "")
#set(ldflags "")
#add_programs(${programs})

