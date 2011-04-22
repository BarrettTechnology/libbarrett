project(libbarrett_examples)

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} ${prog}.cpp)
		target_link_libraries(${prog} barrett)
		set_target_properties(${prog} PROPERTIES
			COMPILE_FLAGS "${Xeno_CFLAGS} ${GSL_CFLAGS}"
			LINK_FLAGS "${Xeno_LDFLAGS} ${GSL_LDFLAGS}"
		)
	endforeach()
endfunction()


add_programs(
	ex01_initialize_wam
	ex03_hold_position
	ex04_simple_move
	#ex06_systems_intro
	ex09_teach_and_play
	ex11_haptics
	ex13_master_master
	seven_dof_only
)

