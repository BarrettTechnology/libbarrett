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
	ex02_hold_position
	ex03_simple_move
	ex04_display_basic_info
	#ex06_systems_intro
	ex07_realtime_move
	ex09_teach_and_play
	ex11_haptics
	ex13_master_master
	seven_dof_only
)


# ex04 uses the ncurses library
target_link_libraries(ex04_display_basic_info ${CURSES_LIBRARIES})
