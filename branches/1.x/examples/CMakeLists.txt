project(libbarrett_examples)

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} ${prog}.cpp)
		target_link_libraries(${prog}
			barrett
			${GSL_LIBRARIES}
			${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_NATIVE}
		)
	endforeach()
endfunction()


add_definitions(${XENOMAI_DEFINITIONS})
add_programs(
	ex01_initialize_wam
	ex02_hold_position
	ex03_simple_move
	ex04_display_basic_info
	ex05_systems_intro
	ex07_realtime_move
	ex08_realtime_data_logging
	ex09_teach_and_play
	ex10_torque_control
	ex11_haptics
	ex13_master_master
)


# ex04 uses the ncurses library
target_link_libraries(ex04_display_basic_info ${CURSES_LIBRARIES})
