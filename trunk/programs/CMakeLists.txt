set(programs gravitycal zerocal)

foreach (prog ${programs})
	add_executable(${prog} src/${prog}.cpp)
	target_link_libraries(${prog} barrett ${CURSES_LIBRARIES})
	set_target_properties(${prog} PROPERTIES
		PREFIX "bt-wam-"
	)
endforeach()

