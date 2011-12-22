set(programs
	gravitycal
	zerocal
)

foreach (prog ${programs})
	add_executable(${prog} get_key.cpp manage_backups.cpp ${prog}.cpp)
	target_link_libraries(${prog} barrett ${CURSES_LIBRARIES})
	set_target_properties(${prog} PROPERTIES
		PREFIX "bt-wam-"
		COMPILE_FLAGS "${Xeno_CFLAGS} ${GSL_CFLAGS}"
		LINK_FLAGS "${Xeno_LDFLAGS} ${GSL_LDFLAGS}"
	)
endforeach()
