set(programs
	gravitycal
	zerocal
	autotension
)


foreach (prog ${programs})
	add_executable(${prog} get_key.cpp manage_backups.cpp ${prog}.cpp)
	target_link_libraries(${prog}
		barrett
		${CURSES_LIBRARIES}
		${Boost_LIBRARIES}
		${GSL_LIBRARIES}
		${XENOMAI_LIBRARY_XENOMAI} ${XENOMAI_LIBRARY_NATIVE}
	)
	set_target_properties(${prog} PROPERTIES
		PREFIX "bt-wam-"
	)

	install(TARGETS ${prog} RUNTIME DESTINATION bin)
endforeach()


# Don't install wamdiscover. It's intended for the development system, not the
# WAM-PC.
#install(PROGRAMS wamdiscover DESTINATION bin)
