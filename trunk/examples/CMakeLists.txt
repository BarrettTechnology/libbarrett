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
	hold_joint_position
	polynomial_evaluator
	seven_dof_only
	zero_torque
)

