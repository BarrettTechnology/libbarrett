project(libbarrett_examples)

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} ${prog}.cpp)
		target_link_libraries(${prog} barrett)
	endforeach()
endfunction()


add_programs(
	hold_joint_position
	polynomial_evaluator
	seven_dof_only
	zero_torque
)

