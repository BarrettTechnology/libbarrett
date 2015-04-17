cmake_minimum_required(VERSION 2.6)
project(libbarrett_sandbox)


# By default, build programs with compiler optimizations and debugging symbols
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()


## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} ${prog}.cpp)
		target_link_libraries(${prog} ${BARRETT_LIBRARIES})
	endforeach()
endfunction()

add_programs(
  dual_rehab
  mirror_image
  rehab_gcomp
  rehab_world
  rowing
  simon
)

#target_link_libraries(hand_buttons_test parapin)
#target_link_libraries(full_system_test ${CURSES_LIBRARIES})
#target_link_libraries(tactile_test ${CURSES_LIBRARIES})
