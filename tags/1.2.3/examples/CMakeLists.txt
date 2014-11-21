cmake_minimum_required(VERSION 2.6)
project(libbarrett_examples)


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
	ex01_initialize_wam
	ex02_hold_position
	ex03_simple_move
	ex04_display_basic_info
	ex05_systems_intro
	ex06_realtime_move
	ex07_realtime_data_logging
	ex08_teach_and_play
	ex09_torque_control
	ex10_haptics
	ex11_master_master
)

# ex04 uses the ncurses library
target_link_libraries(ex04_display_basic_info ${CURSES_LIBRARIES})
