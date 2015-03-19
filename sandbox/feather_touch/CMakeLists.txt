cmake_minimum_required(VERSION 2.6)
project(feather_touch)

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

add_executable(feather_touch feather_touch.cpp)
target_link_libraries(feather_touch ${BARRETT_LIBRARIES} ${CURSES_LIBRARIES})
