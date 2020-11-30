cmake_minimum_required(VERSION 3.9)
project(range_mi)

# add compile options
add_compile_options(-Wall -Wextra)

# Compile as C++20
set(CMAKE_CXX_STANDARD 20)
# don't use gnu extensions
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
	std_srvs
	std_msgs
	roscpp
	nav_msgs
	geometry_msgs
	message_generation
)

add_message_files(
  FILES
  MIGrid.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/${PROJECT_NAME}/grid_line.cpp
	src/${PROJECT_NAME}/grid_mi.cpp
	include/${PROJECT_NAME}/barely_distorted.hpp
	include/${PROJECT_NAME}/distorted.hpp
	include/${PROJECT_NAME}/p_not_measured.hpp
)

add_executable(monte_carlo_mi src/monte_carlo_mi_node.cpp)
add_executable(occupancy_grid_mi src/occupancy_grid_mi_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(monte_carlo_mi
  ${catkin_LIBRARIES}
	${PROJECT_NAME}
)

target_link_libraries(occupancy_grid_mi
  ${catkin_LIBRARIES}
	${PROJECT_NAME}
)

install(TARGETS monte_carlo_mi occupancy_grid_mi
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY msg/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

option(BUILD_TESTS "BUILD_TESTS" OFF)
if (BUILD_TESTS)
  include_directories(test)
  add_subdirectory(test)
endif()
