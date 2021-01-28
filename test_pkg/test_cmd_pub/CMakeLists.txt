cmake_minimum_required(VERSION 3.0.2)
project(test_cmd_pub)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(test_cmd_pub
  src/test_cmd_pub.cpp
)
add_dependencies(test_cmd_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_cmd_pub
  ${catkin_LIBRARIES}
)
