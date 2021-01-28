cmake_minimum_required(VERSION 3.0.2)
project(fake_traffic_sender)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  traffic_light_msgs
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

catkin_package(
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)
add_executable(fake_traffic_sender
  src/fake_traffic_sender.cpp
)
add_dependencies(fake_traffic_sender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_traffic_sender
  ${catkin_LIBRARIES}
)
