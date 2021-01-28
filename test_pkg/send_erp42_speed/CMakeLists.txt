cmake_minimum_required(VERSION 3.0.2)
project(send_erp42_speed)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  erp42_msgs
  roscpp
  std_msgs
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(send_erp42_speed
  src/send_erp42_speed.cpp
)
add_dependencies(send_erp42_speed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(send_erp42_speed
  ${catkin_LIBRARIES}
)