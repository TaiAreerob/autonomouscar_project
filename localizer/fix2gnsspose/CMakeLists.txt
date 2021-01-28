cmake_minimum_required(VERSION 3.0.2)
project(fix2gnsspose)
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs geometry_msgs tf)
find_package(GeographicLib REQUIRED)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

catkin_package(
)

add_executable(fix2tfpose
  src/fix2tfpose.cpp
)

add_dependencies(fix2tfpose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(fix2tfpose
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_definitions(${GeographicLib_DEFINITIONS})