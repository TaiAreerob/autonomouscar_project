cmake_minimum_required(VERSION 3.0.2)
project(myodom)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS roscpp tf)
catkin_package(
)
 
include_directories(
 include ${catkin_INCLUDE_DIRS}
)
 
add_executable(myodom
 src/myodom.cpp
)
add_dependencies(myodom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(myodom
 ${catkin_LIBRARIES}
)

