cmake_minimum_required(VERSION 2.8.3)
project(erp42_parking_manager)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)