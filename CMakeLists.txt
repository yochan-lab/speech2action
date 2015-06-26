cmake_minimum_required(VERSION 2.8.3)
project(speech2action)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

install(PROGRAMS scripts/actions.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
