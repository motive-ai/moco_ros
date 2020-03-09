cmake_minimum_required(VERSION 2.8.3)
project(motive_description)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
