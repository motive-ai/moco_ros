cmake_minimum_required(VERSION 3.1.0)
project(msa6_controllers)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED
REQUIRED COMPONENTS
  controller_manager
  roscpp
  rosparam_shortcuts
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES msa6_controllers
  CATKIN_DEPENDS controller_manager urdf
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

# Moco Hardware Interface
add_library(msa6_hw_interface
            src/msa6_hw_interface.cpp
)
target_link_libraries(msa6_hw_interface
  ${catkin_LIBRARIES}
)

# Moco Hardware Interface
add_library(msa6_hw_control_loop
            src/msa6_hw_control_loop.cpp
)
target_link_libraries(msa6_hw_control_loop
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(msa6_hw_main src/msa6_hw_main.cpp)
target_link_libraries(msa6_hw_main
  msa6_hw_interface
  msa6_hw_control_loop
  ${catkin_LIBRARIES}
)
message(STATUS "msa6_controllers : ${CATKIN_PACKAGE_LIB_DESTINATION}, ${CATKIN_PACKAGE_BIN_DESTINATION}")
message(STATUS "inc: ${catkin_INCLUDE_DIRS}")
set(CATKIN_PACKAGE_BIN_DESTINATION ${CATKIN_GLOBAL_LIBEXEC_DESTINATION}/${PROJECT_NAME} )
set(CATKIN_PACKAGE_LIB_DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})

install(TARGETS msa6_hw_interface
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS msa6_hw_main
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/msa6_controllers/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
