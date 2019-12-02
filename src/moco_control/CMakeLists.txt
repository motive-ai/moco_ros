
project(moco_control)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED REQUIRED COMPONENTS roscpp controller_manager urdf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moco_control
  CATKIN_DEPENDS controller_manager urdf
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

# Moco Hardware Interface
add_library(moco_hw_interface
            src/moco_hw_interface.cpp
)
target_link_libraries(moco_hw_interface
  ${catkin_LIBRARIES}
)

# Moco Hardware Interface
add_library(moco_hw_control_loop
            src/moco_hw_control_loop.cpp
)
target_link_libraries(moco_hw_control_loop
  ${catkin_LIBRARIES}
)


# Main control executable
add_executable(moco_hw_main src/moco_hw_main.cpp)
target_link_libraries(moco_hw_main
  moco_hw_interface
  moco_hw_control_loop
  ${catkin_LIBRARIES}
)
message(STATUS "moco_control : ${CATKIN_PACKAGE_LIB_DESTINATION}, ${CATKIN_PACKAGE_BIN_DESTINATION}")
message(STATUS "inc: ${catkin_INCLUDE_DIRS}")
set(CATKIN_PACKAGE_BIN_DESTINATION ${CATKIN_GLOBAL_LIBEXEC_DESTINATION}/${PROJECT_NAME} )
set(CATKIN_PACKAGE_LIB_DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS moco_hw_interface
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS moco_hw_main
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/moco_control/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
