cmake_minimum_required(VERSION 2.8.3)
project(mobile_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  my_controller_pkg
  roscpp
  trajectory_msgs
  actionlib
)

include_directories(
  include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    my_controller_pkg
    roscpp
    trajectory_msgs
    actionlib
)

add_library(${PROJECT_NAME}
#include <controller_manager/controller_manager.h> 
  src/command_joints_action.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Executable: command_joints_app
add_executable(command_joints_app src/command_joints_app.cpp)
target_link_libraries(command_joints_app ${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} command_joints_app
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY config  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
