cmake_minimum_required(VERSION 2.8.3)
project(mobile_control)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  my_controller_pkg
  roscpp
  trajectory_msgs
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
)

# add_library(${PROJECT_NAME} 
  # src/xxx.cpp
# )
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Executable: xxx
# add_executable(xxx src/xxx.cpp)
# target_link_libraries(xxx ${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(TARGETS ${PROJECT_NAME} #xxx 
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY config  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
