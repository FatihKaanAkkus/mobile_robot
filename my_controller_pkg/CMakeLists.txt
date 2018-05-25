cmake_minimum_required(VERSION 2.8.3)
project(my_controller_pkg)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  my_controller_msgs
  my_filter_pkg
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS 
    controller_manager 
    controller_interface
    my_controller_msgs
    my_filter_pkg
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}
  src/my_robot_hw.cpp
  src/SimpleController.cpp
  src/ArmController.cpp
  src/SimpleArmController.cpp
  src/WheelController.cpp
  src/SpeedLimiter.cpp
  src/Odometry.cpp
  src/MobileController.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(dummy_app src/dummy_app.cpp)
target_link_libraries(dummy_app ${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} dummy_app
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})