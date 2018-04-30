cmake_minimum_required(VERSION 2.8.3)
project(my_controller_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  trajectory_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    ControllerState.msg
    SimpleControllerCommand.msg
    ArmControllerCommand.msg
    WheelControllerCommand.msg
    DynamicPIDParameters.msg
)

# add_service_files(
#     DIRECTORY srv
#     FILES 
#         xxx.srv
# )

# add_action_files(
#     DIRECTORY action
#     FILES
#         xxx.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
    trajectory_msgs 
    std_msgs 
    geometry_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
    geometry_msgs
    message_generation
    message_runtime
    trajectory_msgs
)