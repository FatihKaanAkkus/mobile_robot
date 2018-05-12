cmake_minimum_required(VERSION 2.8.3)
project(mobile_navigation)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    tf
)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(send_goal src/send_robot_goal.cpp)

target_link_libraries(send_goal
    ${catkin_LIBRARIES}
)
