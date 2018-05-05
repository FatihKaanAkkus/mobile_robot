#include <ros/ros.h>
#include <mobile_control/command_joints_action.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "command_joints_action_node");
  ros::NodeHandle node;
  mobile_control::CommandJoints cjn(node);
  ros::spin();
  return 0;
}
