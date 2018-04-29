#include <my_controller_pkg/effort_test_controller.h>

using namespace my_controller_pkg;

bool EffortTestController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
{
	// get all joint states from the hardware interface
	// const std::vector<std::string>& joint_names = hw->getJointNames();
	// for (unsigned i=0; i<joint_names.size(); i++)
	//	ROS_INFO("Got joint %s", joint_names[i].c_str());
	std::vector<std::string> joint_names;

	if (!n.getParam("joints", joint_names))
	{
		joint_names.push_back("hiDOF_joint1");
		joint_names.push_back("hiDOF_joint2");
	}

	for (unsigned i=0; i<joint_names.size(); i++)
		joint_effort_commands_.push_back(hw->getHandle(joint_names[i]));

	return true;
}

void EffortTestController::starting(const ros::Time& /*time*/)
{
	ROS_INFO("Starting JointState Controller");
}

void EffortTestController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
	for (unsigned int i=0; i < joint_effort_commands_.size(); i++)
	{

	}
}

void EffortTestController::stopping(const ros::Time& /*time*/)
{
	ROS_INFO("Stopping JointState Controller");
}

PLUGINLIB_EXPORT_CLASS( my_controller_pkg::EffortTestController, controller_interface::ControllerBase)
