#include <my_controller_pkg/SimpleController.h>

namespace my_controller_pkg
{
	SimpleController::SimpleController()
	{}

	SimpleController::~SimpleController()
	{
		sub_cmd_.shutdown();
	}

	bool SimpleController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
	{
		effortJointHandles.clear();
		std::vector<std::string> jointNames;

		node_ = n;

		if(n.getParam("joints", jointNames))
		{	
			if(hw)
			{
				for(unsigned int i=0; i < jointNames.size(); ++i)
				{
					effortJointHandles.push_back(hw->getHandle(jointNames[i]));
				}

				buffer_command_effort.resize(effortJointHandles.size(), 0.0);
				buffer_current_positions.resize(effortJointHandles.size(), 0.0);
				buffer_current_velocities.resize(effortJointHandles.size(), 0.0);
				buffer_current_efforts.resize(effortJointHandles.size(), 0.0);
			}
			else
			{
				ROS_ERROR("Effort Joint Interface is empty in hardware interace.");
				return false;
			}
		}
		else
		{
			ROS_ERROR("No joints in given namespace: %s", n.getNamespace().c_str());
			return false;
		}

		// Subscribe to command topic 
		const std::string sub_topic("command");
		sub_cmd_ = node_.subscribe(sub_topic, (uint32_t) 1, &SimpleController::sub_cmd_Callback, this); 

		return true;
	} 

	void SimpleController::starting(const ros::Time& time)
	{
		for(unsigned int i = 0; i < effortJointHandles.size(); ++i)
		{
			buffer_current_positions[i] = effortJointHandles[i].getPosition();
			buffer_current_velocities[i] = effortJointHandles[i].getVelocity();
			buffer_current_efforts[i] = effortJointHandles[i].getEffort();
		}
	}

	void SimpleController::update(const ros::Time& time, const ros::Duration& period)
	{
		// Period
		ros::Duration dt = time - last_time_;

		my_controller_msgs::SimpleControllerCommandPtr command;

		command = cmd_box_;

		for(unsigned int i = 0; i < effortJointHandles.size(); ++i)
		{
			buffer_current_positions[i] = effortJointHandles[i].getPosition();
			buffer_current_velocities[i] = effortJointHandles[i].getVelocity();

			buffer_command_effort[i] = 0.0;
			
			effortJointHandles[i].setCommand(buffer_command_effort[i]);
		}	
	}

	void SimpleController::stopping(const ros::Time& time)
	{}

	void SimpleController::sub_cmd_Callback(const my_controller_msgs::SimpleControllerCommandPtr& msg)
	{
		cmd_box_ = msg;
	}
}

PLUGINLIB_EXPORT_CLASS(my_controller_pkg::SimpleController, controller_interface::ControllerBase)
