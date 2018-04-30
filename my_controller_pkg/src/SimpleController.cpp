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

				err.resize(effortJointHandles.size(), 0.0);
				err_old.resize(effortJointHandles.size(), 0.0);
				err_sum.resize(effortJointHandles.size(), 0.0);
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

		// Coefficients
		if(!n.getParam("Kp", coeff_Kp)){
			ROS_INFO("No Kp coefficient is given.");
			coeff_Kp = 0.0;
		}

		if(!n.getParam("Ki", coeff_Ki)){
			ROS_INFO("No Ki coefficient is given.");
			coeff_Ki = 0.0;
		}

		if(!n.getParam("Kd", coeff_Kd)){
			ROS_INFO("No Kd coefficient is given.");
			coeff_Kd = 0.0;
		}

		if(!n.getParam("clamp_iMax", clamp_iMax)){
			ROS_INFO("No clamp_iMax coefficient is given for clamping.");
			clamp_iMax = 0.0;
		}

		ROS_INFO("coeff_Kp: %lf, coeff_Ki: %lf, coeff_Kd: %lf", coeff_Kp, coeff_Ki, coeff_Kd);

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

			err[i] = 0.0;
			err_old[i] = 0.0;
			err_sum[i] = 0.0;
		}
	}

	void SimpleController::update(const ros::Time& time, const ros::Duration& period)
	{
		// Period
		ros::Duration dt = time - last_time_;

		my_controller_msgs::SimpleControllerCommand command;

		command = cmd_box_;

		for(unsigned int i = 0; i < effortJointHandles.size(); ++i)
		{
			buffer_current_positions[i] = effortJointHandles[i].getPosition();
			buffer_current_velocities[i] = effortJointHandles[i].getVelocity();

			// error
			err[i] = ((double) command.command_data) - buffer_current_positions[i];
			// integral of error
			err_sum[i] += dt.toSec() * err[i];
			// limit integral
			if(clamp_iMax != 0.0){
				if(err_sum[i] > clamp_iMax / coeff_Ki) 		err_sum[i] = clamp_iMax / coeff_Ki;
				if(err_sum[i] < -clamp_iMax / coeff_Ki) 	err_sum[i] = -clamp_iMax / coeff_Ki;
			}
			// formula
			double pid = coeff_Kp * err[i] + coeff_Ki * err_sum[i] + coeff_Kd * (err[i]-err_old[i]) / dt.toSec();

			err_old[i] = err[i];

			buffer_command_effort[i] = pid;
			effortJointHandles[i].setCommand(buffer_command_effort[i]);
		}

		last_time_ = time;
	}

	void SimpleController::stopping(const ros::Time& time)
	{}

	void SimpleController::sub_cmd_Callback(const my_controller_msgs::SimpleControllerCommand& msg)
	{
		cmd_box_ = msg;
	}
}

PLUGINLIB_EXPORT_CLASS(my_controller_pkg::SimpleController, controller_interface::ControllerBase)
