#include <my_controller_pkg/WheelController.h>

namespace my_controller_pkg
{
	WheelController::WheelController()
	{}

	WheelController::~WheelController()
	{
		sub_cmd.shutdown();
	}

	bool WheelController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
	{
		node = n;

		if(node.getParam("joint", jointName))
		{	
			if(hw)
			{
				jointHandle = hw->getHandle(jointName);
			}
			else
			{
				ROS_ERROR("Effort Joint Interface is empty in hardware interace.");
				return false;
			}
		}
		else
		{
			ROS_ERROR("No joint in given namespace: %s", node.getNamespace().c_str());
			return false;
		}

		// Limit params assignments
		{
			// Position limits
			if(!node.getParam("has_position_limits", myJointLimits.has_position_limits))
			{
				ROS_INFO("Joint: %s >> No 'has_position_limits' is given. Assuming as 'false'.", jointName.c_str());
				myJointLimits.has_position_limits = false;
			}
			if(myJointLimits.has_position_limits)
			{
				if(!node.getParam("min_position", myJointLimits.min_position)){
					ROS_INFO("Joint: %s >> No 'min_position' is given. Assuming as '0.0'.", jointName.c_str());
					myJointLimits.min_position = 0.0;
				}
				if(!node.getParam("max_position", myJointLimits.max_position)){
					ROS_INFO("Joint: %s >> No 'max_position' is given. Assuming as '0.0'.", jointName.c_str());
					myJointLimits.max_position = 0.0;
				}
			}

			// Velocity limits
			if(!node.getParam("has_velocity_limits", myJointLimits.has_velocity_limits))
			{
				ROS_INFO("Joint: %s >> No 'has_velocity_limits' is given. Assuming as 'false'.", jointName.c_str());
				myJointLimits.has_velocity_limits = false;
			}

			if(myJointLimits.has_velocity_limits)
			{
				if(!node.getParam("max_velocity", myJointLimits.max_velocity)){
					ROS_INFO("Joint: %s >> No 'max_velocity' is given. Assuming as '0.0'.", jointName.c_str());
					myJointLimits.max_velocity = 0.0;
				}
			}

			// Acceleration limits
			if(!node.getParam("has_acceleration_limits", myJointLimits.has_acceleration_limits))
			{
				ROS_INFO("Joint: %s >> No 'has_acceleration_limits' is given. Assuming as 'false'.", jointName.c_str());
				myJointLimits.has_acceleration_limits = false;
			}

			if(myJointLimits.has_acceleration_limits)
			{
				if(!node.getParam("max_acceleration", myJointLimits.max_acceleration)){
					ROS_INFO("Joint: %s >> No 'max_acceleration' is given. Assuming as '0.0'.", jointName.c_str());
					myJointLimits.max_acceleration = 0.0;
				}
			}

			// Jerk limits
			if(!node.getParam("has_jerk_limits", myJointLimits.has_jerk_limits))
			{
				ROS_INFO("Joint: %s >> No 'has_jerk_limits' is given. Assuming as 'false'.", jointName.c_str());
				myJointLimits.has_jerk_limits = false;
			}

			if(myJointLimits.has_jerk_limits)
			{
				if(!node.getParam("max_jerk", myJointLimits.max_jerk)){
					ROS_INFO("Joint: %s >> No 'max_jerk' is given. Assuming as '0.0'.", jointName.c_str());
					myJointLimits.max_jerk = 0.0;
				}
			}

			// Effort limits
			if(!node.getParam("has_effort_limits", myJointLimits.has_effort_limits))
			{
				ROS_INFO("Joint: %s >> No 'has_effort_limits' is given. Assuming as 'false'.", jointName.c_str());
				myJointLimits.has_effort_limits = false;
			}

			if(myJointLimits.has_effort_limits)
			{
				if(!node.getParam("max_effort", myJointLimits.max_effort)){
					ROS_INFO("Joint: %s >> No 'max_effort' is given. Assuming as '0.0'.", jointName.c_str());
					myJointLimits.max_effort = 0.0;
				}
			}

		}

		// Coefficients
		if(!node.getParam("Kp", coeff_Kp)){
			ROS_INFO("Joint: %s >> No 'Kp' coefficient is given.", jointName.c_str());
			coeff_Kp = 0.0;
		}

		if(!node.getParam("Ki", coeff_Ki)){
			ROS_INFO("Joint: %s >> No 'Ki' coefficient is given.", jointName.c_str());
			coeff_Ki = 0.0;
		}

		if(!node.getParam("Kd", coeff_Kd)){
			ROS_INFO("Joint: %s >> No 'Kd' coefficient is given.", jointName.c_str());
			coeff_Kd = 0.0;
		}

		if(!node.getParam("clamp_iMax", clamp_iMax)){
			ROS_INFO("Joint: %s >> No 'clamp_iMax' is given for clamping.", jointName.c_str());
			clamp_iMax = 0.0;
		}

		// Subscribe to command topic 
		const std::string sub_cmd_topic("command");
		sub_cmd = node.subscribe(sub_cmd_topic, (uint32_t) 1, &WheelController::sub_cmd_Callback, this); 

		// Subscribe to pid topic 
		const std::string sub_pid_topic("dynamic_pid");
		sub_pid = node.subscribe(sub_pid_topic, (uint32_t) 1, &WheelController::sub_pid_Callback, this); 

		// Publish current state
		const std::string pub_topic("state");
		pub_state = node.advertise<my_controller_msgs::ControllerState>(pub_topic, (uint32_t) 1);

		return true;
	} 

	void WheelController::starting(const ros::Time& time)
	{
		buffer_current_position = jointHandle.getPosition();
		buffer_current_velocity = jointHandle.getVelocity();
		buffer_current_effort = jointHandle.getEffort();

		error = 0.0;
		error_old = 0.0;
		error_sum = 0.0;
	}

	void WheelController::update(const ros::Time& time, const ros::Duration& period)
	{
		// Period
		ros::Duration dt = time - last_time;

		my_controller_msgs::WheelControllerCommand command;
		my_controller_msgs::ControllerState state;

		command = cmd_box;
		
		// Velocity limits
		if(myJointLimits.has_velocity_limits)
		{
			if(command.command_data > myJointLimits.max_velocity) 	command.command_data = myJointLimits.max_velocity;
			if(command.command_data < -myJointLimits.max_velocity) 	command.command_data = -myJointLimits.max_velocity;
		}

		if(dt.toSec() > 0.005)
		{
			buffer_current_position = jointHandle.getPosition();
			buffer_current_velocity = jointHandle.getVelocity();
			buffer_current_effort = jointHandle.getEffort();

			// error
			error = ((double) command.command_data) - buffer_current_velocity;
			// integral of error
			error_sum += dt.toSec() * error;
			// limit integral
			if(clamp_iMax != 0.0){
				if(error_sum > clamp_iMax / coeff_Ki) 	error_sum = clamp_iMax / coeff_Ki;
				if(error_sum < -clamp_iMax / coeff_Ki) 	error_sum = -clamp_iMax / coeff_Ki;
			}

			// formula
			double pid = coeff_Kp * error + coeff_Ki * error_sum + coeff_Kd * (error-error_old) / dt.toSec();

			error_old = error;

			buffer_command_effort = pid;

			// Effort limits
			if(myJointLimits.has_effort_limits)
			{
				if(buffer_command_effort > myJointLimits.max_effort) 	buffer_command_effort = myJointLimits.max_effort;
				if(buffer_command_effort < -myJointLimits.max_effort) 	buffer_command_effort = -myJointLimits.max_effort;
			}

			jointHandle.setCommand(buffer_command_effort);

			// Publish to topic
			state.position = buffer_current_position;
			state.velocity = buffer_current_velocity;
			state.effort = buffer_current_effort;
			pub_state.publish(state);

			last_time = time;
		}
	}

	void WheelController::stopping(const ros::Time& time)
	{}

	void WheelController::sub_cmd_Callback(const my_controller_msgs::WheelControllerCommand& msg)
	{
		cmd_box = msg;
	}

	void WheelController::sub_pid_Callback(const my_controller_msgs::DynamicPIDParameters& msg)
	{
		coeff_Kp = msg.Kp;
		coeff_Ki = msg.Ki;
		coeff_Kd = msg.Kd;
		clamp_iMax = msg.clamp_iMax;
	}
}

PLUGINLIB_EXPORT_CLASS(my_controller_pkg::WheelController, controller_interface::ControllerBase)
