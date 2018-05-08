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

		if(node.getParam("joint", jointName_))
		{	
			if(hw)
			{
				jointHandle_ = hw->getHandle(jointName_);
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
			if(!node.getParam("has_position_limits", jointLimits_.has_position_limits))
			{
				ROS_INFO("Joint: %s >> No 'has_position_limits' is given. Assuming as 'false'.", jointName_.c_str());
				jointLimits_.has_position_limits = false;
			}
			if(jointLimits_.has_position_limits)
			{
				if(!node.getParam("min_position", jointLimits_.min_position)){
					ROS_INFO("Joint: %s >> No 'min_position' is given. Assuming as '0.0'.", jointName_.c_str());
					jointLimits_.min_position = 0.0;
				}
				if(!node.getParam("max_position", jointLimits_.max_position)){
					ROS_INFO("Joint: %s >> No 'max_position' is given. Assuming as '0.0'.", jointName_.c_str());
					jointLimits_.max_position = 0.0;
				}
			}

			// Velocity limits
			if(!node.getParam("has_velocity_limits", jointLimits_.has_velocity_limits))
			{
				ROS_INFO("Joint: %s >> No 'has_velocity_limits' is given. Assuming as 'false'.", jointName_.c_str());
				jointLimits_.has_velocity_limits = false;
			}

			if(jointLimits_.has_velocity_limits)
			{
				if(!node.getParam("max_velocity", jointLimits_.max_velocity)){
					ROS_INFO("Joint: %s >> No 'max_velocity' is given. Assuming as '0.0'.", jointName_.c_str());
					jointLimits_.max_velocity = 0.0;
				}
			}

			// Acceleration limits
			if(!node.getParam("has_acceleration_limits", jointLimits_.has_acceleration_limits))
			{
				ROS_INFO("Joint: %s >> No 'has_acceleration_limits' is given. Assuming as 'false'.", jointName_.c_str());
				jointLimits_.has_acceleration_limits = false;
			}

			if(jointLimits_.has_acceleration_limits)
			{
				if(!node.getParam("max_acceleration", jointLimits_.max_acceleration)){
					ROS_INFO("Joint: %s >> No 'max_acceleration' is given. Assuming as '0.0'.", jointName_.c_str());
					jointLimits_.max_acceleration = 0.0;
				}
			}

			// Jerk limits
			if(!node.getParam("has_jerk_limits", jointLimits_.has_jerk_limits))
			{
				ROS_INFO("Joint: %s >> No 'has_jerk_limits' is given. Assuming as 'false'.", jointName_.c_str());
				jointLimits_.has_jerk_limits = false;
			}

			if(jointLimits_.has_jerk_limits)
			{
				if(!node.getParam("max_jerk", jointLimits_.max_jerk)){
					ROS_INFO("Joint: %s >> No 'max_jerk' is given. Assuming as '0.0'.", jointName_.c_str());
					jointLimits_.max_jerk = 0.0;
				}
			}

			// Effort limits
			if(!node.getParam("has_effort_limits", jointLimits_.has_effort_limits))
			{
				ROS_INFO("Joint: %s >> No 'has_effort_limits' is given. Assuming as 'false'.", jointName_.c_str());
				jointLimits_.has_effort_limits = false;
			}

			if(jointLimits_.has_effort_limits)
			{
				if(!node.getParam("max_effort", jointLimits_.max_effort)){
					ROS_INFO("Joint: %s >> No 'max_effort' is given. Assuming as '0.0'.", jointName_.c_str());
					jointLimits_.max_effort = 0.0;
				}
			}

		}

		// Coefficients
		if(!node.getParam("Kp", P_)){
			ROS_INFO("Joint: %s >> No 'Kp' coefficient is given.", jointName_.c_str());
			P_ = 0.0;
		}

		if(!node.getParam("Ki", I_)){
			ROS_INFO("Joint: %s >> No 'Ki' coefficient is given.", jointName_.c_str());
			I_ = 0.0;
		}

		if(!node.getParam("Kd", D_)){
			ROS_INFO("Joint: %s >> No 'Kd' coefficient is given.", jointName_.c_str());
			D_ = 0.0;
		}

		if(!node.getParam("iMax", iMax_)){
			ROS_INFO("Joint: %s >> No 'iMax' is given for clamping.", jointName_.c_str());
			iMax_ = 0.0;
		}
		
		if(!node.getParam("filter_constant", filter_constant_)){
			ROS_INFO("Joint: %s >> No 'filter_constant' is given.", jointName_.c_str());
			filter_constant_ = 0.0;
		}

		lowPassFilter.setCutOffFrequency(filter_constant_);

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
		command_effort_ = 0.0;
		jointHandle_.setCommand(command_effort_);
		
		current_position_ = jointHandle_.getPosition();
		current_velocity_ = jointHandle_.getVelocity();
		current_effort_ = jointHandle_.getEffort();

		cmd_box_.command_data = 0.0;

		error = 0.0;
		error_old = 0.0;
		error_sum = 0.0;

		last_time = time;
		last_time_filter = time;
	}

	void WheelController::update(const ros::Time& time, const ros::Duration& period)
	{
		// Period
		ros::Duration dt = time - last_time;
		ros::Duration dt_filter = time - last_time_filter;

		my_controller_msgs::WheelControllerCommand command;
		my_controller_msgs::ControllerState state;

		command = cmd_box_;
		
		// Velocity limits
		if(jointLimits_.has_velocity_limits)
		{
			if(command.command_data > jointLimits_.max_velocity) 	command.command_data = jointLimits_.max_velocity;
			if(command.command_data < -jointLimits_.max_velocity) 	command.command_data = -jointLimits_.max_velocity;
		}

		current_velocity_ = lowPassFilter.update(jointHandle_.getVelocity(), dt_filter.toSec());
		
		if(dt.toSec() > 0.001)
		{
			current_position_ = jointHandle_.getPosition();
			current_effort_ = jointHandle_.getEffort();

			// error
			error = ((double) command.command_data) - current_velocity_;
			// integral of error
			error_sum += dt.toSec() * error;
			// limit integral
			if(iMax_ != 0.0){
				if(error_sum > iMax_ / I_) 	error_sum = iMax_ / I_;
				if(error_sum < -iMax_ / I_) 	error_sum = -iMax_ / I_;
			}

			// formula
			double pid = P_ * error + I_ * error_sum + D_ * (error-error_old) / dt.toSec();

			error_old = error;

			command_effort_ = pid;

			// Effort limits
			if(jointLimits_.has_effort_limits)
			{
				if(command_effort_ > jointLimits_.max_effort) 	command_effort_ = jointLimits_.max_effort;
				if(command_effort_ < -jointLimits_.max_effort) 	command_effort_ = -jointLimits_.max_effort;
			}

			jointHandle_.setCommand(command_effort_);

			// Publish to topic
			state.header.stamp = time;
			state.position = current_position_;
			state.velocity = current_velocity_;
			state.effort = current_effort_;
			pub_state.publish(state);

			last_time = time;
		}

		last_time_filter = time;
	}

	void WheelController::stopping(const ros::Time& time)
	{
		const double vel = 0.0;
		jointHandle_.setCommand(vel);
	}

	void WheelController::sub_cmd_Callback(const my_controller_msgs::WheelControllerCommand& msg)
	{
		cmd_box_ = msg;
	}

	void WheelController::sub_pid_Callback(const my_controller_msgs::DynamicPIDParameters& msg)
	{
		P_ = msg.Kp;
		I_ = msg.Ki;
		D_ = msg.Kd;
		iMax_ = msg.iMax;
	}
}

PLUGINLIB_EXPORT_CLASS(my_controller_pkg::WheelController, controller_interface::ControllerBase)
