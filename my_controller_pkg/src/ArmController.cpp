#include <my_controller_pkg/ArmController.h>

template<typename T>
T clamp(T x, T min, T max)
{
  return std::min(std::max(min, x), max);
}

namespace my_controller_pkg
{
	ArmController::ArmController()
	{}

	ArmController::~ArmController()
	{
		sub_cmd.shutdown();
	}

	bool ArmController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n)
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
				if(!node.getParam("min_velocity", myJointLimits.min_velocity)){
					ROS_INFO("Joint: %s >> No 'min_velocity' is given. Assuming as '0.0'.", jointName.c_str());
					myJointLimits.min_velocity = 0.0;
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
				if(!node.getParam("min_acceleration", myJointLimits.min_acceleration)){
					ROS_INFO("Joint: %s >> No 'min_acceleration' is given. Assuming as '0.0'.", jointName.c_str());
					myJointLimits.min_acceleration = 0.0;
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
				if(!node.getParam("min_jerk", myJointLimits.min_jerk)){
					ROS_INFO("Joint: %s >> No 'min_jerk' is given. Assuming as '0.0'.", jointName.c_str());
					myJointLimits.min_jerk = 0.0;
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
		
		if(!node.getParam("filter_constant", filter_constant)){
			ROS_INFO("Joint: %s >> No 'filter_constant' is given.", jointName.c_str());
			filter_constant = 0.0;
		}

		// Limiter parameters
        limiter.has_velocity_limits = myJointLimits.has_velocity_limits;
        limiter.has_acceleration_limits = myJointLimits.has_acceleration_limits;
        limiter.has_jerk_limits = myJointLimits.has_jerk_limits;
        limiter.min_velocity = myJointLimits.min_velocity;
        limiter.max_velocity = myJointLimits.max_velocity;
        limiter.min_acceleration = myJointLimits.min_acceleration;
        limiter.max_acceleration = myJointLimits.max_acceleration;
        limiter.min_jerk = myJointLimits.min_jerk;
        limiter.max_jerk = myJointLimits.max_jerk;

		lowPassFilter.setCutOffFrequency(filter_constant);

		// Subscribe to command topic 
		const std::string sub_cmd_topic("command");
		sub_cmd = node.subscribe(sub_cmd_topic, (uint32_t) 1, &ArmController::sub_cmd_Callback, this); 

		// Subscribe to pid topic 
		const std::string sub_pid_topic("dynamic_pid");
		sub_pid = node.subscribe(sub_pid_topic, (uint32_t) 1, &ArmController::sub_pid_Callback, this); 

		// Publish current state
		const std::string pub_topic("state");
		pub_state = node.advertise<my_controller_msgs::ControllerState>(pub_topic, (uint32_t) 1);

		return true;
	} 

	void ArmController::starting(const ros::Time& time)
	{
		cmd_box.command_data = 0.0;
		command_effort_ = -0.0001;

		current_position_ = jointHandle.getPosition();
		current_velocity_ = jointHandle.getVelocity();
		current_effort_ = jointHandle.getEffort();

		error = 0.0;
		error_old = 0.0;
		error_sum = 0.0;

		last_time = time;
	}

	void ArmController::update(const ros::Time& time, const ros::Duration& period)
	{
		// Period
		ros::Duration dt = time - last_time;

		my_controller_msgs::ArmControllerCommand command;
		my_controller_msgs::ControllerState state;

		command = cmd_box;
		
		// Position limits
		if(myJointLimits.has_position_limits)
		{
			if(command.command_data > myJointLimits.max_position) 	command.command_data = myJointLimits.max_position;
			if(command.command_data < myJointLimits.min_position) 	command.command_data = myJointLimits.min_position;
		}

		double set_point = command.command_data;

		current_position_ = lowPassFilter.update(jointHandle.getPosition(), filter_constant);
		current_velocity_ = jointHandle.getVelocity();
		current_effort_ = jointHandle.getEffort();

		if(myJointLimits.has_acceleration_limits)
		{
			const double dp = set_point - last_pos0;
			const double dp0 = last_pos0 - last_pos1;

			const double dt2 = period.toSec() * period.toSec();

			const double da_min = myJointLimits.min_acceleration * dt2;
			const double da_max = myJointLimits.max_acceleration * dt2;

			const double da = clamp(dp - dp0, da_min, da_max); 
			
			set_point = last_pos0 + dp0 + da;
		}

		if(myJointLimits.has_velocity_limits)
		{
			const double dp_min = myJointLimits.min_velocity * period.toSec();
			const double dp_max = myJointLimits.max_velocity * period.toSec();

			const double dp =  clamp(set_point - last_pos0, dp_min, dp_max);

			set_point = last_pos0 + dp;
		}

		last_pos1 = last_pos0;
		last_pos0 = set_point;

		// error
		error = set_point - current_position_;
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

		command_effort_ = pid;

		// Effort limits
		if(myJointLimits.has_effort_limits)
		{
			if(command_effort_ > myJointLimits.max_effort) 	command_effort_ = myJointLimits.max_effort;
			if(command_effort_ < -myJointLimits.max_effort) 	command_effort_ = -myJointLimits.max_effort;
		}

		jointHandle.setCommand(command_effort_);

		// Publish to topic
		state.header.stamp = time;
		state.position = current_position_;
		state.velocity = current_velocity_;
		state.effort = current_effort_;
		pub_state.publish(state);

		last_time = time;
	}

	void ArmController::stopping(const ros::Time& time)
	{
		const double vel = 0.0;
		jointHandle.setCommand(vel);
	}

	void ArmController::sub_cmd_Callback(const my_controller_msgs::ArmControllerCommand& msg)
	{
		cmd_box = msg;
	}

	void ArmController::sub_pid_Callback(const my_controller_msgs::DynamicPIDParameters& msg)
	{
		coeff_Kp = msg.Kp;
		coeff_Ki = msg.Ki;
		coeff_Kd = msg.Kd;
		clamp_iMax = msg.clamp_iMax;
	}
}

PLUGINLIB_EXPORT_CLASS(my_controller_pkg::ArmController, controller_interface::ControllerBase)
