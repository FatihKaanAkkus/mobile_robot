#ifndef MY_CONTROLLER_PKG_WHEELCONTROLLER_H
#define MY_CONTROLLER_PKG_WHEELCONTROLLER_H

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <my_controller_msgs/ControllerState.h>
#include <my_controller_msgs/WheelControllerCommand.h>
#include <my_controller_msgs/DynamicPIDParameters.h>

#include <my_filter_pkg/LowPassFilter.h>

namespace my_controller_pkg
{
    // Limit params from controller parameters
    typedef struct MyJointLimits_struct {
        bool has_position_limits;
        bool has_velocity_limits;
        bool has_acceleration_limits;
        bool has_jerk_limits;
        bool has_effort_limits;
        double min_position;
        double max_position;
        double max_velocity;
        double max_acceleration;
        double max_jerk;
        double max_effort;
	} MyJointLimits;

    class WheelController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        WheelController();
        virtual ~WheelController();

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);

        my_controller_msgs::WheelControllerCommand cmd_box;

    private:
		std::string jointName;
        hardware_interface::JointHandle jointHandle;

        MyJointLimits myJointLimits;

        double buffer_command_effort;
        double buffer_current_position;
        double buffer_current_velocity;
        double buffer_current_effort;

        double coeff_Kp;
        double coeff_Ki;
        double coeff_Kd;
        double clamp_iMax;

        double error;
        double error_old;
        double error_sum;

        my_filter_pkg::LowPassFilter lowPassFilter;
        double filter_constant;
        
        ros::NodeHandle node;
        ros::Subscriber sub_cmd;
        ros::Subscriber sub_pid;

        ros::Publisher pub_state;

        ros::Time last_time;
        ros::Time last_time_filter;

        void sub_cmd_Callback(const my_controller_msgs::WheelControllerCommand& msg);
        void sub_pid_Callback(const my_controller_msgs::DynamicPIDParameters& msg);
    };
}

#endif
