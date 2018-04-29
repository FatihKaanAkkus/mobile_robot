#ifndef MY_CONTROLLER_PKG_SIMPLE_CONTROLLER_HPP
#define MY_CONTROLLER_PKG_SIMPLE_CONTROLLER_HPP

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <my_controller_msgs/SimpleControllerCommand.h>

namespace my_controller_pkg
{
    class SimpleController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        SimpleController();
        virtual ~SimpleController();

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);

        my_controller_msgs::SimpleControllerCommandPtr cmd_box_;

    private:
        std::vector<hardware_interface::JointHandle> effortJointHandles;
        std::vector<double> buffer_command_effort;
        std::vector<double> buffer_current_positions;
        std::vector<double> buffer_current_velocities;
        std::vector<double> buffer_current_efforts;
        
        ros::NodeHandle node_;
        ros::Subscriber sub_cmd_;

        ros::Time last_time_;

        void sub_cmd_Callback(const my_controller_msgs::SimpleControllerCommandPtr& msg);
    };
}

#endif
