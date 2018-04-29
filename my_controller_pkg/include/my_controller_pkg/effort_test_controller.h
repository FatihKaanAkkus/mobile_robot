#ifndef MY_CONTROLLER_PKG_EFFORT_TEST_CONTROLLER_H
#define MY_CONTROLLER_PKG_EFFORT_TEST_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

namespace my_controller_pkg
{
    class EffortTestController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        EffortTestController(){}

        using controller_interface::Controller<hardware_interface::EffortJointInterface>::init;
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& /*n*/);
        void starting(const ros::Time& /*time*/);
        void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);
        void stopping(const ros::Time& /*time*/);

    private:
        std::vector<hardware_interface::JointHandle> joint_effort_commands_;

    };
}

#endif