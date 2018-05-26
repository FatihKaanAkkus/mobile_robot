/*
 * Author: Fatih Kaan AKKUS
 */

#ifndef MY_CONTROLLER_PKG_SIMPLEARMCONTROLLER_H
#define MY_CONTROLLER_PKG_SIMPLEARMCONTROLLER_H

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <std_msgs/Float64.h>
#include <my_controller_msgs/ControllerState.h>

#include <realtime_tools/realtime_buffer.h>

namespace my_controller_pkg
{
    class SimpleArmController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        /**
         * \brief Constructor
         */
        SimpleArmController();
        /**
         * \brief Destructor
         */
        ~SimpleArmController();

        /**
         * \brief Initialize the controller
         * \param hw Hardware interface
         * \param n Node handler
         * \return true if the initialization is successful
         */
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
        /**
         * \brief Called once when controller starts
         * \param time Current time
         */
        void starting(const ros::Time& time);
        /**
         * \brief Called to update controller states, pretty much every update about controller
         * \param time Current time
         * \param period Update period
         */
        void update(const ros::Time& time, const ros::Duration& period);
        /**
         * \brief Called once when controller stops
         * \param time Current time
         */
        void stopping(const ros::Time& time);

        /// Interface for joint handling
        hardware_interface::JointHandle joint_;
        /// Holds latest command
        realtime_tools::RealtimeBuffer<double> command_buffer_;

    private:
        /// Last time for command timeout
        ros::Time last_cmd_time_;

        /// Subscribers for controller topics
        ros::Subscriber sub_command_;
        ros::Publisher pub_state_;

        /// Subscriber and publisher callbacks
        void commandCB(const std_msgs::Float64ConstPtr& msg);

    };

}

#endif /* MY_CONTROLLER_PKG_SIMPLEARMCONTROLLER_H */
