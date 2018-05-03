#ifndef MY_CONTROLLER_PKG_MOBILECONTROLLER_H
#define MY_CONTROLLER_PKG_MOBILECONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <my_controller_pkg/Odometry.h>
#include <my_controller_pkg/SpeedLimiter.h>
#include <my_controller_pkg/MobileController.h>

namespace my_controller_pkg 
{
    class MobileController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
    {
        public: 
            MobileController();
            virtual ~MobileController();

            bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
            void starting(const ros::Time& time);
            void update(const ros::Time& time, const ros::Duration& period);
            void stopping(const ros::Time& time);

        private:
            std::string name_;

            // Odometry
            ros::Duration publish_period_;
            ros::Time last_state_publish_time_;

            // Hardware
            size_t wheel_joints_size_;
            std::vector<hardware_interface::JointHandle> left_wheel_joints_;
            std::vector<hardware_interface::JointHandle> right_wheel_joints_;

            // Velocity commands
            struct Commands {
                double lin;
                double ang;
                ros::Time stamp;
                Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
            };
            Commands command_;
            Commands command_struct_;

            // Subscriber to command topic
            ros::Subscriber sub_cmd_;

            // Publisher for executed command
            geometry_msgs::TwistStamped cmd_vel_pub_msg;
            ros::Publisher cmd_vel_pub_;

            // Publisher for odom
            // Publisher for odom to tf
            boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

            // Odometry
            Odometry odometry_;
            double publish_rate_;
            bool open_loop_;

            // Parameters
            double wheel_separation_;
            double wheel_radius_;
            double cmd_vel_timeout_;

            // Frames to use
            std::string base_frame_id_;
            std::string odom_frame_id_;
            
            // Publish odometry to tf or not
            bool enable_odom_tf_;
            
            // Speed limiters
            Commands last1_cmd_;
            Commands last0_cmd_;
            SpeedLimiter limiter_lin_;
            SpeedLimiter limiter_ang_;

            // Publish limited velocity or not
            bool publish_cmd_;

            // Braking function
            void brake();
            void cmd_vel_Callback(const geometry_msgs::Twist& command);
            
            // Sets the odometry publishing fields
            void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    };
};

#endif
