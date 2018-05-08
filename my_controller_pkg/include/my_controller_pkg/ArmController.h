/*
 * Author: Fatih Kaan AKKUS
 */

#ifndef MY_CONTROLLER_PKG_ARMCONTROLLER_H
#define MY_CONTROLLER_PKG_ARMCONTROLLER_H

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <my_controller_msgs/ControllerState.h>
#include <my_controller_msgs/ArmControllerCommand.h>
#include <my_controller_msgs/DynamicPIDParameters.h>

#include <my_filter_pkg/LowPassFilter.h>
#include <my_controller_pkg/SpeedLimiter.h>

namespace my_controller_pkg
{
    class ArmController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        /**
         * \brief Constructor
         */
        ArmController();
        /**
         * \brief Destructor
         */
        virtual ~ArmController();

        /**
         * \brief Initialize the controller
         * \param hw Hardware interface
         * \param n Node handler
         * \return true if the initialization is successful
         */
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& n);
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

        /// Holds latest command
        my_controller_msgs::ArmControllerCommand cmd_box_;

    private:
        /// Holds joint's name
        std::string jointName_;
        /// Interface for joint handling
        hardware_interface::JointHandle jointHandle_;

        /**
         * \struct MyJointLimits
         * \brief Structure for joint limits
         * \var MyJointLimits::has_position_limits
         * If true, applies velocity limits
         * \var MyJointLimits::has_velocity_limits
         * If true, applies acceleration limits
         * \var MyJointLimits::has_acceleration_limits
         * If true, applies acceleration limits
         * \var MyJointLimits::has_jerk_limits
         * If true, applies jerk limits
         * \var MyJointLimits::has_effort_limits
         * If true applies effort limits
         * \var MyJointLimits::min_position
         * Minimum position [rad], usually <= 0
         * \var MyJointLimits::max_position
         * Maximum position [rad], usually => 0
         * \var MyJointLimits::min_velocity
         * Minimum velocity [m/s], usually <= 0
         * \var MyJointLimits::max_velocity
         * Maximum velocity [m/s], usually => 0
         * \var MyJointLimits::min_acceleration
         * Minimum acceleration [m/s^2], usually <= 0
         * \var MyJointLimits::max_acceleration
         * Maximum acceleration [m/s^2], usually => 0
         * \var MyJointLimits::min_jerk
         * Minimum jerk [m/s^3], usually <= 0
         * \var MyJointLimits::max_jerk
         * Maximum jerk [m/s^3], usually => 0
         * \var MyJointLimits::max_effort
         * Maximum effort applied to joints, usually => 0
         */
        typedef struct {
            bool has_position_limits;
            bool has_velocity_limits;
            bool has_acceleration_limits;
            bool has_jerk_limits;
            bool has_effort_limits;
            double min_position;
            double max_position;
            double min_velocity;
            double max_velocity;
            double min_acceleration;
            double max_acceleration;
            double min_jerk;
            double max_jerk;
            double max_effort;
        } MyJointLimits;

        /// Joint limits
        MyJointLimits jointLimits_;

        /// Current joint states
        double command_effort_;   // [voltage]
        double current_position_; // [m]
        double current_velocity_; // [m/s]
        double current_effort_;   // [m/s^2]

        /// PID controller coefficients
        double P_;
        double I_;
        double D_;
        double iMax_;

        /// PID controller variables used for calculation
        double error;
        double error_old;
        double error_sum;

        /// Limiter
        double last_pos1;
        double last_pos0;

        /// Low pass filter to inputs
        my_filter_pkg::LowPassFilter lowPassFilter;
        /// Filter cut-off frequency
        double filter_constant_; // [Hz]
        
        /// Node handler for controller
        ros::NodeHandle node;
        /// Subscribers for controller topics
        ros::Subscriber sub_cmd;
        ros::Subscriber sub_pid;

        /// Publishers for controller topics
        ros::Publisher pub_state;

        /// Last used time in calculations
        ros::Time last_time;
        ros::Time last_time_filter;

        /// Subscriber and publisher callbacks
        void sub_cmd_Callback(const my_controller_msgs::ArmControllerCommand& msg);
        void sub_pid_Callback(const my_controller_msgs::DynamicPIDParameters& msg);
    };
}

#endif /* MY_CONTROLLER_PKG_ARMCONTROLLER_H */
