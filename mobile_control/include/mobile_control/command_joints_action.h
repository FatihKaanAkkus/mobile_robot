/*
 * Author: Fatih Kaan AKKUS
 */

#include <ros/ros.h>

#include <actionlib/server/action_server.h>
#include <my_controller_msgs/ArmControllerCommand.h>
#include <my_controller_msgs/ControllerState.h>
#include <my_controller_msgs/JointPositionsAction.h>

namespace mobile_control
{
    class CommandJoints
    {
    public:
        /**
         * \brief Constructor
         */
        CommandJoints(ros::NodeHandle &n);
        /**
         * \brief Destructor
         */
        ~CommandJoints();

    private:
        /// Action server related type definitions
        typedef actionlib::ActionServer<my_controller_msgs::JointPositionsAction> JPAS;
        typedef JPAS::GoalHandle GoalHandle;

        /// Joints related
        int number_of_joints_;
        /// Node handler
        ros::NodeHandle node_;
        /// Action server
        JPAS action_server_;
        /// Command publisher
        std::vector<ros::Publisher> pub_controller_command_;
        /// State subscriber
        std::vector<ros::Subscriber> sub_controller_state_;
        /// Watchdog timer for action
        ros::Timer watchdog_timer_;

        /// Goal related
        bool has_active_goal_;
        GoalHandle active_goal_;
        ros::Time goal_received_;
        std::vector<my_controller_msgs::ArmControllerCommand> last_goal_commands_;

        /// Current state related
        double min_error_seen_;
        double goal_threshold_;
        double stall_velocity_threshold_;
        double stall_timeout_;
        ros::Time last_movement_time_;

        void watchdog(const ros::TimerEvent &e);
        /// Goal command callback
        void goal_Callback(GoalHandle gh);
        /// Cancel command callback
        void cancel_Callback(GoalHandle gh);

        /// Current controller state related
        std::vector<my_controller_msgs::ControllerStateConstPtr> last_controller_state_;

        void actionStateCallback(void);

        /// Controller state callback functions for each joint controller
        void controllerState_Callback_0(const my_controller_msgs::ControllerStateConstPtr& msg);
        void controllerState_Callback_1(const my_controller_msgs::ControllerStateConstPtr& msg);
        void controllerState_Callback_2(const my_controller_msgs::ControllerStateConstPtr& msg);
        void controllerState_Callback_3(const my_controller_msgs::ControllerStateConstPtr& msg);
        void controllerState_Callback_4(const my_controller_msgs::ControllerStateConstPtr& msg);
    };
}
