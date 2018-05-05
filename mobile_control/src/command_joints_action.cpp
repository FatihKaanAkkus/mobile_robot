/*
 * Author: Fatih Kaan AKKUS
 */

#include <mobile_control/command_joints_action.h>

namespace mobile_control
{
    CommandJoints::CommandJoints(ros::NodeHandle &n):
        node_(n),
        action_server_(node_, "command_joints_action",
            boost::bind(&CommandJoints::goal_Callback, this, _1),
            boost::bind(&CommandJoints::cancel_Callback, this, _1),
            false),
        has_active_goal_(false),
        number_of_joints_(5)
    {
        // Get parameters
        node_.param("goal_threshold", goal_threshold_, 0.01);
        node_.param("stall_velocity_threshold", stall_velocity_threshold_, 1e-6);
        node_.param("stall_timeout", stall_timeout_, 0.1);

        // Resize vectors
        pub_controller_command_.resize(number_of_joints_);
        sub_controller_state_.resize(number_of_joints_);
        last_goal_commands_.resize(number_of_joints_);
        last_controller_state_.resize(number_of_joints_);

        // Create publishers
        pub_controller_command_[0] = node_.advertise<my_controller_msgs::ArmControllerCommand>("joint_1_controller/command", 1);
        pub_controller_command_[1] = node_.advertise<my_controller_msgs::ArmControllerCommand>("joint_2_controller/command", 1);
        pub_controller_command_[2] = node_.advertise<my_controller_msgs::ArmControllerCommand>("joint_3_controller/command", 1);
        pub_controller_command_[3] = node_.advertise<my_controller_msgs::ArmControllerCommand>("joint_4_controller/command", 1);
        pub_controller_command_[4] = node_.advertise<my_controller_msgs::ArmControllerCommand>("joint_5_controller/command", 1);

        // Create subscribers
        sub_controller_state_[0] = node_.subscribe("joint_1_controller/state", (uint32_t) 1, &CommandJoints::controllerState_Callback_0, this);
        sub_controller_state_[1] = node_.subscribe("joint_2_controller/state", (uint32_t) 1, &CommandJoints::controllerState_Callback_1, this);
        sub_controller_state_[2] = node_.subscribe("joint_3_controller/state", (uint32_t) 1, &CommandJoints::controllerState_Callback_2, this);
        sub_controller_state_[3] = node_.subscribe("joint_4_controller/state", (uint32_t) 1, &CommandJoints::controllerState_Callback_3, this);
        sub_controller_state_[4] = node_.subscribe("joint_5_controller/state", (uint32_t) 1, &CommandJoints::controllerState_Callback_4, this);

        action_server_.start();
    }

    CommandJoints::~CommandJoints()
    {
        for(unsigned int i=0; i<number_of_joints_; ++i)
        {
            pub_controller_command_[i].shutdown();
            sub_controller_state_[i].shutdown();
        }
        // watchdog_timer_.stop();
    }

    void CommandJoints::watchdog(const ros::TimerEvent &e)
    {
        // ros::Time now = ros::Time::now();

        // // Aborts the active goal if the controller does not appear to be active.
        // if(has_active_goal_)
        // {
        //     bool should_abort = false;
        //     if(!last_controller_state_)
        //     {
        //         should_abort = true;
        //         ROS_WARN("Aborting goal because we have never heard a controller state message.");
        //     }
        //     else if((now - last_controller_state_->header.stamp) > ros::Duration(5.0))
        //     {
        //         should_abort = true;
        //         ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
        //                 (now - last_controller_state_->header.stamp).toSec());
        //     }

        //     if(should_abort)
        //     {
        //         // Marks the current goal as aborted.
        //         active_goal_.setAborted();
        //         has_active_goal_ = false;
        //     }
        // }
    }

    void CommandJoints::goal_Callback(GoalHandle gh)
    {
        if(has_active_goal_)
        {
            active_goal_.setCanceled();
            has_active_goal_ = false;
        }

        gh.setAccepted();
        active_goal_ = gh;
        has_active_goal_ = true;
        goal_received_ = ros::Time::now();
        min_error_seen_ = 1e10;

        for(unsigned int i=0; i<number_of_joints_; ++i)
        {
            last_goal_commands_[i].command_data = active_goal_.getGoal()->joint_position[i];
            pub_controller_command_[i].publish(last_goal_commands_[i]);
        }

        last_movement_time_ = ros::Time::now();
    }

    void CommandJoints::cancel_Callback(GoalHandle gh)
    {
        if(active_goal_ == gh)
        {
            for(unsigned int i=0; i<number_of_joints_; ++i)
            {
                if(last_controller_state_[i])
                {
                    my_controller_msgs::ArmControllerCommand stop;
                    stop.command_data = last_controller_state_[i]->position;
                    pub_controller_command_[i].publish(stop);
                }
            }
            active_goal_.setCanceled();
            has_active_goal_ = false;
        }
    }

    void CommandJoints::actionStateCallback(void)
    {
        ros::Time now = ros::Time::now();

        if(!has_active_goal_)
            return;
        
        // Last positions
        std::vector<double> positions;
        std::vector<double> position_errors;
        for(unsigned int i=0; i<number_of_joints_; ++i)
        {
            positions.push_back(last_controller_state_[i]->position);
            position_errors.push_back(last_goal_commands_[i].command_data - last_controller_state_[i]->position);
        }

        // Feedback variables
        my_controller_msgs::JointPositionsFeedback feedback;
        feedback.joint_position = positions;
        feedback.joint_position_error = position_errors;

        active_goal_.publishFeedback(feedback);
    }
    
    void CommandJoints::controllerState_Callback_0(const my_controller_msgs::ControllerStateConstPtr& msg)
    {
        last_controller_state_[0] = msg;
        actionStateCallback();
    }
    
    void CommandJoints::controllerState_Callback_1(const my_controller_msgs::ControllerStateConstPtr& msg)
    {
        last_controller_state_[1] = msg;
        actionStateCallback();
    }
    
    void CommandJoints::controllerState_Callback_2(const my_controller_msgs::ControllerStateConstPtr& msg)
    {
        last_controller_state_[2] = msg;
        actionStateCallback();
    }
    
    void CommandJoints::controllerState_Callback_3(const my_controller_msgs::ControllerStateConstPtr& msg)
    {
        last_controller_state_[3] = msg;
        actionStateCallback();
    }
    
    void CommandJoints::controllerState_Callback_4(const my_controller_msgs::ControllerStateConstPtr& msg)
    {
        last_controller_state_[4] = msg;
        actionStateCallback();
    }
}