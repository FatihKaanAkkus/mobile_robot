#include <my_controller_pkg/SimpleArmController.h>

namespace my_controller_pkg
{
    SimpleArmController::SimpleArmController()
    {}

    SimpleArmController::~SimpleArmController()
    {
        sub_command_.shutdown();
    }

    bool SimpleArmController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
    {
        std::string joint_name;
        
        if (!n.getParam("joint", joint_name))
        {
            ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
            return false;
        }

        joint_ = hw->getHandle(joint_name);

        // Subscribe to commands
        sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &SimpleArmController::commandCB, this);

        // Publish state
        const std::string pub_topic("state");
        pub_state_ = n.advertise<my_controller_msgs::ControllerState>(pub_topic, (uint32_t) 1);

        return true;
    }

    void SimpleArmController::starting(const ros::Time& time)
    {}

    void SimpleArmController::update(const ros::Time& time, const ros::Duration& period)
    {
        my_controller_msgs::ControllerState state;     

        // Check if cmd_vel timed out
        if((time - last_cmd_time_).toSec() > 1.0)
        {
            command_buffer_.writeFromNonRT(0.0);
        }

        joint_.setCommand(*command_buffer_.readFromRT());

        state.header.stamp = time;
        state.position = joint_.getPosition();
        state.velocity = joint_.getVelocity();
        state.effort = joint_.getEffort();
        pub_state_.publish(state);
    }

    void SimpleArmController::stopping(const ros::Time& time)
    {}

    void SimpleArmController::commandCB(const std_msgs::Float64ConstPtr& msg)
    {
        command_buffer_.writeFromNonRT(msg->data);
        last_cmd_time_ = ros::Time::now();
    }

}

PLUGINLIB_EXPORT_CLASS(my_controller_pkg::SimpleArmController, controller_interface::ControllerBase)
