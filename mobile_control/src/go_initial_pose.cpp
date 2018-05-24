#include <ros/ros.h>
#include <my_controller_msgs/JointPositionsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sstream>
#include <iostream>

typedef actionlib::SimpleActionClient<my_controller_msgs::JointPositionsAction> JointPositionsClient;

class Commander
{
private:
    JointPositionsClient* command_client_;  
    std::vector<double> positions_;

public:
    Commander()
    {
        positions_.resize(5);

        command_client_ = new JointPositionsClient("mobile/command_joints", true);

        while(!command_client_->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
        }
    }

    ~Commander()
    {
        delete command_client_;
    }

    void posOne()
    {
        my_controller_msgs::JointPositionsGoal posOneGoal;
        positions_[0] = 3.14;
        positions_[1] = 1.567;
        positions_[2] = -2.2;
        positions_[3] = -1.57;
        positions_[4] = 0.0;

        posOneGoal.joint_position = positions_;
        
        ROS_INFO("Sending initial_pose goal");
        command_client_->sendGoal(posOneGoal);
        command_client_->waitForResult();
        if(command_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The initial_pose succeded!");
        else
            ROS_INFO("The initial_pose failed.");
    }

    void posTwo()
    {
        my_controller_msgs::JointPositionsGoal posTwoGoal;
        positions_[0] = 0.0;
        positions_[1] = -1.567;
        positions_[2] = 2.2;
        positions_[3] = 1.57;
        positions_[4] = 3.14;

        posTwoGoal.joint_position = positions_;
        
        ROS_INFO("Sending second_test_pose goal");
        command_client_->sendGoal(posTwoGoal);
        command_client_->waitForResult();
        if(command_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The second_test_pose succeded!");
        else
            ROS_INFO("The second_test_pose failed.");
    }

    void posThree()
    {
        my_controller_msgs::JointPositionsGoal posThreeGoal;
        positions_[0] = 1.57;
        positions_[1] = 0.1;
        positions_[2] = 0.5;
        positions_[3] = 0.57;
        positions_[4] = 1.0;

        posThreeGoal.joint_position = positions_;
        
        ROS_INFO("Sending third_test_pose goal");
        command_client_->sendGoal(posThreeGoal);
        command_client_->waitForResult();
        if(command_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The third_test_pose succeded!");
        else
            ROS_INFO("The third_test_pose failed.");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initial_pos_goals");

    Commander commander;

    ros::Duration(0.5).sleep();
    commander.posOne();
    ros::Duration(0.5).sleep();
    // commander.posTwo();
    // ros::Duration(0.5).sleep();
    // commander.posThree();
    // ros::Duration(0.5).sleep();
    // commander.posOne();
    // ros::Duration(0.5).sleep();

    return 0;
}