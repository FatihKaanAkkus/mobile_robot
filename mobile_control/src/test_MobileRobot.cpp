#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <mobile_control/mobile_control.h>

int main(int argc, char **argv)
{
    // Setup
    ros::init(argc, argv, "MobileRobot");

    mobile_control::MobileRobot robot;
    controller_manager::ControllerManager cm(&robot);

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0);

    while(ros::ok())
    {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;

        std::cout<<time<<"\n";

        robot.read();
        cm.update(time, period);
        robot.write();

        rate.sleep();
    }

    return 0;
}