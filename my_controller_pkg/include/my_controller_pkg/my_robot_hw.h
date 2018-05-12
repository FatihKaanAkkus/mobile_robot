/*
 * CURRENTLY NOT USING 
 */

#ifndef MY_CONTROLLER_PKG_MY_ROBOT_HW_H
#define MY_CONTROLLER_PKG_MY_ROBOT_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

namespace my_controller_pkg
{
    class MyRobotHW : public hardware_interface::RobotHW
    {
        public:
            /**
             * \brief Constructor
             */
            MyRobotHW();

            /**
             * \brief Read from hardware
             */
            void read();
            /**
             * \brief Read to hardware
             */
            void write();

        protected:

        private:
            /// Interface vector for states
            hardware_interface::JointStateInterface    js_interface_;
            /// Interface vector for effort joints
            hardware_interface::EffortJointInterface   ej_interface_;
            /// Interface vector for velocity joints
            hardware_interface::VelocityJointInterface vj_interface_;

            /// Joints related
            std::vector<double> joint_effort_command_;
            std::vector<double> joint_velocity_command_;
            std::vector<double> joint_position_;
            std::vector<double> joint_velocity_;
            std::vector<double> joint_effort_;
            std::vector<std::string> joint_name_;
    };
}

#endif
