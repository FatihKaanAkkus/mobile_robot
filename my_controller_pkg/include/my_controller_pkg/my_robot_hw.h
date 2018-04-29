#ifndef MY_CONTROLLER_PKG_MY_ROBOT_HW_H
#define MY_CONTROLLER_PKG_MY_ROBOT_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

namespace my_controller_pkg
{
	class MyRobotHW : public hardware_interface::RobotHW
	{
		public:
			MyRobotHW();

			void read();
			void write();

		protected:

		private:
			hardware_interface::JointStateInterface    js_interface_;
			hardware_interface::EffortJointInterface   ej_interface_;
			hardware_interface::VelocityJointInterface vj_interface_;

			std::vector<double> joint_effort_command_;
			std::vector<double> joint_velocity_command_;
			std::vector<double> joint_position_;
			std::vector<double> joint_velocity_;
			std::vector<double> joint_effort_;
			std::vector<std::string> joint_name_;
	};
}

#endif
