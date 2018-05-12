#include <my_controller_pkg/my_robot_hw.h>

namespace my_controller_pkg
{

    MyRobotHW::MyRobotHW()
    {
        using namespace hardware_interface;

        // Initialize raw data
        joint_position_.resize(9);
        joint_velocity_.resize(9);
        joint_effort_.resize(9);
        joint_effort_command_.resize(9);
        joint_velocity_command_.resize(9);
        joint_name_.resize(9);

        joint_name_[0] = "wljf";
        joint_position_[0] = 0.0;
        joint_velocity_[0] = 0.0;
        joint_effort_[0] = 0.0;
        joint_effort_command_[0] = 0.0;
        joint_velocity_command_[0] = 0.0;

        joint_name_[1] = "wljb";
        joint_position_[1] = 0.0;
        joint_velocity_[1] = 0.0;
        joint_effort_[1] = 0.0;
        joint_effort_command_[1] = 0.0;
        joint_velocity_command_[1] = 0.0;

        joint_name_[2] = "wrjf";
        joint_position_[2] = 0.0;
        joint_velocity_[2] = 0.0;
        joint_effort_[2] = 0.0;
        joint_effort_command_[2] = 0.0;
        joint_velocity_command_[2] = 0.0;

        joint_name_[3] = "wrjb";
        joint_position_[3] = 0.0;
        joint_velocity_[3] = 0.0;
        joint_effort_[3] = 0.0;
        joint_effort_command_[3] = 0.0;
        joint_velocity_command_[3] = 0.0;

        joint_name_[4] = "m_joint_1";
        joint_position_[4] = 0.0;
        joint_velocity_[4] = 0.0;
        joint_effort_[4] = 0.0;
        joint_effort_command_[4] = 0.0;
        joint_velocity_command_[4] = 0.0;

        joint_name_[5] = "m_joint_2";
        joint_position_[5] = 0.0;
        joint_velocity_[5] = 0.0;
        joint_effort_[5] = 0.0;
        joint_effort_command_[5] = 0.0;
        joint_velocity_command_[5] = 0.0;

        joint_name_[6] = "m_joint_3";
        joint_position_[6] = 0.0;
        joint_velocity_[6] = 0.0;
        joint_effort_[6] = 0.0;
        joint_effort_command_[6] = 0.0;
        joint_velocity_command_[6] = 0.0;

        joint_name_[7] = "m_joint_4";
        joint_position_[7] = 0.0;
        joint_velocity_[7] = 0.0;
        joint_effort_[7] = 0.0;
        joint_effort_command_[7] = 0.0;
        joint_velocity_command_[7] = 0.0;

        joint_name_[8] = "m_joint_5";
        joint_position_[8] = 0.0;
        joint_velocity_[8] = 0.0;
        joint_effort_[8] = 0.0;
        joint_effort_command_[8] = 0.0;
        joint_velocity_command_[8] = 0.0;

        // Populate hardware interfaces
        js_interface_.registerHandle(JointStateHandle(joint_name_[0], &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]));
        js_interface_.registerHandle(JointStateHandle(joint_name_[1], &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]));
        js_interface_.registerHandle(JointStateHandle(joint_name_[2], &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]));
        js_interface_.registerHandle(JointStateHandle(joint_name_[3], &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]));
        js_interface_.registerHandle(JointStateHandle(joint_name_[4], &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]));
        js_interface_.registerHandle(JointStateHandle(joint_name_[5], &joint_position_[5], &joint_velocity_[5], &joint_effort_[5]));
        js_interface_.registerHandle(JointStateHandle(joint_name_[6], &joint_position_[6], &joint_velocity_[6], &joint_effort_[6]));
        js_interface_.registerHandle(JointStateHandle(joint_name_[7], &joint_position_[7], &joint_velocity_[7], &joint_effort_[7]));
        js_interface_.registerHandle(JointStateHandle(joint_name_[8], &joint_position_[8], &joint_velocity_[8], &joint_effort_[8]));

        ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[0]), &joint_effort_command_[0]));
        ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[1]), &joint_effort_command_[1]));
        ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[2]), &joint_effort_command_[2]));
        ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[3]), &joint_effort_command_[3]));
        ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[4]), &joint_effort_command_[4]));
        ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[5]), &joint_effort_command_[5]));
        ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[6]), &joint_effort_command_[6]));
        ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[7]), &joint_effort_command_[7]));
        ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[8]), &joint_effort_command_[8]));

        vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[0]), &joint_velocity_command_[0]));
        vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[1]), &joint_velocity_command_[1]));
        vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[2]), &joint_velocity_command_[2]));
        vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[3]), &joint_velocity_command_[3]));
        vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[4]), &joint_velocity_command_[4]));
        vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[5]), &joint_velocity_command_[5]));
        vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[6]), &joint_velocity_command_[6]));
        vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[7]), &joint_velocity_command_[7]));
        vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[8]), &joint_velocity_command_[8]));

        registerInterface(&js_interface_);
        registerInterface(&ej_interface_);
        registerInterface(&vj_interface_);
    }


    void MyRobotHW::read()
    {

    }

    void MyRobotHW::write()
    {
    }

}
