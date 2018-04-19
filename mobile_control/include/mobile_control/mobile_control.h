#ifndef MOBILEROBOT_H
#define MOBILEROBOT_H

#include <hardware_interface/robot_hw.h>

namespace mobile_control
{
    class MobileRobot :
        public hardware_interface::RobotHW
    {
        public:
            MobileRobot();

            void read();
            void write();
    };

}
#endif
