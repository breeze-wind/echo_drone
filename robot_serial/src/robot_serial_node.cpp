//
// Created by mijiao on 23-11-20.
//

#include "serial_pro/robot_serial.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotSerial>());
    rclcpp::shutdown();
    return 0;
}
