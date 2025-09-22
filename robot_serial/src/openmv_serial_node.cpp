//
// Created by mijiao on 23-11-20.
//

#include "serial_pro/openmv_serial.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpenmvSerial>());
    rclcpp::shutdown();
    return 0;
}
