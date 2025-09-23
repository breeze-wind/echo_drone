#ifndef ROBOT_SERIAL_OPENMV_SERIAL_H
#define ROBOT_SERIAL_OPENMV_SERIAL_H

#include <rclcpp/rclcpp.hpp>
#include "sentry_msg.h"
#include "robot_message.h"
#include "robot_referee.h"
#include "robot_openmv.h"

class OpenmvSerial : public rclcpp::Node {
private:
    openmv::OpenmvcamSerial openmvSerial;

    rclcpp::Publisher<robot_interfaces::msg::OpenmvInfo>::SharedPtr ImagelocationPublisher;

public:
    OpenmvSerial() : Node("openmv_serial_node") {
        declare_parameter("/serial_name_openmv", "/dev/openmv_serial");

        openmvSerial = std::move(openmv::OpenmvcamSerial(get_parameter("/serial_name_openmv").as_string(), 115200));

        RCLCPP_INFO(this->get_logger(),"openmv_serial init success");

        openmvSerial.registerCallback(0xaa, [this](const openmv_info_t& msg){
            robot_interfaces::msg::OpenmvInfo _OpenmvInfo;
            _OpenmvInfo.accurate = msg.accurate;
            _OpenmvInfo.image_x = msg.image_x;
            _OpenmvInfo.image_y = msg.image_y;
            //std::cout << "Image x: " << (int)_OpenmvInfo.image_x << " Image y: " << (int)_OpenmvInfo.image_y << std::endl;
            RCLCPP_INFO(get_logger(), "image location: X: %f Y: %f", msg.image_x, msg.image_y);
            ImagelocationPublisher->publish(_OpenmvInfo);
        });

        ImagelocationPublisher = create_publisher<robot_interfaces::msg::OpenmvInfo>("/robot/openmv_info", 1);

        openmvSerial.spin(true);
    }
};

#endif //ROBOT_SERIAL_OPENMV_SERIAL_H
