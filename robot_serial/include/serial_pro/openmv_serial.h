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

    rclcpp::Publisher<robot_interfaces::msg::Imagelocation>::SharedPtr ImagelocationPublisher;

public:
    OpenmvSerial() : Node("openmv_serial_node") {
        declare_parameter("/serial_name_openmv", "/dev/openmv_serial");

        openmvSerial = std::move(openmv::OpenmvcamSerial(get_parameter("/serial_name_openmv").as_string(), 115200));

        RCLCPP_INFO(this->get_logger(),"openmv_serial init success");

        openmvSerial.registerCallback(0xaa, [this](const image_location_t& msg){
            robot_interfaces::msg::Imagelocation _Imagelocation;
            _Imagelocation.image_x = msg.image_x;
            _Imagelocation.image_y = msg.image_y;
            //std::cout << "Image x: " << (int)_Imagelocation.image_x << " Image y: " << (int)_Imagelocation.image_y << std::endl;
            RCLCPP_INFO(get_logger(), "image location: X: %d Y: %d", msg.image_x, msg.image_y);
            ImagelocationPublisher->publish(_Imagelocation);
        });

        ImagelocationPublisher = create_publisher<robot_interfaces::msg::Imagelocation>("/robot/image_location", 1);

        openmvSerial.spin(true);
    }
};

#endif //ROBOT_SERIAL_OPENMV_SERIAL_H
