//
// Created by elsa on 25-1-31.
//

#include "serial_pro/robot_serial.h"

void RobotSerial::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    static uint8_t SOF = 0x00;
    Velocity velocity{
        (float)(msg->linear.x),
        (float)(msg->linear.y),
        (float)(msg->angular.z),
    };
    SOF++;
    droneSerial.write(0x0501, SOF, velocity);
    RCLCPP_INFO(this->get_logger(), "%f %f %f", msg->linear.x, msg->linear.y, msg->angular.z);
}

RobotSerial::RobotSerial() : Node("robot_serial_node")
{
    declare_parameter("/serial_name_drone", "/dev/drone_serial");
    droneSerial = std::move(drone::DroneSerial(get_parameter("/serial_name_drone").as_string(), 115200));

    RCLCPP_INFO(this->get_logger(), "robot_serial init success");

    if (0)
    {
        droneSerial.registerErrorHandle([this](int label, const std::string& text)
        {
            drone::DroneSerial::error _label;
            _label = (drone::DroneSerial::error)label;
            std::stringstream _str;
            for (auto c : text)
            {
                _str << std::setw(2) << std::hex << (int)*(uint8_t*)&c << " ";
            }
            _str << std::endl;
            switch (_label)
            {
            case drone::DroneSerial::lengthNotMatch:
                RCLCPP_ERROR_STREAM(get_logger(), "drone_lengthNotMatch");
                RCLCPP_ERROR_STREAM(get_logger(), _str.str());
            case drone::DroneSerial::rxLessThanLength:
                RCLCPP_ERROR_STREAM(get_logger(), "drone_rxLessThanLength");
                break;
            case drone::DroneSerial::crcError:
                RCLCPP_ERROR_STREAM(get_logger(), "drone_crc8Error");
                RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                break;
            case drone::DroneSerial::crc16Error:
                RCLCPP_ERROR_STREAM(get_logger(), "drone_crc16Error");
                RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                break;
            default:
                return;
            }
        });
    }

    //串口回调函数，接收下位机转发的消息
    VelocitySubscription = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1,
                                                                          std::bind(&RobotSerial::velocityCallback,
                                                                              this,
                                                                              std::placeholders::_1));

    droneSerial.spin(true);
}
