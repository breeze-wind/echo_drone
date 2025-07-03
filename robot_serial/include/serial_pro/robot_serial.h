//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_SERIAL_H
#define ROBOT_SERIAL_ROBOT_SERIAL_H

#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "drone_msg.h"
#include "robot_message.h"
#include "robot_referee.h"
#include "robot_drone.h"

class RobotSerial : public rclcpp::Node {
private:
    drone::DroneSerial droneSerial; //串口
    rclcpp::Clock rosClock;

    // 初始化发布者和订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr VelocitySubscription;

    // ros回调函数
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

public:
    explicit RobotSerial();
};
 
#endif //ROBOT_SERIAL_ROBOT_SERIAL_H
