//
// Created by elsa on 25-7-4.
//

#ifndef BEHAVIOR_CONTROL_HPP
#define BEHAVIOR_CONTROL_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <utility>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class BehaviorControl : public rclcpp::Node
{
public:
    explicit BehaviorControl(std::string name);
    void step_timer_callback();
    void mission_timer_callback();

private:
    void CurrentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;

    rclcpp::TimerBase::SharedPtr step_timer_;
    rclcpp::TimerBase::SharedPtr mission_timer_;
    std::chrono::seconds step_period_ms;
    std::chrono::seconds mission_period_ms;

    std::vector<double> tank_;
    std::vector<double> tent_;
    std::vector<double> car_;
    std::vector<double> pillbox_;
    std::vector<double> passing_door_src_;
    std::vector<double> passing_door_des_;

    std::map<std::string, std::vector<double>> target_positions_;
    std::map<std::string, bool> if_hit_target_;

    std::vector<std::string> target_sequence_;

    double cruise_height_, detection_height_, passing_door_height_, eject_height_;

    bool if_hit_tank_;
    bool if_hit_car_;
    bool if_hit_pillbox_;
    bool if_hit_tent_;

    double current_x_;
    double current_y_;
    double current_z_;

    int current_step;
};

#endif //BEHAVIOR_CONTROL_HPP
