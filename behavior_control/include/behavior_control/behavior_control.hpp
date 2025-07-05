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

#include "std_msgs/msg/bool.hpp"

class BehaviorControl : public rclcpp::Node
{
public:
    explicit BehaviorControl(std::string name);
    void step_timer_callback();
    void mission_timer_callback();

private:
    void CurrentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void ArmStateCallback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr landing_state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_state_sub_;

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

    std::vector<double> detected_target;

    std::map<std::string, std::vector<double>> target_positions_;
    std::map<std::string, bool> if_hit_target_;

    std::vector<std::string> target_sequence_;

    double cruise_height_, detection_height_, passing_door_height_, eject_height_;

    bool if_hit_tank_;
    bool if_hit_car_;
    bool if_hit_pillbox_;
    bool if_hit_tent_;
    bool if_passing_door_;

    bool if_ready_to_fly;
    bool if_landing;

    double current_x_;
    double current_y_;
    double current_z_;

    int current_step;

    int eject_cnt;
    int detection_cnt;
    int eject_cnt_threshold_;
    int detection_cnt_threshold_;

    geometry_msgs::msg::PoseStamped::SharedPtr current_target_position_;
};

#endif //BEHAVIOR_CONTROL_HPP
