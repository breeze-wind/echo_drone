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
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "robot_interfaces/msg/image_location.hpp"
#include "robot_interfaces/msg/openmv_info.hpp"

class BehaviorControl : public rclcpp::Node
{
public:
    explicit BehaviorControl(std::string name);
    /// 控制当前执行步骤
    void step_timer_callback();
    /// 控制当前步骤执行任务
    void mission_timer_callback();
    void set_parameter();
    void change_mode();
    void handle_parameter_response(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future);

private:
    /// 接收从飞控通信节点传来的起飞状态消息
    void ArmStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
    /// 接收从point-lio节点传来的当前位姿
    void CurrentPoseCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    /// 接收从相机传来的图像位置信息
    void ImageLocationCallback(const robot_interfaces::msg::ImageLocation::SharedPtr msg);
    /// 接收openmv的图像位置信息
    void OpenmvInfoCallback(const robot_interfaces::msg::OpenmvInfo::SharedPtr msg);

    /// 发布目标点位姿
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr target_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal navigate_to_pose_goal_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr passing_door_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr turning_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr obstacle_height_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_state_pub_;
    /// 接收当前起飞状态
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_state_sub_;
    /// 接收当前位姿
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr current_pose_sub_;
    /// 接收相机传来的图像位置信息
    rclcpp::Subscription<robot_interfaces::msg::ImageLocation>::SharedPtr image_location_sub_;
    /// 接收openmv的图像位置信息
    rclcpp::Subscription<robot_interfaces::msg::OpenmvInfo>::SharedPtr openmv_info_sub_;

    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> servo_parameter_client_;
    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> controller_server_parameter_client_;
    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> local_costmap_parameter_client_;

    /// 执行步骤计时器
    rclcpp::TimerBase::SharedPtr step_timer_;
    /// 执行任务计时器
    rclcpp::TimerBase::SharedPtr mission_timer_;
    std::chrono::milliseconds step_period_ms;
    std::chrono::milliseconds mission_period_ms;

    /* 标靶坐标以及穿门起点终点坐标 */
    std::vector<double> tank_;
    std::vector<double> tent_;
    std::vector<double> car_;
    std::vector<double> pillbox_;
    std::vector<double> bridge_;
    std::vector<double> passing_door_src_1_;
    std::vector<double> passing_door_src_2_;
    std::vector<double> passing_door_des_;

    /// 随机靶搜索坐标
    std::vector<double> random_target_search_1_;
    std::vector<double> random_target_search_2_;
    std::vector<double> random_target_search_3_;
    /// 起飞点附近随机靶搜索坐标
    std::vector<double> random_target_init_search_1_;
    std::vector<double> random_target_init_search_2_;
    /// 预设随机靶坐标为其中一个定靶点，找不到随机靶时投这个
    std::vector<double> prev_random_target_;
    /// 最终确定的随机靶坐标
    std::vector<double> random_target_;
    /// openmv识别到的不准的随机靶坐标
    std::vector<double> openmv_detected_random_target_;
    /// 是否找到随机靶，只有不准的openmv坐标时不算找到
    bool if_find_random_target_;
    /// openmv的坐标是否准确
    bool if_openmv_accurate_;
    /// openmv是否找到
    bool if_openmv_find_;

    /// 当前识别到的目标坐标xy
    std::vector<double> detected_target_;
    /// 当前识别到的目标id
    uint8_t detected_target_id_;

    /// 靶子id对应的坐标
    std::map<std::string, std::vector<double>> target_positions_;
    /// 靶子id对应的是否投掷
    std::map<std::string, bool> if_hit_target_;
    /// 目标点顺序
    std::vector<std::string> target_sequence_;

    /// 巡航高度
    double cruise_height_;
    /// 识别高度
    double detection_height_;
    double H_detection_height_;
    double dynamic_detection_height_;
    /// 穿门高度
    double passing_door_height_;
    /// 投掷高度
    double eject_height_;
    /// 动态靶投掷高度
    double dynamic_eject_height_;
    double obstacle_height_;

    /* 是否投掷该目标 */
    bool if_hit_tank_;
    bool if_hit_car_;
    bool if_hit_pillbox_;
    bool if_hit_tent_;
    bool if_hit_bridge_;
    ///是否需要经过所有的靶子
    bool if_need_passing_all_;
    /// 是否穿门
    bool if_passing_door_;
    /// 当前是否在穿门状态
    bool current_passing_door_;
    /// 当前是否在转向状态
    bool if_turning;

    /// 当前是否已经解锁
    bool arming_state;
    /// 是否在准备降落状态
    bool if_landing;
    bool if_nav;

    /* 当前位置 */
    double current_x_;
    double current_y_;
    double current_z_;

    /// 当前步骤
    int current_step;

    /* 计时计数及阈值 */
    int eject_cnt;
    int detection_cnt;
    int turning_cnt;
    int passing_cnt_1_;
    int passing_cnt_2_;
    int eject_cnt_threshold_;
    int dynamic_eject_cnt_threshold_;
    int detection_cnt_threshold_;
    int dynamic_detection_cnt_threshold_;
    int turning_cnt_threshold_;

    /// 舵机投放位置参数控制
    rclcpp::Parameter servo_param;
    /// 舵机投放位置序号
    int servo_index_;
    int last_servo_index_;
    /// 相机坐标系
    std::string camera_frame_;
    /// 目标坐标系
    std::string target_frame_;
    /// 世界系
    std::string map_frame_;

    //穿门时teb参数
    double max_vel_x_passing;
    double max_vel_y_passing;
    double max_vel_x_backwards_passing;
    double max_vel_theta_passing;
    double acc_lim_x_passing;
    double acc_lim_y_passing;
    double acc_lim_theta_passing;
    double max_global_plan_lookahead_dist;
    double weight_inflation;
    double robot_radius;

    //导航模式，0--正常导航，1--穿门时导航
    int current_nav_mode;
    int last_nav_mode;

    geometry_msgs::msg::TransformStamped map_to_livox;

    nav2_msgs::action::NavigateToPose::Goal navigate_to_pose_action_;

    geometry_msgs::msg::TransformStamped current_target_position_;
};

#endif //BEHAVIOR_CONTROL_HPP
