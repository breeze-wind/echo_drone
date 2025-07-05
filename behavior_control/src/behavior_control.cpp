//
// Created by elsa on 25-7-4.
//

#include "behavior_control/behavior_control.hpp"

BehaviorControl::BehaviorControl(std::string name) : Node("behavior_control")
{
    RCLCPP_INFO(this->get_logger(), "%s node create", name.c_str());

    this->declare_parameter<std::vector<double>>("tank_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("tent_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("car_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pillbox_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("passing_door_src", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("passing_door_des", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<std::string>>("target_sequence", std::vector<std::string>{"tent", "car", "pillbox", "tank"});
    this->declare_parameter("cruise_height", 0.6);
    this->declare_parameter("detection_height", 1.5);
    this->declare_parameter("passing_door_height", 0.4);
    this->declare_parameter("eject_height", 0.25);
    this->declare_parameter("if_hit_tank", false);
    this->declare_parameter("if_hit_car", false);
    this->declare_parameter("if_hit_pillbox", false);
    this->declare_parameter("if_hit_tent", false);
    this->declare_parameter("if_passing_door", false);

    this->get_parameter<std::vector<double>>("tank_position", tank_);
    this->get_parameter<std::vector<double>>("tent_position", tent_);
    this->get_parameter<std::vector<double>>("car_position", car_);
    this->get_parameter<std::vector<double>>("pillbox_position", pillbox_);
    this->get_parameter<std::vector<double>>("passing_door_src", passing_door_src_);
    this->get_parameter<std::vector<double>>("passing_door_des", passing_door_des_);
    this->get_parameter<std::vector<std::string>>("target_sequence", target_sequence_);
    this->get_parameter("cruise_height", cruise_height_);
    this->get_parameter("detection_height", detection_height_);
    this->get_parameter("passing_door_height", passing_door_height_);
    this->get_parameter("eject_height", eject_height_);
    this->get_parameter("if_hit_tank", if_hit_tank_);
    this->get_parameter("if_hit_car", if_hit_car_);
    this->get_parameter("if_hit_pillbox", if_hit_pillbox_);
    this->get_parameter("if_hit_tent", if_hit_tent_);
    this->get_parameter("if_passing_door", if_passing_door_);

    target_positions_["tank"] = tank_;
    target_positions_["tent"] = tent_;
    target_positions_["car"] = car_;
    target_positions_["pillbox"] = pillbox_;
    if_hit_target_["tank"] = if_hit_tank_;
    if_hit_target_["tent"] = if_hit_tent_;
    if_hit_target_["car"] = if_hit_tank_;
    if_hit_target_["pillbox"] = if_hit_tank_;

    detected_target.reserve(2);

    current_x_ = 0.0;
    current_y_ = 0.0;
    current_z_ = 0.0;

    current_step = 0;
    eject_cnt = 0;
    detection_cnt = 0;
    eject_cnt_threshold_ = 3.0 / 0.25; //等待投掷时间
    detection_cnt_threshold_ = 5.0 / 0.25; //等待识别时间

    if_ready_to_fly = false;
    if_landing = false;

    current_target_position_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    current_target_position_->pose.orientation.x = 0.0;
    current_target_position_->pose.orientation.y = 0.0;
    current_target_position_->pose.orientation.z = 0.0;
    current_target_position_->pose.orientation.w = 1.0;

    if(!if_passing_door_)
    {
        passing_door_src_[0] = 0.0;
        passing_door_src_[1] = 0.0;
        passing_door_des_[0] = 0.0;
        passing_door_des_[1] = 0.0;
        passing_door_height_ = cruise_height_;
    }

    step_period_ms = std::chrono::seconds(static_cast<int64_t>(0.25));
    mission_period_ms = std::chrono::seconds(static_cast<int64_t>(0.05));

    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot/target_pose", 10);
    landing_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/landing_state", 10);
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/robot/current_pose",
        10, std::bind(&BehaviorControl::CurrentPoseCallback, this, std::placeholders::_1));
    arm_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/robot/arm_state", 10,
        std::bind(&BehaviorControl::ArmStateCallback, this, std::placeholders::_1));

    step_timer_ = this->create_wall_timer(step_period_ms, std::bind(&BehaviorControl::step_timer_callback, this));
    mission_timer_ = this->create_wall_timer(mission_period_ms, std::bind(&BehaviorControl::mission_timer_callback, this));
}

void BehaviorControl::CurrentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;
    current_z_ = msg->pose.position.z;
}

void BehaviorControl::step_timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "---------->current step: %d", current_step);
    if(current_step == 0) //等待飞控解锁
    {
        if(if_ready_to_fly)
            current_step = 1;
    }
    else if(current_step == 1) //等待起飞至巡航高度
    {
        if(fabs(current_z_ - cruise_height_) <= 0.03)
        {
            current_step = 21;
        }
    }
    //进入第一个目标点循环
    else if(current_step == 21) //是否到目标点附近
    {
        if(fabs(current_x_ - target_positions_[target_sequence_[0]][0]) < 0.25)
            if(fabs(current_y_ - target_positions_[target_sequence_[0]][1]) < 0.25)
            {
                if(if_hit_target_[target_sequence_[0]]) //进行投掷
                    current_step = 22;
                else //不投掷
                    current_step = 31;
            }
    }
    else if(current_step == 22) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 23;
        }
    }
    else if(current_step == 23) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 31;
        }
    }
    //进入第二个目标点循环
    else if(current_step == 31) //是否到目标点附近
    {
        if(fabs(current_x_ - target_positions_[target_sequence_[1]][0]) < 0.25)
            if(fabs(current_y_ - target_positions_[target_sequence_[1]][1]) < 0.25)
            {
                if(if_hit_target_[target_sequence_[1]]) //进行投掷
                    current_step = 32;
                else //不投掷
                    current_step = 41;
            }
    }
    else if(current_step == 32) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 33;
        }
    }
    else if(current_step == 33) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 41;
        }
    }
    //进入第三个目标点循环
    else if(current_step == 41) //是否到目标点附近
    {
        if(fabs(current_x_ - target_positions_[target_sequence_[2]][0]) < 0.25)
            if(fabs(current_y_ - target_positions_[target_sequence_[2]][1]) < 0.25)
            {
                if(if_hit_target_[target_sequence_[2]]) //进行投掷
                    current_step = 42;
                else //不投掷
                    current_step = 51;
            }
    }
    else if(current_step == 42) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 43;
        }
    }
    else if(current_step == 43) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 51;
        }
    }
    //进入动态目标点循环（逻辑待改）
    else if(current_step == 51) //是否到目标点附近
    {
        if(fabs(current_x_ - target_positions_[target_sequence_[3]][0]) < 0.15)
            if(fabs(current_y_ - target_positions_[target_sequence_[3]][1]) < 0.15)
            {
                if(if_hit_target_[target_sequence_[3]]) //进行投掷
                    current_step = 52;
                else //不投掷
                    current_step = 61;
            }
    }
    else if(current_step == 52) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 53;
        }
    }
    else if(current_step == 53) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 61;
        }
    }
    //进入穿门任务
    else if(current_step == 61) //去穿门起点
    {
        if(fabs(current_x_ - passing_door_src_[0]) < 0.15)
            if(fabs(current_y_ - passing_door_src_[1]) < 0.15)
            {
                if(!if_passing_door_)
                    current_step = 63;
                else
                    current_step = 62;
            }
    }
    else if(current_step == 62) //导航至穿门终点
    {
        if(fabs(current_x_ - passing_door_des_[0]) < 0.15)
            if(fabs(current_y_ - passing_door_des_[1]) < 0.15)
            {
                current_step = 63;
            }
    }
    else if(current_step == 63) //终点/起点H识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 71;
        }
    }
    //进入降落状态
    else if(current_step == 71)
    {
        if_landing = true;
    }
}

void BehaviorControl::mission_timer_callback()
{
    // RCLCPP_INFO(this->get_logger(), "current target: %d", current_step);

    if(current_step == 0)
    {
        current_target_position_->pose.position.x = 0.0;
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = 0.0;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "等待飞控解锁...");
    }
    else if(current_step == 1)
    {
        current_target_position_->pose.position.x = 0.0;
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = cruise_height_;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "等待起飞至巡航高度...");
    }
    else if(current_step == 21)
    {
        RCLCPP_INFO(this->get_logger(), "第一个目标点，current x y: %lf, %lf", current_x_, current_y_);
    }
    else if(current_step == 22)
    {
        current_target_position_->pose.position.x = 0.0; //改识别位置
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = detection_height_;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "识别中...");
        RCLCPP_INFO(this->get_logger(), "detected id: ");
        RCLCPP_INFO(this->get_logger(), "detection position: ");
    }
    else if(current_step == 23)
    {
        current_target_position_->pose.position.x = 0.0; //改识别位置
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = eject_height_;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "下降投掷");
    }

    else if(current_step == 31)
    {
        RCLCPP_INFO(this->get_logger(), "第二个目标点，current x y: %lf, %lf", current_x_, current_y_);
    }
    else if(current_step == 32)
    {
        current_target_position_->pose.position.x = 0.0; //改识别位置
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = detection_height_;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "识别中...");
        RCLCPP_INFO(this->get_logger(), "detected id: ");
        RCLCPP_INFO(this->get_logger(), "detection position: ");
    }
    else if(current_step == 33)
    {
        current_target_position_->pose.position.x = 0.0; //改识别位置
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = eject_height_;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "下降投掷");
    }

    else if(current_step == 41)
    {
        RCLCPP_INFO(this->get_logger(), "第三个目标点，current x y: %lf, %lf", current_x_, current_y_);
    }
    else if(current_step == 42)
    {
        current_target_position_->pose.position.x = 0.0; //改识别位置
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = detection_height_;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "识别中...");
        RCLCPP_INFO(this->get_logger(), "detected id: ");
        RCLCPP_INFO(this->get_logger(), "detection position: ");
    }
    else if(current_step == 43)
    {
        current_target_position_->pose.position.x = 0.0; //改识别位置
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = eject_height_;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "下降投掷");
    }

    else if(current_step == 51)
    {
        RCLCPP_INFO(this->get_logger(), "动态目标点，current x y: %lf, %lf", current_x_, current_y_);
    }
    else if(current_step == 52)
    {
        current_target_position_->pose.position.x = 0.0; //改识别位置
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = detection_height_;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "识别中...");
        RCLCPP_INFO(this->get_logger(), "detected id: ");
        RCLCPP_INFO(this->get_logger(), "detection position: ");
    }
    else if(current_step == 53)
    {
        current_target_position_->pose.position.x = 0.0; //改识别位置
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = eject_height_;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "下降投掷");
    }

    else if(current_step == 61)
    {
        if(if_passing_door_)
            RCLCPP_INFO(this->get_logger(), "穿门起点，current x y: %lf, %lf", current_x_, current_y_);
        else
            RCLCPP_INFO(this->get_logger(), "返回起点，current x y: %lf, %lf", current_x_, current_y_);
    }
    else if(current_step == 62)
    {
        RCLCPP_INFO(this->get_logger(), "穿门终点，current x y: %lf, %lf", current_x_, current_y_);
    }
    else if(current_step == 63)
    {
        current_target_position_->pose.position.x = 0.0; //改识别位置
        current_target_position_->pose.position.y = 0.0;
        current_target_position_->pose.position.z = passing_door_height_;
        target_pose_pub_->publish(*current_target_position_);
        RCLCPP_INFO(this->get_logger(), "识别终点H...");
    }
    else if(current_step == 71)
    {
        std_msgs::msg::Bool msg;
        msg.data = if_landing;
        landing_state_pub_->publish(msg);
    }
}

void BehaviorControl::ArmStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if_ready_to_fly = msg->data;
}
