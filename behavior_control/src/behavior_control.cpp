//
// Created by elsa on 25-7-4.
//

#include "behavior_control/behavior_control.hpp"

BehaviorControl::BehaviorControl(std::string name) : Node("behavior_control")
{
    RCLCPP_INFO(this->get_logger(), "%s node create", name.c_str());

    /* 读yaml文件参数 */
    this->declare_parameter<std::vector<double>>("tank_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("tent_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("car_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pillbox_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("bridge_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("passing_door_src", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("passing_door_des", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<std::string>>("target_sequence", std::vector<std::string>{"tent", "car", "pillbox", "tank"});
    this->declare_parameter("cruise_height", 0.6);
    this->declare_parameter("detection_height", 1.5);
    this->declare_parameter("H_detection_height", 1.5);
    this->declare_parameter("passing_door_height", 0.4);
    this->declare_parameter("eject_height", 0.25);
    this->declare_parameter("if_hit_tank", false);
    this->declare_parameter("if_hit_car", false);
    this->declare_parameter("if_hit_pillbox", false);
    this->declare_parameter("if_hit_tent", false);
    this->declare_parameter("if_hit_bridge", false);
    this->declare_parameter("if_passing_door", false);
    this->declare_parameter<int>("/servo/servo", 0);
    this->declare_parameter("max_vel_x_navigation", 1.0);
    this->declare_parameter("max_vel_y_navigation", 1.0);
    this->declare_parameter("max_vel_x_backwards_navigation", 1.0);
    this->declare_parameter("max_vel_theta_navigation", 0.15);
    this->declare_parameter("acc_lim_x_navigation", 0.3);
    this->declare_parameter("acc_lim_y_navigation", 0.3);
    this->declare_parameter("acc_lim_theta_navigation", 0.2);
    this->declare_parameter("max_vel_x_passing", 0.6);
    this->declare_parameter("max_vel_y_passing", 1.5);
    this->declare_parameter("max_vel_x_backwards_passing", 0.4);
    this->declare_parameter("max_vel_theta_passing", 0.25);
    this->declare_parameter("acc_lim_x_passing", 1.5);
    this->declare_parameter("acc_lim_y_passing", 0.4);
    this->declare_parameter("acc_lim_theta_passing", 0.25);

    this->get_parameter<std::vector<double>>("tank_position", tank_);
    this->get_parameter<std::vector<double>>("tent_position", tent_);
    this->get_parameter<std::vector<double>>("car_position", car_);
    this->get_parameter<std::vector<double>>("pillbox_position", pillbox_);
    this->get_parameter<std::vector<double>>("bridge_position", bridge_);
    this->get_parameter<std::vector<double>>("passing_door_src", passing_door_src_);
    this->get_parameter<std::vector<double>>("passing_door_des", passing_door_des_);
    this->get_parameter<std::vector<std::string>>("target_sequence", target_sequence_);
    this->get_parameter("cruise_height", cruise_height_);
    this->get_parameter("detection_height", detection_height_);
    this->get_parameter("H_detection_height", H_detection_height_);
    this->get_parameter("passing_door_height", passing_door_height_);
    this->get_parameter("eject_height", eject_height_);
    this->get_parameter("if_hit_tank", if_hit_tank_);
    this->get_parameter("if_hit_car", if_hit_car_);
    this->get_parameter("if_hit_pillbox", if_hit_pillbox_);
    this->get_parameter("if_hit_tent", if_hit_tent_);
    this->get_parameter("if_hit_bridge", if_hit_bridge_);
    this->get_parameter("if_passing_door", if_passing_door_);
    this->get_parameter("max_vel_x_navigation", max_vel_x_navigation);
    this->get_parameter("max_vel_y_navigation", max_vel_y_navigation);
    this->get_parameter("max_vel_x_backwards_navigation", max_vel_x_backwards_navigation);
    this->get_parameter("max_vel_theta_navigation", max_vel_theta_navigation);
    this->get_parameter("acc_lim_x_navigation", acc_lim_x_navigation);
    this->get_parameter("acc_lim_y_navigation", acc_lim_y_navigation);
    this->get_parameter("acc_lim_theta_navigation", acc_lim_theta_navigation);
    this->get_parameter("max_vel_x_passing", max_vel_x_passing);
    this->get_parameter("max_vel_y_passing", max_vel_y_passing);
    this->get_parameter("max_vel_x_backwards_passing", max_vel_x_backwards_passing);
    this->get_parameter("max_vel_theta_passing", max_vel_theta_passing);
    this->get_parameter("acc_lim_x_passing", acc_lim_x_passing);
    this->get_parameter("acc_lim_y_passing", acc_lim_y_passing);
    this->get_parameter("acc_lim_theta_passing", acc_lim_theta_passing);

    if (!if_hit_tank_)
    {
        tank_[0] = 0.0;
        tank_[1] = 0.0;
    }

    RCLCPP_INFO(this->get_logger(), "tank_position: %lf, %lf", tank_[0], tank_[1]);
    RCLCPP_INFO(this->get_logger(), "tent_position: %lf, %lf", tent_[0], tent_[1]);
    RCLCPP_INFO(this->get_logger(), "car_position: %lf, %lf", car_[0], car_[1]);
    RCLCPP_INFO(this->get_logger(), "pillbox_position: %lf, %lf", pillbox_[0], pillbox_[1]);
    RCLCPP_INFO(this->get_logger(), "bridge_position: %lf, %lf", bridge_[0], bridge_[1]);

    target_positions_["tank"] = tank_;
    target_positions_["tent"] = tent_;
    target_positions_["car"] = car_;
    target_positions_["pillbox"] = pillbox_;
    target_positions_["bridge"] = bridge_;
    if_hit_target_["tank"] = if_hit_tank_;
    if_hit_target_["tent"] = if_hit_tent_;
    if_hit_target_["car"] = if_hit_car_;
    if_hit_target_["pillbox"] = if_hit_pillbox_;
    if_hit_target_["bridge"] = if_hit_bridge_;

    current_passing_door_ = false;
    if_turning = false;

    detected_target.reserve(2);

    current_x_ = 0.0;
    current_y_ = 0.0;
    current_z_ = 0.0;

    current_step = 71;
    eject_cnt = 0;
    detection_cnt = 0;
    turning_cnt = 0;
    eject_cnt_threshold_ = 5.0 / 0.25; //等待投掷时间
    detection_cnt_threshold_ = 2.5 / 0.25; //等待识别时间
    turning_cnt_threshold_ = 6.0 / 0.25; //等待转向时间

    servo_index_ = 0;
    last_servo_index_ = 0;
    current_nav_mode = 0;
    last_nav_mode = 0;

    arming_state = false;
    if_landing = false;
    if_nav = false;

    current_target_position_.transform.rotation.x = 0.0;
    current_target_position_.transform.rotation.y = 0.0;
    current_target_position_.transform.rotation.z = 0.0;
    current_target_position_.transform.rotation.w = 1.0;

    map_frame_ = "map";
    camera_frame_ = "camera_link";
    target_frame_ = "target_position";

    if(!if_passing_door_)
    {
        passing_door_src_[0] = 0.0;
        passing_door_src_[1] = 0.0;
        passing_door_des_[0] = 0.0;
        passing_door_des_[1] = 0.0;
        passing_door_height_ = cruise_height_;
    }

    /* 计时器，pubsub初始化 */
    step_period_ms = std::chrono::milliseconds(static_cast<int64_t>(250));
    mission_period_ms = std::chrono::milliseconds(static_cast<int64_t>(100));

    navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    if (!this->navigate_to_pose_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
    }
    navigate_to_pose_action_ = nav2_msgs::action::NavigateToPose::Goal();
    navigate_to_pose_action_.pose.pose.position.x = 0.0;
    navigate_to_pose_action_.pose.pose.position.y = 0.0;
    navigate_to_pose_action_.pose.pose.position.z = 0.0;
    navigate_to_pose_action_.pose.pose.orientation.x = 0.0;
    navigate_to_pose_action_.pose.pose.orientation.y = 0.0;
    navigate_to_pose_action_.pose.pose.orientation.z = 0.0;
    navigate_to_pose_action_.pose.pose.orientation.w = 1.0;
    navigate_to_pose_goal_ = nav2_msgs::action::NavigateToPose::Goal();

    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/robot/target_pose", 10);
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    nav_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/nav_state", 10);
    passing_door_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/passing_door_state", 10);
    turning_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/turning_state", 10);
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/robot/current_pose",
        10, std::bind(&BehaviorControl::CurrentPoseCallback, this, std::placeholders::_1));
    arm_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/robot/arm_state", 10,
        std::bind(&BehaviorControl::ArmStateCallback, this, std::placeholders::_1));

    servo_parameter_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/servo_node/set_parameters");
    controller_server_parameter_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/controller_server/set_parameters");
    //等待服务可用
    while (!servo_parameter_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "servo_parameter service not available, waiting...");
    }
    while (!controller_server_parameter_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "controller_server_parameter service not available, waiting...");
    }

    tfbuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_);

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
        if(arming_state)
            current_step = 1;
    }
    else if(current_step == 1) //等待起飞至巡航高度
    {
        if(fabs(current_z_ - cruise_height_) <= 0.05)
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
    else if(current_step == 22) //拉高
    {
        if (fabs(current_z_ - 1.6) < 0.2)
        {
            current_step = 23;
        }
    }
    else if(current_step == 23) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 24;
        }
    }
    else if(current_step == 24) //下降投掷
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
    else if(current_step == 32) //拉高
    {
        if (fabs(current_z_ - 1.6) < 0.2)
        {
            current_step = 33;
        }
    }
    else if(current_step == 33) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 34;
        }
    }
    else if(current_step == 34) //下降投掷
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
    else if(current_step == 42) //拉高
    {
        if (fabs(current_z_ - 1.6) < 0.2)
        {
            current_step = 43;
        }
    }
    else if(current_step == 43) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 44;
        }
    }
    else if(current_step == 44) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 51;
        }
    }
    //进入第四个目标点循环
    else if(current_step == 51) //是否到目标点附近
    {
        if(fabs(current_x_ - target_positions_[target_sequence_[3]][0]) < 0.25)
            if(fabs(current_y_ - target_positions_[target_sequence_[3]][1]) < 0.25)
            {
                if(if_hit_target_[target_sequence_[3]]) //进行投掷
                    current_step = 52;
                else //不投掷
                    current_step = 61;
            }
    }
    else if(current_step == 52) //拉高
    {
        if (fabs(current_z_ - 1.6) < 0.2)
        {
            current_step = 53;
        }
    }
    else if(current_step == 53) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 54;
        }
    }
    else if(current_step == 54) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 61;
        }
    }
    //进入动态目标点循环（逻辑待改）
    else if(current_step == 61)
    {
        if (if_hit_target_[target_sequence_[4]]) //进行投掷
        {
            if (fabs(current_x_ - target_positions_[target_sequence_[4]][0]) < 0.15) //是否到目标点附近
                if (fabs(current_y_ - target_positions_[target_sequence_[4]][1]) < 0.15)
                    current_step = 62;
        }
        else //不投掷
            current_step = 71;
    }
    else if(current_step == 62) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 63;
        }
    }
    else if(current_step == 63) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 71;
        }
    }
    //进入穿门任务
    else if(current_step == 71) //去穿门起点
    {
        if(fabs(current_x_ - passing_door_src_[0]) < 0.15)
            if(fabs(current_y_ - passing_door_src_[1]) < 0.15)
            {
                if(!if_passing_door_) //不穿门回起点
                    current_step = 75;
                else
                    current_step = 72;
            }
    }
    else if(current_step == 72) //原地转向90度
    {
        turning_cnt++;
        if (turning_cnt >= turning_cnt_threshold_)
        {
            turning_cnt = 0;
            current_step = 73;
        }
    }
    else if(current_step == 73) //导航至穿门终点
    {
        if(fabs(current_x_ - passing_door_des_[0]) < 0.12)
            if(fabs(current_y_ - passing_door_des_[1]) < 0.12)
            {
                current_step = 82;
            }
    }
    else if(current_step == 75) //直接回起点
    {
        if(fabs(current_x_) < 0.1)
            if(fabs(current_y_) < 0.1)
            {
                current_step = 81;
            }
    }
    //进入降落起点状态
    else if(current_step == 81)
    {
        if_landing = true;
    }
    //进入降落穿门后终点状态
    else if(current_step == 82)
    {
        if_landing = true;
    }
}

void BehaviorControl::mission_timer_callback()
{
    if(current_step == 0)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        current_target_position_.transform.translation.z = 0.0;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = false;
        RCLCPP_INFO(this->get_logger(), "等待飞控解锁...");
    }
    else if(current_step == 1)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        current_target_position_.transform.translation.z = cruise_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "等待起飞至巡航高度...");
    }
    else if(current_step == 21)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = target_positions_[target_sequence_[0]][0];
        action_goal.pose.pose.position.y = target_positions_[target_sequence_[0]][1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        // geometry_msgs::msg::PoseStamped goal_pose_;
        // goal_pose_.pose.position.x = target_positions_[target_sequence_[0]][0];
        // goal_pose_.pose.position.y = target_positions_[target_sequence_[0]][1];
        // goal_pose_.pose.position.z = cruise_height_;
        // goal_pose_pub_->publish(goal_pose_);
        RCLCPP_INFO(this->get_logger(), "第一个目标点: %lf, %lf", target_positions_[target_sequence_[0]][0],
                            target_positions_[target_sequence_[0]][1]);
    }
    else if(current_step == 22)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[0]][0];
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[0]][1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
    }
    else if(current_step == 23)
    {
        try {
            map_to_target = tfbuffer_->lookupTransform(map_frame_, target_frame_, rclcpp::Time(),
                                               rclcpp::Duration::from_seconds(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        current_target_position_.transform.translation.x = map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
		RCLCPP_INFO(this->get_logger(), "识别中...");
        RCLCPP_INFO(this->get_logger(), "detection position: %lf, %lf",
            map_to_target.transform.translation.x, map_to_target.transform.translation.y);
    }
    else if(current_step == 24)
    {
        current_target_position_.transform.translation.x = map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = map_to_target.transform.translation.y;
        if(current_z_ - eject_height_ >= 0.5)
            current_target_position_.transform.translation.z = current_z_ - 0.5;
        else
            current_target_position_.transform.translation.z = eject_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        if(eject_cnt >= 16)
        {
            servo_index_ = 1;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
    }

    else if(current_step == 31)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = target_positions_[target_sequence_[1]][0];
        action_goal.pose.pose.position.y = target_positions_[target_sequence_[1]][1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        // geometry_msgs::msg::PoseStamped goal_pose_;
        // goal_pose_.pose.position.x = target_positions_[target_sequence_[1]][0];
        // goal_pose_.pose.position.y = target_positions_[target_sequence_[1]][1];
        // goal_pose_.pose.position.z = cruise_height_;
        // goal_pose_pub_->publish(goal_pose_);
        RCLCPP_INFO(this->get_logger(), "第二个目标点: %lf, %lf", target_positions_[target_sequence_[1]][0],
                            target_positions_[target_sequence_[1]][1]);
    }
    else if(current_step == 32)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[1]][0];
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[1]][1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
    }
    else if(current_step == 33)
    {
        try {
            map_to_target = tfbuffer_->lookupTransform(map_frame_, target_frame_, rclcpp::Time(),
                                               rclcpp::Duration::from_seconds(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        current_target_position_.transform.translation.x = map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
		RCLCPP_INFO(this->get_logger(), "识别中...");
        RCLCPP_INFO(this->get_logger(), "detection position: %lf, %lf",
            map_to_target.transform.translation.x, map_to_target.transform.translation.y);
    }
    else if(current_step == 34)
    {
        current_target_position_.transform.translation.x = map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = map_to_target.transform.translation.y;
        if(current_z_ - eject_height_ >= 0.5)
            current_target_position_.transform.translation.z = current_z_ - 0.5;
        else
            current_target_position_.transform.translation.z = eject_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        if(eject_cnt >= 16)
        {
            servo_index_ = 2;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
    }

    else if(current_step == 41)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = target_positions_[target_sequence_[2]][0];
        action_goal.pose.pose.position.y = target_positions_[target_sequence_[2]][1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        // geometry_msgs::msg::PoseStamped goal_pose_;
        // goal_pose_.pose.position.x = target_positions_[target_sequence_[2]][0];
        // goal_pose_.pose.position.y = target_positions_[target_sequence_[2]][1];
        // goal_pose_.pose.position.z = cruise_height_;
        // goal_pose_pub_->publish(goal_pose_);
        RCLCPP_INFO(this->get_logger(), "第三个目标点: %lf, %lf", target_positions_[target_sequence_[2]][0],
                            target_positions_[target_sequence_[2]][1]);
    }
    else if(current_step == 42)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[2]][0];
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[2]][1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
    }
    else if(current_step == 43)
    {
        try {
            map_to_target = tfbuffer_->lookupTransform(map_frame_, target_frame_, rclcpp::Time(),
                                               rclcpp::Duration::from_seconds(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        current_target_position_.transform.translation.x = map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
		RCLCPP_INFO(this->get_logger(), "识别中...");
        RCLCPP_INFO(this->get_logger(), "detection position: %lf, %lf",
            map_to_target.transform.translation.x, map_to_target.transform.translation.y);
    }
    else if(current_step == 44)
    {
        current_target_position_.transform.translation.x = map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = map_to_target.transform.translation.y;
        if(current_z_ - eject_height_ >= 0.5)
            current_target_position_.transform.translation.z = current_z_ - 0.5;
        else
            current_target_position_.transform.translation.z = eject_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        if(eject_cnt >= 16)
        {
            servo_index_ = 3;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
    }

    else if(current_step == 51)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = target_positions_[target_sequence_[3]][0];
        action_goal.pose.pose.position.y = target_positions_[target_sequence_[3]][1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "第四个目标点: %lf, %lf", target_positions_[target_sequence_[3]][0],
					target_positions_[target_sequence_[3]][1]);
    }
    else if(current_step == 52)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[3]][0];
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[3]][1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
    }
    else if(current_step == 53)
    {
        try {
            map_to_target = tfbuffer_->lookupTransform(map_frame_, target_frame_, rclcpp::Time(),
                                               rclcpp::Duration::from_seconds(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        current_target_position_.transform.translation.x = map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
		RCLCPP_INFO(this->get_logger(), "识别中...");
        RCLCPP_INFO(this->get_logger(), "detection position: %lf, %lf",
            map_to_target.transform.translation.x, map_to_target.transform.translation.y);
    }
    else if(current_step == 54)
    {
        current_target_position_.transform.translation.x = map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = map_to_target.transform.translation.y;
        if(current_z_ - eject_height_ >= 0.5)
            current_target_position_.transform.translation.z = current_z_ - 0.5;
        else
            current_target_position_.transform.translation.z = eject_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        if(eject_cnt >= 16)
        {
            servo_index_ = 3;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
    }

    else if(current_step == 61)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = target_positions_[target_sequence_[4]][0];
        action_goal.pose.pose.position.y = target_positions_[target_sequence_[4]][1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        RCLCPP_INFO(this->get_logger(), "动态目标点，current x y: %lf, %lf", current_x_, current_y_);
    }
    else if(current_step == 62) //识别投掷策略待修改
    {
        /*try {
            map_to_target = tfbuffer_->lookupTransform(map_frame_, target_frame_, rclcpp::Time(),
                                               rclcpp::Duration::from_seconds(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "detected id: ");
        RCLCPP_INFO(this->get_logger(), "detection position: ");*/

        current_target_position_.transform.translation.x = map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
		RCLCPP_INFO(this->get_logger(), "识别中...");
    }
    else if(current_step == 63)
    {
        current_target_position_.transform.translation.x = map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = map_to_target.transform.translation.y;
        if(current_z_ - eject_height_ >= 0.5)
            current_target_position_.transform.translation.z = current_z_ - 0.5;
        else
            current_target_position_.transform.translation.z = eject_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        if(eject_cnt >= 16)
        {
            servo_index_ = 3;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
    }

    else if(current_step == 71)
    {
        if(if_passing_door_){
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
            action_goal.pose.header.frame_id = "map";
            action_goal.pose.pose.position.x = passing_door_src_[0];
            action_goal.pose.pose.position.y = passing_door_src_[1];
            action_goal.pose.pose.position.z = passing_door_height_;
            action_goal.pose.pose.orientation.x = 0.0;
            action_goal.pose.pose.orientation.y = 0.0;
            action_goal.pose.pose.orientation.z = 0.0;
            action_goal.pose.pose.orientation.w = 1.0;
            navigate_to_pose_client_->async_send_goal(action_goal);
            RCLCPP_INFO(this->get_logger(), "穿门起点: %lf, %lf", passing_door_src_[0], passing_door_src_[1]);
        }
        else{
			passing_door_des_[0] = 0.0;
			passing_door_des_[1] = 0.0;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
            action_goal.pose.header.frame_id = "map";
            action_goal.pose.pose.position.x = 0.0;
            action_goal.pose.pose.position.y = 0.0;
            action_goal.pose.pose.position.z = cruise_height_;
            action_goal.pose.pose.orientation.x = 0.0;
            action_goal.pose.pose.orientation.y = 0.0;
            action_goal.pose.pose.orientation.z = 0.0;
            action_goal.pose.pose.orientation.w = 1.0;
            navigate_to_pose_client_->async_send_goal(action_goal);
            RCLCPP_INFO(this->get_logger(), "返回起点，current x y: %lf, %lf", current_x_, current_y_);
        }
        if_nav = true;
        if_turning = false;
    }
    else if(current_step == 72)
    {
        current_target_position_.transform.translation.x = passing_door_src_[0];
        current_target_position_.transform.translation.y = passing_door_src_[1];
        current_target_position_.transform.translation.z = passing_door_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = true;
        if_turning = true; //转向状态

        current_nav_mode = 1;
        if (current_nav_mode != last_nav_mode)
        {
            change_mode();
            last_nav_mode  = current_nav_mode;
        }
        RCLCPP_INFO(this->get_logger(), "穿门前转向中...");
    }
    else if(current_step == 73)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = passing_door_des_[0];
        action_goal.pose.pose.position.y = passing_door_des_[1];
        action_goal.pose.pose.position.z = passing_door_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        navigate_to_pose_client_->async_send_goal(action_goal);
        if_nav = true;
        current_passing_door_ = true;
        if_turning = false;
        RCLCPP_INFO(this->get_logger(), "穿门终点: %lf, %lf", passing_door_des_[0], passing_door_des_[1]);
    }
    else if(current_step == 75)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        current_target_position_.transform.translation.z = cruise_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = false;
        if_turning = false;
    }
    else if(current_step == 81)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        if(current_z_ + 0.3 >= 0.5)
            current_target_position_.transform.translation.z = current_z_ - 0.5;
        else
            current_target_position_.transform.translation.z = -0.3;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = if_passing_door_;
        if_turning = false;
        RCLCPP_INFO(this->get_logger(), "降落中...");
    }
    else if(current_step == 82)
    {
        current_target_position_.transform.translation.x = passing_door_des_[0];
        current_target_position_.transform.translation.y = passing_door_des_[1];
        if(current_z_ + 0.3 >= 0.5)
            current_target_position_.transform.translation.z = current_z_ - 0.5;
        else
            current_target_position_.transform.translation.z = -0.3;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = if_passing_door_;
        if_turning = false;
        RCLCPP_INFO(this->get_logger(), "穿门后降落中...");
    }
    std_msgs::msg::Bool nav_state_msg;
    nav_state_msg.data = if_nav;
    nav_state_pub_->publish(nav_state_msg);
    RCLCPP_INFO(this->get_logger(), "<<<<<<<<<<<<<<<<<<<<<<< if_nav: %d", if_nav);
    std_msgs::msg::Bool passing_door_state_msg;
    passing_door_state_msg.data = current_passing_door_;
    passing_door_state_pub_->publish(passing_door_state_msg);
    RCLCPP_INFO(this->get_logger(), "/////////////////////// current_passing_door_: %d", current_passing_door_);
    std_msgs::msg::Bool turning_state_msg;
    turning_state_msg.data = if_turning;
    turning_state_pub_->publish(turning_state_msg);
    RCLCPP_INFO(this->get_logger(), "////////......... if_turning: %d", if_turning);
}

void BehaviorControl::ArmStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    arming_state = msg->data;
    RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>arming_state: %d", arming_state);
}

void BehaviorControl::set_parameter()
{
    // 构建请求
    auto servo_server_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter servo_server_param_servo_index;
    servo_server_param_servo_index.name = "/servo/servo";
    servo_server_param_servo_index.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    servo_server_param_servo_index.value.integer_value = servo_index_;

    servo_server_request->parameters.push_back(servo_server_param_servo_index);

    // 使用回调的异步调用
    auto servo_server_future = servo_parameter_client_->async_send_request(
        servo_server_request,
        [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
            this->handle_parameter_response(future);
        });
}

void BehaviorControl::handle_parameter_response(
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future)
{
    try {
        auto response = future.get();
        for (const auto& result : response->results) {
            if (result.successful) {
                RCLCPP_INFO(this->get_logger(), "Parameter set successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s",
                            result.reason.c_str());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}

void BehaviorControl::change_mode()
{
    // 构建请求
    auto controller_server_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter controller_server_param_max_vel_theta;
    controller_server_param_max_vel_theta.name = "FollowPath.max_vel_theta";
    controller_server_param_max_vel_theta.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_max_vel_x;
    controller_server_param_max_vel_x.name = "FollowPath.max_vel_x";
    controller_server_param_max_vel_x.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_max_vel_y;
    controller_server_param_max_vel_y.name = "FollowPath.max_vel_y";
    controller_server_param_max_vel_y.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_max_vel_x_backwards;
    controller_server_param_max_vel_x_backwards.name = "FollowPath.max_vel_x_backwards";
    controller_server_param_max_vel_x_backwards.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_acc_lim_x;
    controller_server_param_acc_lim_x.name = "FollowPath.acc_lim_x";
    controller_server_param_acc_lim_x.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_acc_lim_y;
    controller_server_param_acc_lim_y.name = "FollowPath.acc_lim_y";
    controller_server_param_acc_lim_y.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_acc_lim_theta;
    controller_server_param_acc_lim_theta.name = "FollowPath.acc_lim_theta";
    controller_server_param_acc_lim_theta.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    controller_server_param_max_vel_theta.value.double_value = max_vel_theta_passing;
    controller_server_param_max_vel_x.value.double_value = max_vel_x_passing;
    controller_server_param_max_vel_y.value.double_value = max_vel_y_passing;
    controller_server_param_max_vel_x_backwards.value.double_value = max_vel_x_backwards_passing;
    controller_server_param_acc_lim_x.value.double_value = acc_lim_x_passing;
    controller_server_param_acc_lim_y.value.double_value = acc_lim_y_passing;
    controller_server_param_acc_lim_theta.value.double_value = acc_lim_theta_passing;

    // 所有要修改的参数一起push_back
    controller_server_request->parameters.push_back(controller_server_param_max_vel_theta);
    controller_server_request->parameters.push_back(controller_server_param_max_vel_x);
    controller_server_request->parameters.push_back(controller_server_param_max_vel_y);
    controller_server_request->parameters.push_back(controller_server_param_max_vel_x_backwards);
    controller_server_request->parameters.push_back(controller_server_param_acc_lim_x);
    controller_server_request->parameters.push_back(controller_server_param_acc_lim_y);
    controller_server_request->parameters.push_back(controller_server_param_acc_lim_theta);

    // 使用回调的异步调用
    auto controller_server_future = controller_server_parameter_client_->async_send_request(
        controller_server_request,
        [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
            this->handle_parameter_response(future);
        });
}
