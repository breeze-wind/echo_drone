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
    this->declare_parameter("passing_door_height", 0.4);
    this->declare_parameter("eject_height", 0.25);
    this->declare_parameter("if_hit_tank", false);
    this->declare_parameter("if_hit_car", false);
    this->declare_parameter("if_hit_pillbox", false);
    this->declare_parameter("if_hit_tent", false);
    this->declare_parameter("if_hit_bridge", false);
    this->declare_parameter("if_passing_door", false);
    this->declare_parameter<int>("/servo/servo", 0);

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
    this->get_parameter("passing_door_height", passing_door_height_);
    this->get_parameter("eject_height", eject_height_);
    this->get_parameter("if_hit_tank", if_hit_tank_);
    this->get_parameter("if_hit_car", if_hit_car_);
    this->get_parameter("if_hit_pillbox", if_hit_pillbox_);
    this->get_parameter("if_hit_tent", if_hit_tent_);
    this->get_parameter("if_hit_bridge", if_hit_bridge_);
    this->get_parameter("if_passing_door", if_passing_door_);

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

    detected_target.reserve(2);

    current_x_ = 0.0;
    current_y_ = 0.0;
    current_z_ = 0.0;

    current_step = 0;
    eject_cnt = 0;
    detection_cnt = 0;
    eject_cnt_threshold_ = 3.0 / 0.25; //等待投掷时间
    detection_cnt_threshold_ = 5.0 / 0.25; //等待识别时间

    servo_index_ = 0;
    servo_param = rclcpp::Parameter("/servo/servo", servo_index_);
    this->set_parameter(servo_param);
    servo_index_++;

    arming_state = false;
    if_landing = false;
    if_nav = false;

    current_target_position_.transform.rotation.x = 0.0;
    current_target_position_.transform.rotation.y = 0.0;
    current_target_position_.transform.rotation.z = 0.0;
    current_target_position_.transform.rotation.w = 1.0;

    map_frame_ = "map";
    camera_frame_ = "camera_color_frame";
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
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/robot/current_pose",
        10, std::bind(&BehaviorControl::CurrentPoseCallback, this, std::placeholders::_1));
    arm_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/robot/arm_state", 10,
        std::bind(&BehaviorControl::ArmStateCallback, this, std::placeholders::_1));

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
    //进入动态目标点循环（逻辑待改）
    else if(current_step == 61) //是否到目标点附近
    {
        if(fabs(current_x_ - target_positions_[target_sequence_[4]][0]) < 0.15)
            if(fabs(current_y_ - target_positions_[target_sequence_[4]][1]) < 0.15)
            {
                if(if_hit_target_[target_sequence_[4]]) //进行投掷
                    current_step = 62;
                else //不投掷
                    current_step = 71;
            }
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
                if(!if_passing_door_)
                    current_step = 73;
                else
                    current_step = 72;
            }
    }
    else if(current_step == 72) //导航至穿门终点
    {
        if(fabs(current_x_ - passing_door_des_[0]) < 0.15)
            if(fabs(current_y_ - passing_door_des_[1]) < 0.15)
            {
                current_step = 73;
            }
    }
    else if(current_step == 73) //终点/起点H识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 81;
        }
    }
    //进入降落状态
    else if(current_step == 81)
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
                            target_positions_[target_sequence_[0]][1]);    }
    else if(current_step == 22)
    {
        /*try {
            map_to_target = tfbuffer_->lookupTransform(target_frame_, map_frame_, rclcpp::Time(),
                                               rclcpp::Duration::from_seconds(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "detected id: ");
        RCLCPP_INFO(this->get_logger(), "detection position: ");*/

        current_target_position_.transform.translation.x = target_positions_[target_sequence_[0]][0]; //map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[0]][1]; //map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
		RCLCPP_INFO(this->get_logger(), "识别中...");
    }
    else if(current_step == 23)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[0]][0]; //map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[0]][1]; //map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = eject_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        if(eject_cnt >= 4)
        {
            servo_param = rclcpp::Parameter("/servo/servo", 1);
            this->set_parameter(servo_param);
        }
        RCLCPP_INFO(this->get_logger(), "下降投掷");
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
                            target_positions_[target_sequence_[1]][1]);    }
    else if(current_step == 32)
    {
        /*try {
            map_to_target = tfbuffer_->lookupTransform(target_frame_, map_frame_, rclcpp::Time(),
                                               rclcpp::Duration::from_seconds(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "detected id: ");
        RCLCPP_INFO(this->get_logger(), "detection position: ");*/

        current_target_position_.transform.translation.x = target_positions_[target_sequence_[1]][0]; //map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[1]][1]; //map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
		RCLCPP_INFO(this->get_logger(), "识别中...");
    }
    else if(current_step == 33)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[1]][0]; //map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[1]][1]; //map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = eject_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        if(eject_cnt >= 4)
        {
            servo_param = rclcpp::Parameter("/servo/servo", 2);
            this->set_parameter(servo_param);
        }
        RCLCPP_INFO(this->get_logger(), "下降投掷");
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
                            target_positions_[target_sequence_[2]][1]);    }
    else if(current_step == 42)
    {
        /*try {
            map_to_target = tfbuffer_->lookupTransform(target_frame_, map_frame_, rclcpp::Time(),
                                               rclcpp::Duration::from_seconds(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "detected id: ");
        RCLCPP_INFO(this->get_logger(), "detection position: ");*/

        current_target_position_.transform.translation.x = target_positions_[target_sequence_[2]][0]; //map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[2]][1]; //map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
		RCLCPP_INFO(this->get_logger(), "识别中...");
    }
    else if(current_step == 43)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[2]][0]; //map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[2]][1]; //map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = eject_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        if(eject_cnt >= 4)
        {
            servo_param = rclcpp::Parameter("/servo/servo", servo_index_);
            this->set_parameter(servo_param);
        }
        RCLCPP_INFO(this->get_logger(), "下降投掷");
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
        /*try {
            map_to_target = tfbuffer_->lookupTransform(target_frame_, map_frame_, rclcpp::Time(),
                                               rclcpp::Duration::from_seconds(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "detected id: ");
        RCLCPP_INFO(this->get_logger(), "detection position: ");*/

        current_target_position_.transform.translation.x = target_positions_[target_sequence_[3]][0]; //map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[3]][1]; //map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
		RCLCPP_INFO(this->get_logger(), "识别中...");
    }
    else if(current_step == 53)
    {
        current_target_position_.transform.translation.x = target_positions_[target_sequence_[3]][0]; //map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = target_positions_[target_sequence_[3]][1]; //map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = eject_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        if(eject_cnt >= 4)
        {
            servo_param = rclcpp::Parameter("/servo/servo", 3);
            this->set_parameter(servo_param);
        }
        RCLCPP_INFO(this->get_logger(), "下降投掷");
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
            map_to_target = tfbuffer_->lookupTransform(target_frame_, map_frame_, rclcpp::Time(),
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
        current_target_position_.transform.translation.z = eject_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        if(eject_cnt >= 4)
        {
            servo_param = rclcpp::Parameter("/servo/servo", servo_index_);
            this->set_parameter(servo_param);
        }
        RCLCPP_INFO(this->get_logger(), "下降投掷");
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
    }
    else if(current_step == 72)
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
        RCLCPP_INFO(this->get_logger(), "穿门终点: %lf, %lf", passing_door_des_[0], passing_door_des_[1]);
    }
    else if(current_step == 73)
    {
        /*try {
            map_to_target = tfbuffer_->lookupTransform(target_frame_, map_frame_, rclcpp::Time(),
                                               rclcpp::Duration::from_seconds(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "识别终点H...");
        RCLCPP_INFO(this->get_logger(), "detection position: ");*/

        current_target_position_.transform.translation.x = passing_door_des_[0]; //map_to_target.transform.translation.x;
        current_target_position_.transform.translation.y = passing_door_des_[1]; //map_to_target.transform.translation.y;
        current_target_position_.transform.translation.z = passing_door_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
    }
    else if(current_step == 81)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        current_target_position_.transform.translation.z = -0.5;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "降落中...");
    }
    std_msgs::msg::Bool nav_state_msg;
    nav_state_msg.data = if_nav;
    nav_state_pub_->publish(nav_state_msg);
    RCLCPP_INFO(this->get_logger(), "<<<<<<<<<<<<<<<<<<<<<<< if_nav: %d", if_nav);
}

void BehaviorControl::ArmStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    arming_state = msg->data;
    RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>arming_state: %d", arming_state);
}
