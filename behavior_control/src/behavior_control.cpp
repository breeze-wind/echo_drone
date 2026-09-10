//
// 由 elsa 于 25-7-4 创建。
//

#include "behavior_control/behavior_control.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

namespace
{
// 110 作为台架测试截断步骤：确认达到起飞高度后，只持续发布圆周目标，
// 不再进入旧的搜索、投掷或穿门流程。
constexpr int kTakeoffCircleStep = 110;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kMinCircleRadius = 0.05;
constexpr double kMinCircleSpeed = 0.01;
}

BehaviorControl::BehaviorControl(std::string name) : Node("behavior_control")
{
    RCLCPP_INFO(this->get_logger(), "%s node create", name.c_str());
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /* 读yaml文件参数 */
    this->declare_parameter<std::vector<double>>("tank_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("tent_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("car_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pillbox_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("bridge_position", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("passing_door_src_1", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("passing_door_src_2", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("passing_door_des", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("random_target_search_1", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("random_target_search_2", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("random_target_search_3", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("random_target_init_search_1", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("random_target_init_search_2", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("prev_random_target", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("prev_random_tank_target", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<std::string>>("target_sequence", std::vector<std::string>{"tent", "car", "pillbox", "tank"});
    this->declare_parameter("cruise_height", 0.6);
    this->declare_parameter("detection_height", 1.5);
    this->declare_parameter("H_detection_height", 1.5);
    this->declare_parameter("passing_door_height", 0.4);
    this->declare_parameter("eject_height", 0.25);
    this->declare_parameter("dynamic_eject_height", 0.5);
    this->declare_parameter("if_hit_tank", false);
    this->declare_parameter("if_hit_car", false);
    this->declare_parameter("if_hit_pillbox", false);
    this->declare_parameter("if_hit_tent", false);
    this->declare_parameter("if_hit_bridge", false);
    this->declare_parameter("if_passing_door", false);
    this->declare_parameter("if_need_passing_all", false);
    this->declare_parameter("offset_x_1", 0.1);
    this->declare_parameter("offset_y_1", 0.1);
    this->declare_parameter("offset_x_2", 0.1);
    this->declare_parameter("offset_y_2", 0.1);
    this->declare_parameter("offset_x_3", 0.1);
    this->declare_parameter("offset_y_3", 0.1);
    this->declare_parameter<int>("/servo/servo", 0);
    this->declare_parameter("max_vel_x_passing", 0.6);
    this->declare_parameter("max_vel_y_passing", 1.5);
    this->declare_parameter("max_vel_x_backwards_passing", 0.4);
    this->declare_parameter("max_vel_theta_passing", 0.25);
    this->declare_parameter("acc_lim_x_passing", 1.5);
    this->declare_parameter("acc_lim_y_passing", 0.4);
    this->declare_parameter("acc_lim_theta_passing", 0.25);
    this->declare_parameter("max_global_plan_lookahead_dist", 1.25);
    this->declare_parameter("weight_inflation", 1.0);
    this->declare_parameter("robot_radius", 0.05);
    this->declare_parameter("takeoff_circle_enabled", true);
    this->declare_parameter("takeoff_circle_radius", 0.5);
    this->declare_parameter("takeoff_circle_speed", 0.2);
    this->declare_parameter("takeoff_circle_direction", 1.0);
    // 决策运行时参数：dry_run 控制输出安全门，autostart 控制初始生命周期，
    // start_state 支持命名状态或旧数字 step，便于分阶段回归调试。
    this->declare_parameter("dry_run", false);
    this->declare_parameter("autostart", true);
    this->declare_parameter("start_state", std::string("legacy_default"));
    // 导航执行器默认仍使用 Nav2。sls_goal 是显式试验模式：只把最终目标
    // 交给 SLS 速度控制器，因而不会得到 Nav2 的实时局部避障。
    this->declare_parameter("navigation_execution_mode", std::string("nav2"));
    this->declare_parameter("sls_nav_goal_topic", std::string("/sls_circle/nav_goal"));

    this->get_parameter<std::vector<double>>("tank_position", tank_);
    this->get_parameter<std::vector<double>>("tent_position", tent_);
    this->get_parameter<std::vector<double>>("car_position", car_);
    this->get_parameter<std::vector<double>>("pillbox_position", pillbox_);
    this->get_parameter<std::vector<double>>("bridge_position", bridge_);
    this->get_parameter<std::vector<double>>("passing_door_src_1", passing_door_src_1_);
    this->get_parameter<std::vector<double>>("passing_door_src_2", passing_door_src_2_);
    this->get_parameter<std::vector<double>>("passing_door_des", passing_door_des_);
    this->get_parameter<std::vector<std::string>>("target_sequence", target_sequence_);
    this->get_parameter<std::vector<double>>("random_target_search_1", random_target_search_1_);
    this->get_parameter<std::vector<double>>("random_target_search_2", random_target_search_2_);
    this->get_parameter<std::vector<double>>("random_target_search_3", random_target_search_3_);
    this->get_parameter<std::vector<double>>("random_target_init_search_1", random_target_init_search_1_);
    this->get_parameter<std::vector<double>>("random_target_init_search_2", random_target_init_search_2_);
    this->get_parameter<std::vector<double>>("prev_random_target", prev_random_target_);
    this->get_parameter<std::vector<double>>("prev_random_tank_target", prev_random_tank_target_);
    this->get_parameter("cruise_height", cruise_height_);
    this->get_parameter("detection_height", detection_height_);
    this->get_parameter("H_detection_height", H_detection_height_);
    this->get_parameter("passing_door_height", passing_door_height_);
    this->get_parameter("eject_height", eject_height_);
    this->get_parameter("dynamic_eject_height", dynamic_eject_height_);
    this->get_parameter("if_hit_tank", if_hit_tank_);
    this->get_parameter("if_hit_car", if_hit_car_);
    this->get_parameter("if_hit_pillbox", if_hit_pillbox_);
    this->get_parameter("if_hit_tent", if_hit_tent_);
    this->get_parameter("if_hit_bridge", if_hit_bridge_);
    this->get_parameter("if_passing_door", if_passing_door_);
    this->get_parameter("if_need_passing_all", if_need_passing_all_);
    this->get_parameter("offset_x_1", offset_x_1_);
    this->get_parameter("offset_y_1", offset_y_1_);
    this->get_parameter("offset_x_2", offset_x_2_);
    this->get_parameter("offset_y_2", offset_y_2_);
    this->get_parameter("offset_x_3", offset_x_3_);
    this->get_parameter("offset_y_3", offset_y_3_);
    this->get_parameter("max_vel_x_passing", max_vel_x_passing);
    this->get_parameter("max_vel_y_passing", max_vel_y_passing);
    this->get_parameter("max_vel_x_backwards_passing", max_vel_x_backwards_passing);
    this->get_parameter("max_vel_theta_passing", max_vel_theta_passing);
    this->get_parameter("acc_lim_x_passing", acc_lim_x_passing);
    this->get_parameter("acc_lim_y_passing", acc_lim_y_passing);
    this->get_parameter("acc_lim_theta_passing", acc_lim_theta_passing);
    this->get_parameter("max_global_plan_lookahead_dist", max_global_plan_lookahead_dist);
    this->get_parameter("weight_inflation", weight_inflation);
    this->get_parameter("robot_radius", robot_radius);
    this->get_parameter("takeoff_circle_enabled", takeoff_circle_enabled_);
    this->get_parameter("takeoff_circle_radius", takeoff_circle_radius_);
    this->get_parameter("takeoff_circle_speed", takeoff_circle_speed_);
    this->get_parameter("takeoff_circle_direction", takeoff_circle_direction_);
    this->get_parameter("dry_run", dry_run_);
    this->get_parameter("autostart", autostart_);
    this->get_parameter("start_state", start_state_);
    this->get_parameter("navigation_execution_mode", navigation_execution_mode_);
    this->get_parameter("sls_nav_goal_topic", sls_nav_goal_topic_);
    std::transform(
        navigation_execution_mode_.begin(), navigation_execution_mode_.end(),
        navigation_execution_mode_.begin(), ::tolower);
    if(navigation_execution_mode_ != "nav2" && navigation_execution_mode_ != "sls_goal")
    {
        RCLCPP_WARN(this->get_logger(),
            "Unknown navigation_execution_mode '%s'; using nav2",
            navigation_execution_mode_.c_str());
        navigation_execution_mode_ = "nav2";
    }
    takeoff_circle_direction_ = takeoff_circle_direction_ >= 0.0 ? 1.0 : -1.0;
    configured_start_step_ = resolve_start_state(start_state_);
    current_step = configured_start_step_;
    mission_runtime_ = behavior_control::MissionRuntime(autostart_);
    state_enter_time_ = this->now();

    if (!if_hit_tank_)
    {
        tank_[0] = 0.0;
        tank_[1] = 0.0;
    }

    random_target_.resize(2);
    random_tank_target_.resize(2);
    detected_target_.resize(2);

    RCLCPP_INFO(this->get_logger(), "tank_position: %lf, %lf", tank_[0], tank_[1]);
    RCLCPP_INFO(this->get_logger(), "tent_position: %lf, %lf", tent_[0], tent_[1]);
    RCLCPP_INFO(this->get_logger(), "car_position: %lf, %lf", car_[0], car_[1]);
    RCLCPP_INFO(this->get_logger(), "pillbox_position: %lf, %lf", pillbox_[0], pillbox_[1]);
    RCLCPP_INFO(this->get_logger(), "bridge_position: %lf, %lf", bridge_[0], bridge_[1]);

    // target_positions_["tank"] = tank_;
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
    if_find_random_target_ = false;
    if_find_random_tank_target_ = false;

    current_x_ = 0.0;
    current_y_ = 0.0;
    current_z_ = 0.0;
    dynamic_detection_height_ = detection_height_ + 0.35;

    eject_cnt = 0;
    detection_cnt = 0;
    turning_cnt = 0;
    passing_cnt_1_ = 0;
    passing_cnt_2_ = 0;
    eject_cnt_threshold_ = 4.0 / 0.25; //等待投掷时间
    dynamic_eject_cnt_threshold_ = 1.5 / 0.25; //动态靶等待投掷时间
    detection_cnt_threshold_ = 2.5 / 0.25; //等待识别时间
    dynamic_detection_cnt_threshold_ = 5.0 / 0.25; //在初始起飞后寻找随机靶的等待时间
    turning_cnt_threshold_ = 5.0 / 0.25; //等待转向时间

    obstacle_height_ = 2.0;

    servo_index_ = 0;
    last_servo_index_ = 0;
    current_nav_mode = 0;
    last_nav_mode = 0;
    eject_last_x = 0.0;
    eject_last_y = 0.0;
    arming_state = false;
    if_landing = false;
    if_nav = false;

    is_tank_or_bridge_ = false;
    takeoff_circle_started_ = false;
    takeoff_circle_center_x_ = 0.0;
    takeoff_circle_center_y_ = 0.0;

    current_target_position_.transform.rotation.x = 0.0;
    current_target_position_.transform.rotation.y = 0.0;
    current_target_position_.transform.rotation.z = 0.0;
    current_target_position_.transform.rotation.w = 1.0;

    map_frame_ = "map";
    camera_frame_ = "camera_link";
    target_frame_ = "target_position";

    livox_to_camera_affine = Eigen::Affine3d::Identity();
    try
    {
        livox_to_camera = tf_buffer_->lookupTransform("livox", "camera_link", rclcpp::Time(),
                                        rclcpp::Duration::from_seconds(0.0));
        livox_to_camera_affine = tf2::transformToEigen(livox_to_camera);
        tf_ready_ = true;
    }
    catch(const tf2::TransformException & ex)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "livox -> camera_link TF is not ready; real mission commands remain gated: %s",
            ex.what());
    }
    // RCLCPP_INFO(this->get_logger(), "-----------------+++++++++++++++livox_to_camera x y z: %lf, %lf, %lf",
    //     livox_to_camera.transform.translation.x, livox_to_camera.transform.translation.y, livox_to_camera.transform.translation.z);
    map_to_livox_affine = Eigen::Affine3d::Identity(); //map->雷达
    map_to_camera_affine = Eigen::Affine3d::Identity();

    camera_pt_.header.frame_id = "camera_link";
    camera_pt_.point.z = 0.0;

    if(!if_passing_door_)
    {
        passing_door_src_1_[0] = 0.0;
        passing_door_src_1_[1] = 0.0;
        passing_door_des_[0] = 0.0;
        passing_door_des_[1] = 0.0;
        passing_door_height_ = cruise_height_;
    }

    /* 计时器，pubsub初始化 */
    step_period_ms = std::chrono::milliseconds(static_cast<int64_t>(250));
    mission_period_ms = std::chrono::milliseconds(static_cast<int64_t>(100));

    navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    // 决策节点启动不再同步等待 Nav2。实际发送目标前再检查 server 是否就绪，
    // 避免缺少导航子系统时整个调试接口无法启动。
    if (!this->navigate_to_pose_client_->action_server_is_ready())
        RCLCPP_WARN(this->get_logger(), "navigate_to_pose action server is not ready yet");
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
    sls_nav_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(sls_nav_goal_topic_, 10);
    sls_goal_control_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/robot/sls_goal_control_active", 10);
    nav_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/nav_state", 10);
    passing_door_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/passing_door_state", 10);
    turning_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/turning_state", 10);
    obstacle_height_pub_ = this->create_publisher<std_msgs::msg::Float64>("/robot/obstacle_height", 10);
    clear_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/clear_state", 10);
    camera_choose_pub_ = this->create_publisher<std_msgs::msg::Bool>("/camera/choose", 10);
    mission_status_pub_ = this->create_publisher<robot_interfaces::msg::MissionStatus>("/mission/status", 10);
    legacy_mission_status_pub_ = this->create_publisher<robot_interfaces::msg::MissionStatus>("/robot/mission_status", 10);
    mission_event_pub_ = this->create_publisher<robot_interfaces::msg::MissionEvent>("/mission/event", 50);
    mission_graph_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/mission/graph_dot", rclcpp::QoS(1).transient_local().reliable());
    arm_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/robot/arm_state", 10,
        std::bind(&BehaviorControl::ArmStateCallback, this, std::placeholders::_1));
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("/robot/current_pose",
            10, std::bind(&BehaviorControl::CurrentPoseCallback, this, std::placeholders::_1));
    image_location_sub_ = this->create_subscription<robot_interfaces::msg::ImageLocation>("/robot/image_location",
            10, std::bind(&BehaviorControl::ImageLocationCallback, this, std::placeholders::_1));
    usbcamera_info_sub_ = this->create_subscription<robot_interfaces::msg::ImageLocation>("/robot/usb_camera",
            10, std::bind(&BehaviorControl::USBCameraInfoCallback, this, std::placeholders::_1));

    servo_parameter_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/servo_node/set_parameters");
    controller_server_parameter_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/controller_server/set_parameters");
    local_costmap_parameter_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/local_costmap/local_costmap/set_parameters");

    mission_start_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/start", std::bind(&BehaviorControl::handle_start, this,
        std::placeholders::_1, std::placeholders::_2));
    mission_pause_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/pause", std::bind(&BehaviorControl::handle_pause, this,
        std::placeholders::_1, std::placeholders::_2));
    mission_resume_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/resume", std::bind(&BehaviorControl::handle_resume, this,
        std::placeholders::_1, std::placeholders::_2));
    mission_step_once_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/step_once", std::bind(&BehaviorControl::handle_step_once, this,
        std::placeholders::_1, std::placeholders::_2));
    mission_abort_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/abort", std::bind(&BehaviorControl::handle_abort, this,
        std::placeholders::_1, std::placeholders::_2));
    mission_dump_context_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/mission/dump_context", std::bind(&BehaviorControl::handle_dump_context, this,
        std::placeholders::_1, std::placeholders::_2));
    mission_jump_service_ = this->create_service<robot_interfaces::srv::JumpMissionState>(
        "/mission/jump_to_state", std::bind(&BehaviorControl::handle_jump_to_state, this,
        std::placeholders::_1, std::placeholders::_2));
    mission_shift_service_ = this->create_service<robot_interfaces::srv::ShiftMissionState>(
        "/mission/shift_state", std::bind(&BehaviorControl::handle_shift_state, this,
        std::placeholders::_1, std::placeholders::_2));

    step_timer_ = this->create_wall_timer(step_period_ms, std::bind(&BehaviorControl::step_timer_callback, this));
    mission_timer_ = this->create_wall_timer(mission_period_ms, std::bind(&BehaviorControl::mission_timer_callback, this));
    mission_status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&BehaviorControl::publish_mission_status, this));
    mission_graph_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&BehaviorControl::publish_mission_graph, this));

    publish_mission_event(current_step, current_step, autostart_ ? "autostart" : "initialized_idle");
    publish_mission_graph();
    if(use_sls_goal_navigation())
    {
        RCLCPP_WARN(this->get_logger(),
            "navigation_execution_mode=sls_goal: final goals go to %s; Nav2 local obstacle avoidance is bypassed",
            sls_nav_goal_topic_.c_str());
    }
}

int BehaviorControl::resolve_start_state(const std::string & value) const
{
    // legacy_default 保持历史启动行为：从 step 1（等待达到巡航高度）开始。
    if(value.empty() || value == "legacy_default")
        return 1;

    const auto * named_node = mission_graph_.node_by_name(value);
    if(named_node != nullptr)
        return named_node->legacy_step;

    try
    {
        const int numeric_step = std::stoi(value);
        const auto * legacy_node = mission_graph_.node_by_legacy_step(numeric_step);
        if(legacy_node != nullptr)
            return legacy_node->legacy_step;
    }
    catch(const std::exception &)
    {
    }

    RCLCPP_WARN(this->get_logger(), "Unknown start_state '%s'; using legacy_default (step 1)", value.c_str());
    return 1;
}

std::string BehaviorControl::mission_state_name(int step) const
{
    // 此映射是数字状态机迁移期间的兼容边界。后续每迁出一个状态，
    // 仍保留对应 legacy_step，便于新旧 rosbag 和事件日志逐项对比。
    const auto * node = mission_graph_.node_by_legacy_step(step);
    return node == nullptr ? "unknown" : node->name;
}

std::string BehaviorControl::mission_phase_name(int step) const
{
    if(step == 0 || step == 1 || step == 300)
        return "startup";
    if(step == kTakeoffCircleStep)
        return "circle";
    if(step == 21 || step == 31 || step == 41 || step == 51 || step == 61 || step == 101 ||
       step == 71 || step == 73 || step == 74 || step == 75 || step == 91 || step == 92 ||
       step == 93 || step == 111 || step == 113)
        return "navigate";
    if(step == 22 || step == 32 || step == 42 || step == 52 || step == 62 || step == 102)
        return "raise";
    if(step == 23 || step == 33 || step == 43 || step == 53 || step == 63 || step == 103 ||
       step == 112 || step == 114)
        return "detect";
    if(step == 24 || step == 34 || step == 44 || step == 54 || step == 64 || step == 104)
        return "eject";
    if(step == 72)
        return "turn";
    if(step == 81 || step == 82)
        return "land";
    return "unknown";
}

std::string BehaviorControl::mission_target_name(int step) const
{
    if(step >= 21 && step <= 24 && target_sequence_.size() > 0)
        return target_sequence_[0];
    if(step >= 31 && step <= 34 && target_sequence_.size() > 1)
        return target_sequence_[1];
    if(step >= 41 && step <= 44 && target_sequence_.size() > 2)
        return target_sequence_[2];
    if(step >= 51 && step <= 54 && target_sequence_.size() > 3)
        return target_sequence_[3];
    if(step >= 61 && step <= 64)
        return "random_tank";
    if((step >= 91 && step <= 93) || (step >= 101 && step <= 104) ||
       (step >= 111 && step <= 114))
        return "random";
    if(step >= 71 && step <= 75)
        return "door";
    if(step == 81 || step == 82)
        return "landing";
    if(step == kTakeoffCircleStep)
        return "takeoff_circle";
    return "";
}

void BehaviorControl::publish_mission_status()
{
    // 状态话题只负责观测，不会触发任何任务动作。
    robot_interfaces::msg::MissionStatus msg;
    const auto * node = mission_graph_.node_by_legacy_step(current_step);
    msg.stamp = this->now();
    msg.legacy_step = current_step;
    msg.state_id = node == nullptr ? -1 : node->state_id;
    msg.state = mission_state_name(current_step);
    msg.phase = mission_phase_name(current_step);
    msg.target = mission_target_name(current_step);
    msg.execution_state = mission_runtime_.state_name();
    msg.elapsed_sec = std::max(0.0, (this->now() - state_enter_time_).seconds());
    msg.dry_run = dry_run_;
    msg.armed = arming_state;
    msg.navigation_active = if_nav;
    if(if_find_random_target_)
        msg.flags.push_back("random_target_found");
    if(if_find_random_tank_target_)
        msg.flags.push_back("random_tank_found");
    if(current_passing_door_)
        msg.flags.push_back("passing_door");
    if(if_turning)
        msg.flags.push_back("turning");
    if(!tf_ready_)
        msg.flags.push_back("tf_missing");
    if(!pose_received_)
        msg.flags.push_back("pose_missing");
    msg.outgoing_state_ids = mission_graph_.outgoing_state_ids(msg.state_id);
    const auto * previous = mission_graph_.shift_state(msg.state_id, -1);
    const auto * next = mission_graph_.shift_state(msg.state_id, 1);
    msg.debug_previous_state_id = previous == nullptr ? -1 : previous->state_id;
    msg.debug_next_state_id = next == nullptr ? -1 : next->state_id;
    mission_status_pub_->publish(msg);
    legacy_mission_status_pub_->publish(msg);
}

void BehaviorControl::publish_mission_event(
    int from_step, int to_step, const std::string & reason)
{
    // 同状态事件用于记录 pause/resume/dump 等控制操作；真正的状态转移则
    // 同时携带前后 step 和命名状态。
    robot_interfaces::msg::MissionEvent msg;
    msg.stamp = this->now();
    msg.sequence = ++mission_event_sequence_;
    msg.from_legacy_step = from_step;
    msg.to_legacy_step = to_step;
    const auto * from_node = mission_graph_.node_by_legacy_step(from_step);
    const auto * to_node = mission_graph_.node_by_legacy_step(to_step);
    msg.from_state_id = from_node == nullptr ? -1 : from_node->state_id;
    msg.to_state_id = to_node == nullptr ? -1 : to_node->state_id;
    msg.from_state = mission_state_name(from_step);
    msg.to_state = mission_state_name(to_step);
    msg.reason = reason;
    msg.execution_state = mission_runtime_.state_name();
    mission_event_pub_->publish(msg);
    RCLCPP_INFO(
        this->get_logger(), "mission event #%lu: %s(%d) -> %s(%d), reason=%s, execution=%s",
        static_cast<unsigned long>(msg.sequence), msg.from_state.c_str(), from_step,
        msg.to_state.c_str(), to_step, reason.c_str(), msg.execution_state.c_str());
    // 状态变化后立即刷新图；1 Hz 定时器负责后加入的普通 volatile 订阅者。
    publish_mission_graph();
}

void BehaviorControl::publish_mission_graph()
{
    std_msgs::msg::String graph;
    const auto * current = mission_graph_.node_by_legacy_step(current_step);
    graph.data = mission_graph_.to_dot(
        current == nullptr ? -1 : current->state_id, mission_runtime_.state_name());
    mission_graph_pub_->publish(graph);
}

void BehaviorControl::reset_mission_context()
{
    current_step = configured_start_step_;
    reset_state_local_context();
    state_enter_time_ = this->now();
}

void BehaviorControl::reset_state_local_context()
{
    detection_cnt = 0;
    eject_cnt = 0;
    turning_cnt = 0;
    passing_cnt_1_ = 0;
    passing_cnt_2_ = 0;
    servo_index_ = 0;
    last_servo_index_ = 0;
    if_nav = false;
    sls_goal_control_active_ = false;
    current_passing_door_ = false;
    if_turning = false;
    if_landing = false;
    takeoff_circle_started_ = false;
}

bool BehaviorControl::publish_safe_hold()
{
    // 不用默认零值冒充当前位置。只有实飞模式、必要 TF 和真实位姿都有效时，
    // 才允许发布一次当前位置保持目标。
    if(dry_run_ || !tf_ready_ || !pose_received_)
        return false;

    current_target_position_.header.stamp = this->now();
    current_target_position_.header.frame_id = map_frame_;
    current_target_position_.transform.translation.x = current_x_;
    current_target_position_.transform.translation.y = current_y_;
    current_target_position_.transform.translation.z = current_z_;
    target_pose_pub_->publish(current_target_position_);

    std_msgs::msg::Bool disabled;
    disabled.data = false;
    nav_state_pub_->publish(disabled);
    sls_goal_control_pub_->publish(disabled);
    passing_door_state_pub_->publish(disabled);
    turning_state_pub_->publish(disabled);
    if_nav = false;
    current_passing_door_ = false;
    if_turning = false;
    return true;
}

bool BehaviorControl::use_sls_goal_navigation() const
{
    return navigation_execution_mode_ == "sls_goal";
}

void BehaviorControl::send_navigation_goal(
    const nav2_msgs::action::NavigateToPose::Goal & goal)
{
    if(use_sls_goal_navigation())
    {
        // SLS 只需要最终目标。header.stamp 在这里刷新，便于下游拒绝陈旧目标；
        // 不复用 /goal_pose，避免与 RViz/Nav2 的人工目标入口混淆。
        auto sls_goal = goal.pose;
        sls_goal.header.stamp = this->now();
        sls_nav_goal_pub_->publish(sls_goal);
        sls_goal_control_active_ = true;
        return;
    }
    // action 调用保持异步；Nav2 尚未启动时丢弃本次目标并节流告警，
    // 不能在决策 tick 中同步等待 server。
    if(dry_run_)
        return;
    if(!navigate_to_pose_client_->action_server_is_ready())
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "navigate_to_pose action server is unavailable; goal was not sent");
        return;
    }
    navigate_to_pose_client_->async_send_goal(goal);
}

void BehaviorControl::handle_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    if(!dry_run_ && !tf_ready_)
    {
        response->success = false;
        response->message = "mission start rejected: livox -> camera_link TF is missing";
        return;
    }
    const int previous_step = current_step;
    response->success = mission_runtime_.start();
    if(response->success)
    {
        reset_mission_context();
        response->message = "mission started at " + mission_state_name(current_step);
        publish_mission_event(previous_step, current_step, "start_requested");
    }
    else
        response->message = "mission start rejected while execution_state=" + mission_runtime_.state_name();
}

void BehaviorControl::handle_pause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    response->success = mission_runtime_.pause();
    response->message = response->success ? "mission paused" : "mission is not running";
    if(response->success)
    {
        const bool hold_published = publish_safe_hold();
        response->message = hold_published ?
            "mission paused; current-position hold published" :
            "mission paused; command output stopped (hold unavailable in dry-run or without pose/TF)";
        publish_mission_event(current_step, current_step, "pause_requested");
    }
}

void BehaviorControl::handle_resume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    response->success = mission_runtime_.resume();
    response->message = response->success ? "mission resumed" : "mission is not paused";
    if(response->success)
        publish_mission_event(current_step, current_step, "resume_requested");
}

void BehaviorControl::handle_step_once(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    response->success = mission_runtime_.request_step_once();
    response->message = response->success ? "one decision tick queued" : "step_once requires a paused mission";
    if(response->success)
        publish_mission_event(current_step, current_step, "step_once_requested");
}

void BehaviorControl::handle_abort(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    response->success = mission_runtime_.abort();
    response->message = response->success ? "mission aborted" : "mission already aborted";
    if(response->success)
    {
        const bool hold_published = publish_safe_hold();
        response->message += hold_published ?
            "; current-position hold published" :
            "; command output stopped (hold unavailable in dry-run or without pose/TF)";
        publish_mission_event(current_step, current_step, "abort_requested");
    }
}

void BehaviorControl::handle_dump_context(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    const auto * current_node = mission_graph_.node_by_legacy_step(current_step);
    std::ostringstream json;
    json << std::boolalpha
         << "{\"execution_state\":\"" << mission_runtime_.state_name()
         << "\",\"state\":\"" << mission_state_name(current_step)
         << "\",\"legacy_step\":" << current_step
         << ",\"state_id\":" << (current_node == nullptr ? -1 : current_node->state_id)
         << ",\"phase\":\"" << mission_phase_name(current_step)
         << "\",\"target\":\"" << mission_target_name(current_step)
         << "\",\"dry_run\":" << dry_run_
         << ",\"tf_ready\":" << tf_ready_
         << ",\"pose_received\":" << pose_received_
         << ",\"armed\":" << arming_state
         << ",\"current_pose\":{"
         << "\"x\":" << current_x_ << ",\"y\":" << current_y_ << ",\"z\":" << current_z_ << "}"
         << ",\"if_nav\":" << if_nav
         << ",\"random_target_found\":" << if_find_random_target_
         << ",\"random_tank_found\":" << if_find_random_tank_target_
         << ",\"event_sequence\":" << mission_event_sequence_ << "}";
    response->success = true;
    response->message = json.str();
    publish_mission_event(current_step, current_step, "context_dumped");
}

bool BehaviorControl::apply_debug_jump(
    int target_state_id, bool reset_context, const std::string & reason,
    std::string & message)
{
    if(!dry_run_)
    {
        message = "hard jump rejected: dry_run must be true";
        return false;
    }

    const auto execution_state = mission_runtime_.state();
    if(execution_state != behavior_control::MissionExecutionState::kIdle &&
       execution_state != behavior_control::MissionExecutionState::kPaused)
    {
        message = "hard jump requires idle or paused execution state";
        return false;
    }

    const auto * target_node = mission_graph_.node_by_id(target_state_id);
    if(target_node == nullptr)
    {
        message = "unknown state_id=" + std::to_string(target_state_id);
        return false;
    }

    const int previous_step = current_step;
    if(reset_context)
        reset_state_local_context();
    current_step = target_node->legacy_step;
    state_enter_time_ = this->now();

    // idle 下的硬跳同时更新下一次 /mission/start 的起点；paused 下只改变当前停点。
    if(execution_state == behavior_control::MissionExecutionState::kIdle)
        configured_start_step_ = current_step;

    publish_mission_event(previous_step, current_step, reason);
    message = "jumped to state_id=" + std::to_string(target_node->state_id) +
        " state=" + target_node->name;
    return true;
}

void BehaviorControl::handle_jump_to_state(
    const std::shared_ptr<robot_interfaces::srv::JumpMissionState::Request> request,
    std::shared_ptr<robot_interfaces::srv::JumpMissionState::Response> response)
{
    response->success = apply_debug_jump(
        request->state_id, request->reset_context, "operator_jump_absolute", response->message);
    const auto * resulting_node = mission_graph_.node_by_legacy_step(current_step);
    response->resulting_state_id = resulting_node == nullptr ? -1 : resulting_node->state_id;
    response->resulting_state = mission_state_name(current_step);
}

void BehaviorControl::handle_shift_state(
    const std::shared_ptr<robot_interfaces::srv::ShiftMissionState::Request> request,
    std::shared_ptr<robot_interfaces::srv::ShiftMissionState::Response> response)
{
    const auto * current_node = mission_graph_.node_by_legacy_step(current_step);
    if(request->delta == 0)
    {
        response->success = false;
        response->message = "delta must not be zero";
    }
    else if(current_node == nullptr)
    {
        response->success = false;
        response->message = "current legacy step is not registered in mission graph";
    }
    else
    {
        const auto * target = mission_graph_.shift_state(current_node->state_id, request->delta);
        if(target == nullptr)
        {
            response->success = false;
            response->message = "relative jump exceeds debug-order boundary";
        }
        else
        {
            response->success = apply_debug_jump(
                target->state_id, request->reset_context,
                request->delta > 0 ? "operator_jump_forward" : "operator_jump_backward",
                response->message);
        }
    }

    const auto * resulting_node = mission_graph_.node_by_legacy_step(current_step);
    response->resulting_state_id = resulting_node == nullptr ? -1 : resulting_node->state_id;
    response->resulting_state = mission_state_name(current_step);
}

void BehaviorControl::CurrentPoseCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    current_x_ = msg->transform.translation.x;
    current_y_ = msg->transform.translation.y;
    current_z_ = msg->transform.translation.z + 0.39;
    pose_received_ = true;

    map_to_livox.header.stamp = msg->header.stamp;
    map_to_livox.header.frame_id = "map";
    map_to_livox.child_frame_id = "livox";
    map_to_livox.transform.translation.x = current_x_;
    map_to_livox.transform.translation.y = current_y_;
    map_to_livox.transform.translation.z = current_z_;
    map_to_livox.transform.rotation.x = msg->transform.rotation.x;
    map_to_livox.transform.rotation.y = msg->transform.rotation.y;
    map_to_livox.transform.rotation.z = msg->transform.rotation.z;
    map_to_livox.transform.rotation.w = msg->transform.rotation.w;

    if(!tf_ready_)
    {
        try
        {
            livox_to_camera = tf_buffer_->lookupTransform(
                "livox", "camera_link", rclcpp::Time(), rclcpp::Duration::from_seconds(0.0));
            livox_to_camera_affine = tf2::transformToEigen(livox_to_camera);
            tf_ready_ = true;
            publish_mission_event(current_step, current_step, "required_tf_available");
        }
        catch(const tf2::TransformException &)
        {
        }
    }

    map_to_livox_affine = tf2::transformToEigen(map_to_livox);
    map_to_camera_affine = map_to_livox_affine * livox_to_camera_affine;

    map_to_camera.header.stamp = msg->header.stamp;
    map_to_camera.header.frame_id = "map";
    map_to_camera.child_frame_id = "camera_link";
    map_to_camera.transform.translation.x = map_to_camera_affine.translation().x();
    map_to_camera.transform.translation.y = map_to_camera_affine.translation().y();
    map_to_camera.transform.translation.z = map_to_camera_affine.translation().z();
    Eigen::Matrix3d rotation_matrix = map_to_camera_affine.rotation();
    Eigen::Quaterniond base_quat_result(rotation_matrix);
    map_to_camera.transform.rotation.x = base_quat_result.x();
    map_to_camera.transform.rotation.y = base_quat_result.y();
    map_to_camera.transform.rotation.z = base_quat_result.z();
    map_to_camera.transform.rotation.w = base_quat_result.w();
}

void BehaviorControl::USBCameraInfoCallback(const robot_interfaces::msg::ImageLocation::SharedPtr msg) //usb相机发过来的是livox系
{
    if(current_step == 112 || current_step == 114)
    {
        if(msg->id == 6)
        {
            if_find_random_target_ = true;
            geometry_msgs::msg::PointStamped camera_point, world_point;
            camera_point.header.frame_id = "livox";
            camera_point.header.stamp = this->now();
            camera_point.point.x = msg->image_x;
            camera_point.point.y = msg->image_y;
            camera_point.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_point, world_point, map_to_livox);

                random_target_[0] = world_point.point.x;
                random_target_[1] = world_point.point.y;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in USBCameraInfoCallback: %s", ex.what());
            }
        }
        if(msg->id == 5)
        {
            RCLCPP_INFO(this->get_logger(), "USB Camera detected!!!!!");
            if_find_random_tank_target_ = true;
            geometry_msgs::msg::PointStamped camera_point, world_point;
            camera_point.header.frame_id = "livox";
            camera_point.header.stamp = this->now();
            camera_point.point.x = msg->image_x;
            camera_point.point.y = msg->image_y;
            camera_point.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_point, world_point, map_to_livox);

                random_tank_target_[0] = world_point.point.x;
                random_tank_target_[1] = world_point.point.y;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in USBCameraInfoCallback: %s", ex.what());
            }

            //
            for (std::size_t i = 0; i < target_positions_.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(), "1111111111111111111111111");
                if ((random_tank_target_[0] - target_positions_[target_sequence_[i]][0])*(random_tank_target_[0] - target_positions_[target_sequence_[i]][0]) +
                    (random_tank_target_[1] - target_positions_[target_sequence_[i]][1])*(random_tank_target_[1] - target_positions_[target_sequence_[i]][1]) <= 1.0)
                {
                    RCLCPP_INFO(this->get_logger(), "tank与其他目标点重合!!!!!!!!");
                    if_find_random_tank_target_ = false;
                }
            }
        }
    }
}

void BehaviorControl::start_takeoff_circle_if_needed()
{
    if(takeoff_circle_started_)
        return;

    // 只在首次进入时锁定圆心，使 Point-LIO 后续轻微抖动也不会改变操作员
    // 看到的圆周轨迹；第一个目标点落在当前位置。
    takeoff_circle_start_time_ = this->now();
    takeoff_circle_center_x_ = current_x_ - takeoff_circle_radius_;
    takeoff_circle_center_y_ = current_y_;
    takeoff_circle_started_ = true;

    RCLCPP_INFO(this->get_logger(),
        "起飞确认，开始匀速圆周运动: radius=%lf, speed=%lf, direction=%lf, center=(%lf, %lf)",
        takeoff_circle_radius_, takeoff_circle_speed_, takeoff_circle_direction_,
        takeoff_circle_center_x_, takeoff_circle_center_y_);
}

void BehaviorControl::publish_takeoff_circle_target()
{
    // 圆周参数非法时退化为原地悬停目标，不进入后续旧任务流程。
    if(takeoff_circle_radius_ < kMinCircleRadius || takeoff_circle_speed_ < kMinCircleSpeed)
    {
        current_target_position_.transform.translation.x = current_x_;
        current_target_position_.transform.translation.y = current_y_;
        current_target_position_.transform.translation.z = cruise_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        return;
    }

    start_takeoff_circle_if_needed();

    // 用 omega = v / r 把固定线速度转换成角度推进量。
    double elapsed = (this->now() - takeoff_circle_start_time_).seconds();
    double angle_abs = std::fmod(takeoff_circle_speed_ / takeoff_circle_radius_ * elapsed, kTwoPi);

    double angle = takeoff_circle_direction_ * angle_abs;
    current_target_position_.header.stamp = this->now();
    current_target_position_.header.frame_id = map_frame_;
    current_target_position_.child_frame_id = target_frame_;
    current_target_position_.transform.translation.x =
        takeoff_circle_center_x_ + takeoff_circle_radius_ * std::cos(angle);
    current_target_position_.transform.translation.y =
        takeoff_circle_center_y_ + takeoff_circle_radius_ * std::sin(angle);
    current_target_position_.transform.translation.z = cruise_height_;
    current_target_position_.transform.rotation.x = 0.0;
    current_target_position_.transform.rotation.y = 0.0;
    current_target_position_.transform.rotation.z = 0.0;
    current_target_position_.transform.rotation.w = 1.0;
    target_pose_pub_->publish(current_target_position_);

    if_nav = false;
    current_passing_door_ = false;
    if_turning = false;
}

void BehaviorControl::ImageLocationCallback(const robot_interfaces::msg::ImageLocation::SharedPtr msg) //d435发过来是camera_link系
{
    detected_target_id_ = msg->id;
    if(detected_target_id_ == 6) //十字随机目标
    {
        if (!if_find_random_target_)
        {
            if_find_random_target_ = true;
            detected_target_[0] = msg->image_x;
            detected_target_[1] = msg->image_y;

            geometry_msgs::msg::PointStamped camera_point, world_point;
            camera_point.header.frame_id = "camera_link";
            camera_point.header.stamp = this->now();
            camera_point.point.x = msg->image_x;
            camera_point.point.y = msg->image_y;
            camera_point.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_point, world_point, map_to_camera);

                random_target_[0] = world_point.point.x;
                random_target_[1] = world_point.point.y;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in ImageLocationCallback: %s", ex.what());
            }
        }
    }
    else if(detected_target_id_ == 5)  //tank随机目标
    {
        RCLCPP_INFO(this->get_logger(), "!!!!!!!!Detected tank!!!!!");
        if (!if_find_random_tank_target_)
        {
            RCLCPP_INFO(this->get_logger(), "----------------------");
            if_find_random_tank_target_ = true;
            detected_target_[0] = msg->image_x;
            detected_target_[1] = msg->image_y;

            geometry_msgs::msg::PointStamped camera_point, world_point;
            camera_point.header.frame_id = "camera_link";
            camera_point.header.stamp = this->now();
            camera_point.point.x = msg->image_x;
            camera_point.point.y = msg->image_y;
            camera_point.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_point, world_point, map_to_camera);

                random_tank_target_[0] = world_point.point.x;
                random_tank_target_[1] = world_point.point.y;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in ImageLocationCallback: %s", ex.what());
            }
            //
            for (std::size_t i = 0; i < target_positions_.size(); i++)
            {
                if ((random_tank_target_[0] - target_positions_[target_sequence_[i]][0])*(random_tank_target_[0] - target_positions_[target_sequence_[i]][0]) +
                    (random_tank_target_[1] - target_positions_[target_sequence_[i]][1])*(random_tank_target_[1] - target_positions_[target_sequence_[i]][1]) <= 1.0)
                {
                    RCLCPP_INFO(this->get_logger(), "??????目标点重合");
                    if_find_random_tank_target_ = false;
                }
            }
        }
    }
    else
    {
        if(detected_target_id_ == 2)  //排除tank和bridge误识别情况
        {
            // 先转坐标系
            geometry_msgs::msg::PointStamped camera_point, world_point;
            double curr_detected_target[2];
            camera_point.header.frame_id = "camera_link";
            camera_point.header.stamp = this->now();
            camera_point.point.x = msg->image_x;
            camera_point.point.y = msg->image_y;
            camera_point.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_point, world_point, map_to_camera);

                curr_detected_target[0] = world_point.point.x;
                curr_detected_target[1] = world_point.point.y;
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in ImageLocationCallback: %s", ex.what());
            }

            if((curr_detected_target[0] - bridge_[0]) * (curr_detected_target[0] - bridge_[0]) +
                (curr_detected_target[1] - bridge_[1]) * (curr_detected_target[1] - bridge_[1]) >= 1.0) //实际是tank
            {
                if_find_random_tank_target_ = true;
                random_tank_target_[0] = curr_detected_target[0];
                random_tank_target_[1] = curr_detected_target[1];
            }
        }
        detected_target_[0] = msg->image_x;
        detected_target_[1] = msg->image_y;
    }
}

void BehaviorControl::step_timer_callback()
{
    // running 时持续放行；paused 时只有 /mission/step_once 能放行一次。
    if(!mission_runtime_.consume_decision_tick())
        return;

    const int previous_step = current_step;
    RCLCPP_INFO(this->get_logger(), "---------->current step: %d", current_step);
    if(current_step == 0) //等待飞控解锁
    {
        if(arming_state)
            current_step = 1;
    }
    else if(current_step == 300)
    {
    RCLCPP_INFO(this->get_logger(),"------------------模拟拉高");
        if(fabs(current_z_ - cruise_height_) <= 0.05)
        {
            RCLCPP_INFO(this->get_logger(),"------------------拉高结束，高度：%f",current_z_);
        }
    }
    else if(current_step == 1) //等待起飞至巡航高度
    {
        if(fabs(current_z_ - cruise_height_) <= 0.05)
        {
            // 当前重构调试阶段的截断点：确认起飞后只进入圆周运动。
            if(takeoff_circle_enabled_)
                current_step = kTakeoffCircleStep;
            else
                current_step = 111;  //current_step=61;
        }
    }
    else if(current_step == kTakeoffCircleStep) //起飞后持续执行匀速圆周运动
    {
        // 不再推进 current_step，确保不会误入后续投掷/穿门流程。
        start_takeoff_circle_if_needed();
    }
    //在起飞点左右两侧移动，寻找随机靶
    else if(current_step == 111)
    {
        if(fabs(current_x_ - random_target_init_search_1_[0]) <= 0.2)
            if(fabs(current_y_ - random_target_init_search_1_[1]) <= 0.2)
            {
                current_step = 112;
            }
    }
    else if(current_step == 112)
    {
        if(if_find_random_target_ && if_find_random_tank_target_) //找到随机靶
        {
            detection_cnt = 0;
            current_step = 21;
            for(int i = 0; i < 3; i++)
            {
                std_msgs::msg::Bool camera_choose_msg;
                camera_choose_msg.data = true;
                if(!dry_run_)
                    camera_choose_pub_->publish(camera_choose_msg);
                RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>使用d435<<<<<<<<<<<<<<<<<<<");
            }
        }
        detection_cnt++;
        if(detection_cnt >= dynamic_detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 113;
        }
    }
    else if(current_step == 113)
    {
        if(fabs(current_x_ - random_target_init_search_2_[0]) <= 0.2)
            if(fabs(current_y_ - random_target_init_search_2_[1]) <= 0.2)
            {
                current_step = 114;
            }
    }
    else if(current_step == 114)
    {
        if(if_find_random_target_ && if_find_random_tank_target_) //找到随机靶
        {
            detection_cnt = 0;
            current_step = 21;
            for(int i = 0; i < 3; i++)
            {
                std_msgs::msg::Bool camera_choose_msg;
                camera_choose_msg.data = true;
                if(!dry_run_)
                    camera_choose_pub_->publish(camera_choose_msg);
                RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>使用d435<<<<<<<<<<<<<<<<<<<");
            }
        }
        detection_cnt++;
        if(detection_cnt >= dynamic_detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 21;
            for(int i = 0; i < 3; i++)
            {
                std_msgs::msg::Bool camera_choose_msg;
                camera_choose_msg.data = true;
                if(!dry_run_)
                    camera_choose_pub_->publish(camera_choose_msg);
                RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>使用d435<<<<<<<<<<<<<<<<<<<");
            }
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
        if (fabs(current_z_ - detection_height_) < 0.2)
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
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];
        }
    }
    else if(current_step == 24) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            if(if_need_passing_all_) //需要遍历静态靶
                current_step = 31;
            else
                current_step = 91; //不需要遍历静态靶
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
        if (fabs(current_z_ - detection_height_) < 0.2)
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
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];
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
        if (fabs(current_z_ - detection_height_) < 0.2)
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
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];
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
        if (fabs(current_z_ - detection_height_) < 0.2)
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
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];
        }
    }
    else if(current_step == 54) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 91;
        }
    }
    //进入搜索随机靶任务
    else if(current_step == 91) //搜索第一个点
    {
        if(if_find_random_target_ && if_find_random_tank_target_)
        {
            current_step = 61; //找到2个随机靶
            RCLCPP_INFO(this->get_logger(), "----------找到2个随机靶--------");
            state_enter_time_ = this->now();
            publish_mission_event(previous_step, current_step, "random_targets_found");
            return;
        }
        if(fabs(current_x_ - random_target_search_1_[0]) < 0.15)
            if(fabs(current_y_ - random_target_search_1_[1]) < 0.15)
            {
                current_step = 92;
            }
    }
    else if(current_step == 92) //搜索第二个点
    {
        if(if_find_random_target_ && if_find_random_tank_target_)
        {
            current_step = 61; //找到2个随机靶
            state_enter_time_ = this->now();
            publish_mission_event(previous_step, current_step, "random_targets_found");
            return;
        }
        if(fabs(current_x_ - random_target_search_2_[0]) < 0.15)
            if(fabs(current_y_ - random_target_search_2_[1]) < 0.15)
            {
                current_step = 93;
            }
    }
    else if(current_step == 93) //搜索第三个点
    {
        if(fabs(current_x_ - random_target_search_3_[0]) < 0.15)
            if(fabs(current_y_ - random_target_search_3_[1]) < 0.15)
            {
                if(!if_find_random_tank_target_)
                {
                    random_tank_target_[0] = prev_random_tank_target_[0];
                    random_tank_target_[1] = prev_random_tank_target_[1];
                }
                current_step = 61;
                RCLCPP_INFO(this->get_logger(), "random_target: %lf, %lf", random_target_[0], random_target_[1]);
                RCLCPP_INFO(this->get_logger(), "random_tank_target: %lf, %lf", random_tank_target_[0], random_tank_target_[1]);
            }
    }
    //进入动态目标点循环（逻辑待改）
    else if (current_step == 61)
    {
        if(!if_find_random_tank_target_)
        {
            random_tank_target_[0] = prev_random_tank_target_[0];
            random_tank_target_[1] = prev_random_tank_target_[1];
        }
        if (fabs(current_x_ - random_tank_target_[0]) < 0.2) //是否到目标点附近
            if (fabs(current_y_ - random_tank_target_[1]) < 0.2)
                current_step = 62;
    }
    else if(current_step == 62)  //拉高
    {
        if (fabs(current_z_ - detection_height_) < 0.2)
        {
            current_step = 63;
        }
    }
    else if(current_step == 63) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 64;
            if(if_find_random_tank_target_)
            {
                stored_point[0] = random_tank_target_[0];
                stored_point[1] = random_tank_target_[1];
            }
            else
            {
                stored_point[0] = detected_target_[0];
                stored_point[1] = detected_target_[1];
            }
        }
    }
    else if(current_step == 64) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 101; //进入随机靶投掷任务
        }
    }
    //进入随机靶投掷任务
    else if(current_step == 101) //是否到目标点附近
    {
        if(!if_find_random_tank_target_)
        {
            random_target_[0] = prev_random_target_[0];
            random_target_[1] = prev_random_target_[1];
        }
        if(fabs(current_x_ - random_target_[0]) < 0.2)
            if(fabs(current_y_ - random_target_[1]) < 0.2)
            {
                current_step = 102;
            }
    }
    else if(current_step == 102) //拉高
    {
        if (fabs(current_z_ - detection_height_) < 0.2)
        {
            current_step = 103;
        }
    }
    else if(current_step == 103) //进行识别
    {
        detection_cnt++;
        if(detection_cnt >= detection_cnt_threshold_)
        {
            detection_cnt = 0;
            current_step = 104;
            if(if_find_random_target_)
            {
                stored_point[0] = random_target_[0];
                stored_point[1] = random_target_[1];
            }
            else
            {
                stored_point[0] = detected_target_[0];
                stored_point[1] = detected_target_[1];
            }
        }
    }
    else if(current_step == 104) //下降投掷
    {
        eject_cnt++;
        if(eject_cnt >= eject_cnt_threshold_) //计时
        {
            eject_cnt = 0;
            current_step = 71; //进入穿门任务
        }
    }
    //进入穿门任务
    else if(current_step == 71) //去穿门起点
    {
        if(fabs(current_x_ - passing_door_src_1_[0]) < 0.15)
            if(fabs(current_y_ - passing_door_src_1_[1]) < 0.15)
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
        if(turning_cnt >= 23)
        {
            std_msgs::msg::Bool clear_state_msg;
            clear_state_msg.data = true;
            if(!dry_run_)
                clear_state_pub_->publish(clear_state_msg);
        }
        if (turning_cnt >= turning_cnt_threshold_)
        {
            std_msgs::msg::Bool clear_state_msg;
            clear_state_msg.data = false;
            if(!dry_run_)
                clear_state_pub_->publish(clear_state_msg);

            turning_cnt = 0;
            current_step = 74;
        }
    }
    else if(current_step == 73) //导航至穿门中间点
    {
        if(fabs(current_x_ - passing_door_src_2_[0]) < 0.1)
            if(fabs(current_y_ - passing_door_src_2_[1]) < 0.1)
            {
                current_step = 74;
            }
    }
    else if(current_step == 74) //导航至穿门终点
    {
        if(fabs(current_x_ - passing_door_des_[0]) < 0.12)
            if(fabs(current_y_ - passing_door_des_[1]) < 0.12)
            {
                current_step = 82; //进入降落穿门后终点状态
            }
    }

    else if(current_step == 75) //不穿门直接回起点
    {
        if(fabs(current_x_) < 0.1)
            if(fabs(current_y_) < 0.1)
            {
                current_step = 81; //进入降落起点状态
            }
    }

    if(current_step != previous_step)
    {
        // 旧代码仍直接修改 current_step，因此在单次判断结束后统一检测转移，
        // 为每次变化补齐命名事件和状态进入时间。
        state_enter_time_ = this->now();
        publish_mission_event(previous_step, current_step, "legacy_transition_condition_met");
    }
}

void BehaviorControl::mission_timer_callback()
{
    // 暂停、中止、idle 和 dry-run 都不允许进入旧动作发布分支。
    if(!mission_runtime_.commands_enabled() || dry_run_)
        return;
    if(!tf_ready_)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "mission command output is gated until livox -> camera_link TF is available");
        return;
    }

    // 每个命令周期先撤销 SLS 源，只有本周期确实给出最终目标时才在
    // send_navigation_goal() 中重新置位。这使抬升、识别、投放等状态不会
    // 继续使用上一个导航目标。
    sls_goal_control_active_ = false;

    RCLCPP_INFO(this->get_logger(), "+++++++++++++++++tank position: %lf, %lf+++++++++++++++++",
        random_tank_target_[0], random_tank_target_[1]);
    RCLCPP_INFO(this->get_logger(), "+++++++++++++++if_find_random_tank_target: %d+++++++++++++++++",if_find_random_tank_target_);
    if(current_step == 0)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        current_target_position_.transform.translation.z = 0.0;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        current_passing_door_ = false;
        RCLCPP_INFO(this->get_logger(), "等待飞控解锁...");
        obstacle_height_ = 2.0;
    }
    else if(current_step == 300)
    {
        current_target_position_.transform.translation.x = 0.0;
        current_target_position_.transform.translation.y = 0.0;
        current_target_position_.transform.translation.z = cruise_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "等待起飞至巡航高度...");
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
    else if(current_step == kTakeoffCircleStep)
    {
        // 圆周目标走 /robot/target_pose，再由 flight_control/mavros_adapter
        // 转成 MAVROS setpoint_position/local。
        publish_takeoff_circle_target();
    }
    else if(current_step == 111)
    {
        current_target_position_.transform.translation.x = random_target_init_search_1_[0];
        current_target_position_.transform.translation.y = random_target_init_search_1_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;

        // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        // action_goal.pose.header.frame_id = "map";
        // action_goal.pose.pose.position.x = random_target_init_search_1_[0];
        // action_goal.pose.pose.position.y = random_target_init_search_1_[1];
        // action_goal.pose.pose.position.z = cruise_height_;
        // action_goal.pose.pose.orientation.x = 0.0;
        // action_goal.pose.pose.orientation.y = 0.0;
        // action_goal.pose.pose.orientation.z = 0.0;
        // action_goal.pose.pose.orientation.w = 1.0;
        // navigate_to_pose_client_->async_send_goal(action_goal);
        // if_nav = true;
    }
    else if(current_step == 112)
    {
        current_target_position_.transform.translation.x = random_target_init_search_1_[0];
        current_target_position_.transform.translation.y = random_target_init_search_1_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "第一个随机靶搜索点");
    }
    else if(current_step == 113)
    {
        current_target_position_.transform.translation.x = random_target_init_search_2_[0];
        current_target_position_.transform.translation.y = random_target_init_search_2_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        // rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        // action_goal.pose.header.frame_id = "map";
        // action_goal.pose.pose.position.x = random_target_init_search_2_[0];
        // action_goal.pose.pose.position.y = random_target_init_search_2_[1];
        // action_goal.pose.pose.position.z = cruise_height_;
        // action_goal.pose.pose.orientation.x = 0.0;
        // action_goal.pose.pose.orientation.y = 0.0;
        // action_goal.pose.pose.orientation.z = 0.0;
        // action_goal.pose.pose.orientation.w = 1.0;
        // navigate_to_pose_client_->async_send_goal(action_goal);
        // if_nav = true;
    }
    else if(current_step == 114)
    {
        current_target_position_.transform.translation.x = random_target_init_search_2_[0];
        current_target_position_.transform.translation.y = random_target_init_search_2_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "第二个随机靶搜索点");
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
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
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
        RCLCPP_INFO(this->get_logger(), "拉高中...current_target_z: %f", current_target_position_.transform.translation.z);
    }
    else if(current_step == 23)  // 进行识别
	{
        camera_pt_.header.stamp = this->now();
        camera_pt_.point.x = detected_target_[0];
        camera_pt_.point.y = detected_target_[1];

        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_1_;
            current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_1_;
            current_target_position_.transform.translation.z = detection_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (相机发来map坐标: %.2f, %.2f, %.2f)",
                        world_pt_.point.x, world_pt_.point.y, detection_height_);
            RCLCPP_INFO(this->get_logger(), "识别中... (预设map坐标: %.2f, %.2f, %.2f)",
                        target_positions_[target_sequence_[0]][0], target_positions_[target_sequence_[0]][1], detection_height_);
            RCLCPP_INFO(this->get_logger(), "识别中... (飞机自身坐标: %.2f, %.2f)",
                                current_x_, current_y_);
            RCLCPP_INFO(this->get_logger(), "识别中... (转换前坐标: %.2f, %.2f)",
                        camera_pt_.point.x, camera_pt_.point.y);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 23: %s", ex.what());
        }

        if_nav = false;
    }
    else if(current_step == 24)  // 下降投掷
    {
        camera_pt_.header.stamp = this->now();
        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_1_;
            current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_1_;

            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "下降投掷目标(map): %.2f, %.2f, %.2f",
                        world_pt_.point.x, world_pt_.point.y, current_target_position_.transform.translation.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 24: %s", ex.what());
        }
        if(eject_cnt >= 13)
        {
            servo_index_ = 1;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
        if_nav = false;
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
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
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
        camera_pt_.header.stamp = this->now();

        camera_pt_.point.x = detected_target_[0];
        camera_pt_.point.y = detected_target_[1];

        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;
            current_target_position_.transform.translation.z = detection_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (map坐标: %.2f, %.2f, %.2f)",
                        world_pt_.point.x, world_pt_.point.y, detection_height_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 33: %s", ex.what());
        }

        if_nav = false;
    }
    else if(current_step == 34)  // 下降投掷
    {
        camera_pt_.header.stamp = this->now();
        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;

            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "下降投掷目标(map): %.2f, %.2f, %.2f",
                        world_pt_.point.x, world_pt_.point.y, current_target_position_.transform.translation.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 34: %s", ex.what());
        }
        if(eject_cnt >= 13)
        {
            servo_index_ = 1;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
        if_nav = false;
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
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
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
        camera_pt_.header.stamp = this->now();

        camera_pt_.point.x = detected_target_[0];
        camera_pt_.point.y = detected_target_[1];

        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;
            current_target_position_.transform.translation.z = detection_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (map坐标: %.2f, %.2f, %.2f)",
                        world_pt_.point.x, world_pt_.point.y, detection_height_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 43: %s", ex.what());
        }

        if_nav = false;
    }
    else if(current_step == 44)  // 下降投掷
    {
        camera_pt_.header.stamp = this->now();
        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;

            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "下降投掷目标(map): %.2f, %.2f, %.2f",
                        world_pt_.point.x, world_pt_.point.y, current_target_position_.transform.translation.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 44: %s", ex.what());
        }
        if(eject_cnt >= 13)
        {
            servo_index_ = 1;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
        if_nav = false;
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
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
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
        camera_pt_.header.stamp = this->now();

        camera_pt_.point.x = detected_target_[0];
        camera_pt_.point.y = detected_target_[1];

        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;
            current_target_position_.transform.translation.z = detection_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (map坐标: %.2f, %.2f, %.2f)",
                        world_pt_.point.x, world_pt_.point.y, detection_height_);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 53: %s", ex.what());
        }

        if_nav = false;
    }
    else if(current_step == 54)  // 下降投掷
    {
        camera_pt_.header.stamp = this->now();
        try
        {
            tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

            current_target_position_.transform.translation.x = world_pt_.point.x;
            current_target_position_.transform.translation.y = world_pt_.point.y;

            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "下降投掷目标(map): %.2f, %.2f, %.2f",
                        world_pt_.point.x, world_pt_.point.y, current_target_position_.transform.translation.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed in step 54: %s", ex.what());
        }
        if(eject_cnt >= 13)
        {
            servo_index_ = 1;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
        if_nav = false;
    }
//----------------------动态靶妹写完----------------------
    else if(current_step == 61)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
		action_goal.pose.header.frame_id = "map";
        if(!if_find_random_tank_target_)
        {
            random_tank_target_[0] = prev_random_tank_target_[0];
            random_tank_target_[1] = prev_random_tank_target_[1];
        }
        action_goal.pose.pose.position.x = random_tank_target_[0];
        action_goal.pose.pose.position.y = random_tank_target_[1];
        action_goal.pose.pose.position.z = detection_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
        RCLCPP_INFO(this->get_logger(), "tank目标点: %lf, %lf", random_tank_target_[0], random_tank_target_[1]);
    }
    else if(current_step == 62)
  	{
  	    current_target_position_.transform.translation.x = random_tank_target_[0];
        current_target_position_.transform.translation.y = random_tank_target_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
 	}
    else if(current_step == 63)
    {
        camera_pt_.header.stamp = this->now();
        if(if_find_random_tank_target_)
        {
            current_target_position_.transform.translation.x = random_tank_target_[0] + offset_x_2_;
            current_target_position_.transform.translation.y = random_tank_target_[1] + offset_y_2_;
            current_target_position_.transform.translation.z = detection_height_;
            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (相机发来map坐标: %.2f, %.2f)",
                                random_tank_target_[0], random_tank_target_[1]);
        }
        else
        {
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];
            try
            {
                tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

                current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_2_;
                current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_2_;
                current_target_position_.transform.translation.z = detection_height_;
                target_pose_pub_->publish(current_target_position_);
                RCLCPP_INFO(this->get_logger(), "识别中... (相机发来map坐标: %.2f, %.2f)",
                                    world_pt_.point.x, world_pt_.point.y);
                // RCLCPP_INFO(this->get_logger(), "识别中... (飞机自身坐标: %.2f, %.2f)",
                //                 current_x_, current_y_);
                RCLCPP_INFO(this->get_logger(), "识别中... (转换前坐标: %.2f, %.2f)",
                            camera_pt_.point.x, camera_pt_.point.y);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in step 63: %s", ex.what());
            }
        }

        if_nav = false;
    }
    else if(current_step == 64) //下降投掷
    {
        camera_pt_.header.stamp = this->now();
        if(if_find_random_tank_target_)
        {
            current_target_position_.transform.translation.x = stored_point[0] + offset_x_2_;
            current_target_position_.transform.translation.y = stored_point[1] + offset_y_2_;
            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
        }
        else
        {
            camera_pt_.point.x = stored_point[0];
            camera_pt_.point.y = stored_point[1];
            camera_pt_.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

                current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_2_;
                current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_2_;
                if (current_z_ - eject_height_ >= 0.6)
                    current_target_position_.transform.translation.z = current_z_ - 0.6;
                else
                    current_target_position_.transform.translation.z = eject_height_;

                target_pose_pub_->publish(current_target_position_);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in step 63: %s", ex.what());
            }
        }

        if(eject_cnt >= 13)
        {
            servo_index_ = 2;
            RCLCPP_INFO(this->get_logger(), "下降投掷，第 %d 个投放位", servo_index_);
            if (servo_index_ == last_servo_index_)
                return;
            last_servo_index_ = servo_index_;
            set_parameter();
        }
        if_nav = false;
    }

    else if(current_step == 91)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = random_target_search_1_[0];
        action_goal.pose.pose.position.y = random_target_search_1_[1];
        action_goal.pose.pose.position.z = detection_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
        RCLCPP_INFO(this->get_logger(), "第一个搜索点: %lf, %lf", random_target_search_1_[0], random_target_search_1_[1]);
    }
    else if(current_step == 92)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = random_target_search_2_[0];
        action_goal.pose.pose.position.y = random_target_search_2_[1];
        action_goal.pose.pose.position.z = detection_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
        RCLCPP_INFO(this->get_logger(), "第二个搜索点: %lf, %lf", random_target_search_2_[0], random_target_search_2_[1]);
    }
    else if(current_step == 93)
    {
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = random_target_search_3_[0];
        action_goal.pose.pose.position.y = random_target_search_3_[1];
        action_goal.pose.pose.position.z = detection_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
        RCLCPP_INFO(this->get_logger(), "第三个搜索点: %lf, %lf", random_target_search_3_[0], random_target_search_3_[1]);
    }

    else if(current_step == 101)
    {
        if(!if_find_random_tank_target_)
        {
            random_target_[0] = prev_random_target_[0];
            random_target_[1] = prev_random_target_[1];
        }
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal action_goal;
        action_goal.pose.header.frame_id = "map";
        action_goal.pose.pose.position.x = random_target_[0];
        action_goal.pose.pose.position.y = random_target_[1];
        action_goal.pose.pose.position.z = cruise_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
        RCLCPP_INFO(this->get_logger(), "---------随机靶: %lf, %lf", random_target_[0], random_target_[1]);
    }
    else if(current_step == 102)
    {
        current_target_position_.transform.translation.x = random_target_[0];
        current_target_position_.transform.translation.y = random_target_[1];
        current_target_position_.transform.translation.z = detection_height_;
        target_pose_pub_->publish(current_target_position_);
        if_nav = false;
        RCLCPP_INFO(this->get_logger(), "拉高中...");
    }
    else if(current_step == 103)
    {
        camera_pt_.header.stamp = this->now();
        if(if_find_random_target_)
        {
            current_target_position_.transform.translation.x = random_target_[0] + offset_x_3_;
            current_target_position_.transform.translation.y = random_target_[1] + offset_y_3_;
            current_target_position_.transform.translation.z = detection_height_;
            target_pose_pub_->publish(current_target_position_);
            RCLCPP_INFO(this->get_logger(), "识别中... (map坐标: %.2f, %.2f, %.2f)",
                        random_target_[0], random_target_[1], detection_height_);
        }
        else
        {
            camera_pt_.point.x = detected_target_[0];
            camera_pt_.point.y = detected_target_[1];

            try
            {
                tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

                current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_3_;
                current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_3_;
                current_target_position_.transform.translation.z = detection_height_;

                target_pose_pub_->publish(current_target_position_);
                RCLCPP_INFO(this->get_logger(), "识别中... (map坐标: %.2f, %.2f, %.2f)",
                            world_pt_.point.x, world_pt_.point.y, detection_height_);
                RCLCPP_INFO(this->get_logger(), "识别中... (转换前坐标: %.2f, %.2f)",
                            camera_pt_.point.x, camera_pt_.point.y);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in step 103: %s", ex.what());
            }
        }

        if_nav = false;
    }

    else if(current_step == 104)
    {
        camera_pt_.header.stamp = this->now();
        if(if_find_random_target_)
        {
            current_target_position_.transform.translation.x = stored_point[0] + offset_x_2_;
            current_target_position_.transform.translation.y = stored_point[1] + offset_y_2_;
            if (current_z_ - eject_height_ >= 0.6)
                current_target_position_.transform.translation.z = current_z_ - 0.6;
            else
                current_target_position_.transform.translation.z = eject_height_;

            target_pose_pub_->publish(current_target_position_);
        }
        else
        {
            camera_pt_.point.x = stored_point[0];
            camera_pt_.point.y = stored_point[1];
            camera_pt_.point.z = 0.0;
            try
            {
                tf2::doTransform(camera_pt_, world_pt_, map_to_camera);

                current_target_position_.transform.translation.x = world_pt_.point.x + offset_x_2_;
                current_target_position_.transform.translation.y = world_pt_.point.y + offset_y_2_;
                if (current_z_ - eject_height_ >= 0.6)
                    current_target_position_.transform.translation.z = current_z_ - 0.6;
                else
                    current_target_position_.transform.translation.z = eject_height_;

                target_pose_pub_->publish(current_target_position_);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform failed in step 63: %s", ex.what());
            }
        }

        if(eject_cnt >= 13)
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
            action_goal.pose.pose.position.x = passing_door_src_1_[0];
            action_goal.pose.pose.position.y = passing_door_src_1_[1];
            action_goal.pose.pose.position.z = passing_door_height_;
            action_goal.pose.pose.orientation.x = 0.0;
            action_goal.pose.pose.orientation.y = 0.0;
            action_goal.pose.pose.orientation.z = 0.0;
            action_goal.pose.pose.orientation.w = 1.0;
            send_navigation_goal(action_goal);
            RCLCPP_INFO(this->get_logger(), "穿门起点: %lf, %lf", passing_door_src_1_[0], passing_door_src_1_[1]);
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
            send_navigation_goal(action_goal);
            RCLCPP_INFO(this->get_logger(), "返回起点，current x y: %lf, %lf", current_x_, current_y_);
        }
        if_nav = !use_sls_goal_navigation();
        if_turning = false;
    }
    else if(current_step == 72)
    {
        obstacle_height_ = 1.4;
        current_target_position_.transform.translation.x = passing_door_src_1_[0];
        current_target_position_.transform.translation.y = passing_door_src_1_[1];
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
        action_goal.pose.pose.position.x = passing_door_src_2_[0];
        action_goal.pose.pose.position.y = passing_door_src_2_[1];
        action_goal.pose.pose.position.z = passing_door_height_;
        action_goal.pose.pose.orientation.x = 0.0;
        action_goal.pose.pose.orientation.y = 0.0;
        action_goal.pose.pose.orientation.z = 0.0;
        action_goal.pose.pose.orientation.w = 1.0;
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
        current_passing_door_ = true;
        if_turning = false;
        RCLCPP_INFO(this->get_logger(), "穿门中间点: %lf, %lf", passing_door_src_2_[0], passing_door_src_2_[1]);
    }
    else if(current_step == 74)
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
        send_navigation_goal(action_goal);
        if_nav = !use_sls_goal_navigation();
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
    std_msgs::msg::Bool sls_goal_control_msg;
    sls_goal_control_msg.data = sls_goal_control_active_;
    sls_goal_control_pub_->publish(sls_goal_control_msg);
    // RCLCPP_INFO(this->get_logger(), "<<<<<<<<<<<<<<<<<<<<<<< if_nav: %d", if_nav);
    std_msgs::msg::Bool passing_door_state_msg;
    passing_door_state_msg.data = current_passing_door_;
    passing_door_state_pub_->publish(passing_door_state_msg);
    // RCLCPP_INFO(this->get_logger(), "/////////////////////// current_passing_door_: %d", current_passing_door_);
    std_msgs::msg::Bool turning_state_msg;
    turning_state_msg.data = if_turning;
    turning_state_pub_->publish(turning_state_msg);
    // RCLCPP_INFO(this->get_logger(), "////////......... if_turning: %d", if_turning);
    std_msgs::msg::Float64 obstacle_height_msg;
    obstacle_height_msg.data = obstacle_height_;
    obstacle_height_pub_->publish(obstacle_height_msg);
}

void BehaviorControl::ArmStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    arming_state = msg->data;
    RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>arming_state: %d", arming_state);
}

void BehaviorControl::set_parameter()
{
    if(dry_run_)
        return;
    if(!servo_parameter_client_->service_is_ready())
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "servo parameter service is unavailable; eject command was not sent");
        return;
    }

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
                RCLCPP_INFO(this->get_logger(), ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Parameter set successfully");
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
    if(dry_run_)
        return;
    if(!controller_server_parameter_client_->service_is_ready() ||
       !local_costmap_parameter_client_->service_is_ready())
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Nav2 parameter services are unavailable; door profile was not changed");
        return;
    }

    /* 修改FollowPath中的参数 */
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
    rcl_interfaces::msg::Parameter controller_server_param_max_global_plan_lookahead_dist;
    controller_server_param_max_global_plan_lookahead_dist.name = "FollowPath.max_global_plan_lookahead_dist";
    controller_server_param_max_global_plan_lookahead_dist.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rcl_interfaces::msg::Parameter controller_server_param_weight_inflation;
    controller_server_param_weight_inflation.name = "FollowPath.weight_inflation";
    controller_server_param_weight_inflation.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    controller_server_param_max_vel_theta.value.double_value = max_vel_theta_passing;
    controller_server_param_max_vel_x.value.double_value = max_vel_x_passing;
    controller_server_param_max_vel_y.value.double_value = max_vel_y_passing;
    controller_server_param_max_vel_x_backwards.value.double_value = max_vel_x_backwards_passing;
    controller_server_param_acc_lim_x.value.double_value = acc_lim_x_passing;
    controller_server_param_acc_lim_y.value.double_value = acc_lim_y_passing;
    controller_server_param_acc_lim_theta.value.double_value = acc_lim_theta_passing;
    controller_server_param_max_global_plan_lookahead_dist.value.double_value = max_global_plan_lookahead_dist;
    controller_server_param_weight_inflation.value.double_value = weight_inflation;

    // 所有要修改的参数一起push_back
    controller_server_request->parameters.push_back(controller_server_param_max_vel_theta);
    controller_server_request->parameters.push_back(controller_server_param_max_vel_x);
    controller_server_request->parameters.push_back(controller_server_param_max_vel_y);
    controller_server_request->parameters.push_back(controller_server_param_max_vel_x_backwards);
    controller_server_request->parameters.push_back(controller_server_param_acc_lim_x);
    controller_server_request->parameters.push_back(controller_server_param_acc_lim_y);
    controller_server_request->parameters.push_back(controller_server_param_acc_lim_theta);
    controller_server_request->parameters.push_back(controller_server_param_max_global_plan_lookahead_dist);
    controller_server_request->parameters.push_back(controller_server_param_weight_inflation);

    // 使用回调的异步调用
    auto controller_server_future = controller_server_parameter_client_->async_send_request(
        controller_server_request,
        [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
            this->handle_parameter_response(future);
        });

    /* local_costmap */
    //构建请求
    auto local_costmap_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

    rcl_interfaces::msg::Parameter local_costmap_param_robot_radius;
    local_costmap_param_robot_radius.name = "robot_radius";
    local_costmap_param_robot_radius.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    local_costmap_param_robot_radius.value.double_value = robot_radius;

    local_costmap_request->parameters.push_back(local_costmap_param_robot_radius);

    // 使用回调的异步调用
    auto local_costmap_future = local_costmap_parameter_client_->async_send_request(
        local_costmap_request,
        [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
            this->handle_parameter_response(future);
        });
}
