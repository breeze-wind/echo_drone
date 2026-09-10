//
// 由 elsa 于 25-7-4 创建。
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
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "robot_interfaces/msg/image_location.hpp"
#include "robot_interfaces/msg/mission_event.hpp"
#include "robot_interfaces/msg/mission_status.hpp"
#include "robot_interfaces/srv/jump_mission_state.hpp"
#include "robot_interfaces/srv/shift_mission_state.hpp"
#include "behavior_control/mission_graph.hpp"
#include "behavior_control/mission_runtime.hpp"

/// 任务决策节点，当前调试版本在起飞高度确认后会截断到匀速圆周运动。
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
    /// 接收usb相机传来的随机靶信息
    void USBCameraInfoCallback(const robot_interfaces::msg::ImageLocation::SharedPtr msg);
    /// 首次进入圆周调试状态时锁定圆心、起始时间和方向。
    void start_takeoff_circle_if_needed();
    /// 按固定线速度向 /robot/target_pose 持续发布圆周上的目标点。
    void publish_takeoff_circle_target();
    /// 非阻塞发送 Nav2 目标；action server 未就绪时拒绝本次发送。
    void send_navigation_goal(const nav2_msgs::action::NavigateToPose::Goal & goal);
    /// 当前是否把最终任务目标直接交给 SLS 速度控制器，而非 Nav2 action。
    bool use_sls_goal_navigation() const;
    /// 周期发布具体任务状态、执行器生命周期和健康标志。
    void publish_mission_status();
    /// 发布一次状态转移或调试操作事件。
    void publish_mission_event(int from_step, int to_step, const std::string & reason);
    /// 发布 Graphviz DOT 格式的完整决策有向图。
    void publish_mission_graph();
    /// 在位姿和必要 TF 有效时发布当前位置保持；无法安全保持时返回 false。
    bool publish_safe_hold();
    /// 重新开始任务前清空计数器和临时执行标志。
    void reset_mission_context();
    /// 将命名 start_state 或旧数字字符串解析成兼容 step。
    int resolve_start_state(const std::string & value) const;
    /// 以下三个函数集中维护“旧 step -> 命名状态/阶段/目标”的映射。
    std::string mission_state_name(int step) const;
    std::string mission_phase_name(int step) const;
    std::string mission_target_name(int step) const;
    /// /mission/start：从 idle/aborted 按配置起点重新启动。
    void handle_start(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /// /mission/pause：停止状态推进和周期命令，条件允许时发布一次安全保持。
    void handle_pause(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /// /mission/resume：从暂停点继续运行。
    void handle_resume(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /// /mission/step_once：暂停状态下仅放行一个决策判断 tick。
    void handle_step_once(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /// /mission/abort：中止任务并禁止后续周期命令。
    void handle_abort(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /// /mission/dump_context：以 JSON 字符串导出当前调试上下文。
    void handle_dump_context(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    /// /mission/jump_to_state：按稳定 ID 绝对跳转。
    void handle_jump_to_state(
        const std::shared_ptr<robot_interfaces::srv::JumpMissionState::Request> request,
        std::shared_ptr<robot_interfaces::srv::JumpMissionState::Response> response);
    /// /mission/shift_state：沿调试顺序做 +1/-1 等相对跳转。
    void handle_shift_state(
        const std::shared_ptr<robot_interfaces::srv::ShiftMissionState::Request> request,
        std::shared_ptr<robot_interfaces::srv::ShiftMissionState::Response> response);
    /// 执行调试硬跳转；仅允许 dry-run 且 idle/paused。
    bool apply_debug_jump(
        int target_state_id, bool reset_context, const std::string & reason,
        std::string & message);
    /// 清理仅属于当前状态的计数器和临时动作标志。
    void reset_state_local_context();

    /// 发布目标点位姿
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr target_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    /// sls_goal 模式专用：发布最终目标给 SLS，避免复用 RViz 的 /goal_pose。
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sls_nav_goal_pub_;
    /// 显式仲裁桥接器的速度输入源；true 时只接收 SLS 的速度参考。
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sls_goal_control_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal navigate_to_pose_goal_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr passing_door_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr turning_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr obstacle_height_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr camera_choose_pub_;
    /// 新决策调试接口；/robot/mission_status 是设计稿中旧名称的兼容发布。
    rclcpp::Publisher<robot_interfaces::msg::MissionStatus>::SharedPtr mission_status_pub_;
    rclcpp::Publisher<robot_interfaces::msg::MissionStatus>::SharedPtr legacy_mission_status_pub_;
    rclcpp::Publisher<robot_interfaces::msg::MissionEvent>::SharedPtr mission_event_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_graph_pub_;
    /// 接收当前起飞状态
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_state_sub_;
    /// 接收当前位姿
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr current_pose_sub_;
    /// 接收相机传来的图像位置信息
    rclcpp::Subscription<robot_interfaces::msg::ImageLocation>::SharedPtr image_location_sub_;
    /// 接收usb相机传来的随机靶信息
    rclcpp::Subscription<robot_interfaces::msg::ImageLocation>::SharedPtr usbcamera_info_sub_;

    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> servo_parameter_client_;
    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> controller_server_parameter_client_;
    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> local_costmap_parameter_client_;

    /// 执行步骤计时器
    rclcpp::TimerBase::SharedPtr step_timer_;
    /// 执行任务计时器
    rclcpp::TimerBase::SharedPtr mission_timer_;
    rclcpp::TimerBase::SharedPtr mission_status_timer_;
    /// 以 1 Hz 发布动态图，使普通 volatile CLI 订阅者也能看到图和运行状态。
    rclcpp::TimerBase::SharedPtr mission_graph_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mission_start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mission_pause_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mission_resume_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mission_step_once_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mission_abort_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mission_dump_context_service_;
    rclcpp::Service<robot_interfaces::srv::JumpMissionState>::SharedPtr mission_jump_service_;
    rclcpp::Service<robot_interfaces::srv::ShiftMissionState>::SharedPtr mission_shift_service_;
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

    bool camera_choose_; //True -- d435

    /// 起飞点附近随机靶搜索坐标
    std::vector<double> random_target_init_search_1_;
    std::vector<double> random_target_init_search_2_;

    /// 随机靶搜索坐标
    std::vector<double> random_target_search_1_;
    std::vector<double> random_target_search_2_;
    std::vector<double> random_target_search_3_;

    /// 预设随机靶坐标为其中一个定靶点，找不到随机靶时投这个
    std::vector<double> prev_random_target_;
    /// 最终确定的随机靶坐标
    std::vector<double> random_target_;
    /// 是否找到随机靶
    bool if_find_random_target_;

    // ******************* tank ********************
    /// 预设随机靶坐标为其中一个定靶点，找不到随机靶时投这个
    std::vector<double> prev_random_tank_target_;
    /// 最终确定的随机靶坐标
    std::vector<double> random_tank_target_;
    /// 是否找到随机靶
    bool if_find_random_tank_target_;

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

    bool is_tank_or_bridge_;

    /* 当前位置 */
    double current_x_;
    double current_y_;
    double current_z_;

    /// 当前步骤
    int current_step;
    /// start_state 解析后的旧 step，重新 start 时回到这里。
    int configured_start_step_;
    /// dry-run 下允许观察和单步，但禁止真实任务命令输出。
    bool dry_run_;
    /// 是否保持旧节点“启动即运行”的兼容行为。
    bool autostart_;
    /// launch/YAML 提供的命名起始状态或旧数字字符串。
    std::string start_state_;
    /// nav2（默认）或 sls_goal（最终目标由 SLS 转成速度 setpoint）。
    std::string navigation_execution_mode_;
    /// SLS 最终目标输入话题；仅 sls_goal 模式使用。
    std::string sls_nav_goal_topic_;
    /// SLS 最终目标控制是否在当前任务状态生效。
    bool sls_goal_control_active_{false};
    /// 独立于具体任务 step 的执行门控运行时。
    behavior_control::MissionRuntime mission_runtime_;
    /// 稳定状态 ID、正常有向边和调试顺序的统一注册表。
    behavior_control::MissionGraph mission_graph_;
    /// 当前具体任务状态的进入时间，用于计算 elapsed_sec。
    rclcpp::Time state_enter_time_;
    /// /mission/event 的单调递增序号。
    uint64_t mission_event_sequence_{0};

    float eject_last_x;
    float eject_last_y;
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

    //投掷偏置
    double offset_x_1_;
    double offset_y_1_;
    double offset_x_2_;
    double offset_y_2_;
    double offset_x_3_;
    double offset_y_3_;

    /// 起飞后圆周调试开关；关闭后才会进入旧的随机靶/投掷流程。
    bool takeoff_circle_enabled_;
    /// 防止每次定时器回调都重置圆心和起始时间。
    bool takeoff_circle_started_;
    /// 圆周半径，单位 m。
    double takeoff_circle_radius_;
    /// 圆周线速度，单位 m/s。
    double takeoff_circle_speed_;
    /// 运动方向，配置值会归一为 +1 或 -1。
    double takeoff_circle_direction_;
    /// 进入圆周状态时根据当前位置锁定的圆心。
    double takeoff_circle_center_x_;
    double takeoff_circle_center_y_;
    /// 圆周角度积分的起始时间。
    rclcpp::Time takeoff_circle_start_time_;

    //导航模式，0--正常导航，1--穿门时导航
    int current_nav_mode;
    int last_nav_mode;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    /// 必要相机外参是否可用；缺失时禁止真实任务命令。
    bool tf_ready_{false};
    /// 是否至少收到过一次有效当前位姿；防止暂停/中止时发布零点保持。
    bool pose_received_{false};

    geometry_msgs::msg::TransformStamped map_to_livox;
    geometry_msgs::msg::TransformStamped livox_to_camera;
    geometry_msgs::msg::TransformStamped map_to_camera;

    Eigen::Affine3d map_to_livox_affine;
    Eigen::Affine3d livox_to_camera_affine;
    Eigen::Affine3d map_to_camera_affine;

    nav2_msgs::action::NavigateToPose::Goal navigate_to_pose_action_;

    geometry_msgs::msg::TransformStamped current_target_position_;

    geometry_msgs::msg::PointStamped camera_pt_, world_pt_;

    double stored_point[2];
};

#endif //BEHAVIOR_CONTROL_HPP
