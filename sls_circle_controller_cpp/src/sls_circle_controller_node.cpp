#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sls_qsf_core/qsf_c_api.h"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

// 当前实机/PX4 SITL 优先使用的 SLS/QSF 圆周控制器。
// 输入 MAVROS local_position、可选负载位姿和 MAVROS state，输出 reference、调试姿态目标和真实 attitude setpoint。
namespace
{

// 轻量三维向量，避免为单文件控制器额外引入 Eigen 依赖。
struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Reference
{
  // 参考轨迹同时保存到 snap，供 QSF 生成算法使用。
  Vec3 pos;
  Vec3 vel;
  Vec3 acc;
  Vec3 jerk;
  Vec3 snap;
};

struct LoadState
{
  // active=false 时使用“机体正下方一根竖直绳”的虚拟负载，便于没有吊载传感器时先调机体轨迹。
  Vec3 pos;
  Vec3 vel;
  bool active{false};
};

double clamp_value(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double norm3(const Vec3 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

bool finite3(const Vec3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

constexpr double kMaxAcceptedStateSpeed = 20.0;

Vec3 operator-(const Vec3 & a, const Vec3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3 & a, double scale)
{
  return {a.x * scale, a.y * scale, a.z * scale};
}

Vec3 normalize3(const Vec3 & v, const Vec3 & fallback)
{
  const double n = norm3(v);
  if (n < 1.0e-9) {
    return fallback;
  }
  return {v.x / n, v.y / n, v.z / n};
}

Vec3 unit_vector(const Vec3 & v)
{
  return normalize3(v, {0.0, 0.0, 0.0});
}

Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x,
  };
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.compare(0, prefix.size(), prefix) == 0;
}

std::string json_bool(bool value)
{
  return value ? "true" : "false";
}

std::string json_escape(const std::string & value)
{
  std::ostringstream out;
  for (const char c : value) {
    switch (c) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

std::string json_vec(const Vec3 & v)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "[" << v.x << "," << v.y << "," << v.z << "]";
  return out.str();
}

template<size_t N>
std::string json_array(const std::array<double, N> & v)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(6) << "[";
  for (size_t i = 0; i < N; ++i) {
    if (i > 0) {
      out << ",";
    }
    out << v[i];
  }
  out << "]";
  return out.str();
}

std::array<double, 4> quat_from_rotation_matrix(const double r[3][3])
{
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  const double trace = r[0][0] + r[1][1] + r[2][2];
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    qw = 0.25 * s;
    qx = (r[2][1] - r[1][2]) / s;
    qy = (r[0][2] - r[2][0]) / s;
    qz = (r[1][0] - r[0][1]) / s;
  } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
    const double s = std::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0;
    qw = (r[2][1] - r[1][2]) / s;
    qx = 0.25 * s;
    qy = (r[0][1] + r[1][0]) / s;
    qz = (r[0][2] + r[2][0]) / s;
  } else if (r[1][1] > r[2][2]) {
    const double s = std::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0;
    qw = (r[0][2] - r[2][0]) / s;
    qx = (r[0][1] + r[1][0]) / s;
    qy = 0.25 * s;
    qz = (r[1][2] + r[2][1]) / s;
  } else {
    const double s = std::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0;
    qw = (r[1][0] - r[0][1]) / s;
    qx = (r[0][2] + r[2][0]) / s;
    qy = (r[1][2] + r[2][1]) / s;
    qz = 0.25 * s;
  }

  const double n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (n < 1.0e-9) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  return {qx / n, qy / n, qz / n, qw / n};
}

std::array<double, 4> attitude_from_net_accel(
  const Vec3 & net_accel_enu, double yaw, double gravity)
{
  // MAVROS AttitudeTarget 需要姿态和归一化推力，这里先把净加速度转换成总推力方向。
  const Vec3 thrust_accel_enu{
    net_accel_enu.x,
    net_accel_enu.y,
    net_accel_enu.z + gravity,
  };
  const Vec3 z_b = normalize3(thrust_accel_enu, {0.0, 0.0, 1.0});
  const Vec3 x_c{std::cos(yaw), std::sin(yaw), 0.0};
  const Vec3 y_b = normalize3(cross(z_b, x_c), {0.0, 1.0, 0.0});
  const Vec3 x_b = normalize3(cross(y_b, z_b), {1.0, 0.0, 0.0});
  const double rot[3][3] = {
    {x_b.x, y_b.x, z_b.x},
    {x_b.y, y_b.y, z_b.y},
    {x_b.z, y_b.z, z_b.z},
  };
  return quat_from_rotation_matrix(rot);
}

class LesoAxis
{
public:
  // 单轴三阶 LESO，z3 表示估计到的等效扰动加速度。
  explicit LesoAxis(double bandwidth)
  {
    set_bandwidth(bandwidth);
  }

  void set_bandwidth(double bandwidth)
  {
    bandwidth_ = std::max(0.1, bandwidth);
    beta1_ = 3.0 * bandwidth_;
    beta2_ = 3.0 * bandwidth_ * bandwidth_;
    beta3_ = bandwidth_ * bandwidth_ * bandwidth_;
  }

  void reset(double position, double velocity)
  {
    z1_ = position;
    z2_ = velocity;
    z3_ = 0.0;
    initialized_ = true;
  }

  double update(double position, double nominal_accel, double dt)
  {
    if (!initialized_) {
      reset(position, 0.0);
    }
    if (dt <= 0.0) {
      return z3_;
    }
    const double error = position - z1_;
    z1_ += (z2_ + beta1_ * error) * dt;
    z2_ += (z3_ + beta2_ * error + nominal_accel) * dt;
    z3_ += beta3_ * error * dt;
    return z3_;
  }

private:
  bool initialized_{false};
  double bandwidth_{4.0};
  double beta1_{12.0};
  double beta2_{48.0};
  double beta3_{64.0};
  double z1_{0.0};
  double z2_{0.0};
  double z3_{0.0};
};

class RateMeter
{
public:
  // 以 0.5 秒窗口估算实时频率，用于 /sls_circle/status 和风调试日志。
  void tick(double now)
  {
    if (!window_started_) {
      window_start_ = now;
      window_started_ = true;
      sample_count_ = 0;
    }
    ++sample_count_;
    const double elapsed = now - window_start_;
    if (elapsed >= 0.5) {
      hz_ = static_cast<double>(sample_count_) / elapsed;
      window_start_ = now;
      sample_count_ = 0;
    }
  }

  double hz() const
  {
    return hz_;
  }

private:
  bool window_started_{false};
  double window_start_{0.0};
  size_t sample_count_{0};
  double hz_{0.0};
};

class SlsCircleControllerCpp : public rclcpp::Node
{
public:
  SlsCircleControllerCpp(int argc, char ** argv)
  : Node("sls_circle_controller"),
    leso_{LesoAxis(4.0), LesoAxis(4.0), LesoAxis(4.0)}
  {
    declare_parameters();
    read_parameters();
    apply_cli_parameter_overrides(argc, argv);
    normalize_parameters();
    qsf_core_version_ = sls_qsf_core_version();
    for (auto & axis : leso_) {
      axis.set_bandwidth(wind_observer_bandwidth_);
    }

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlsCircleControllerCpp::pose_callback, this, std::placeholders::_1));
    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      velocity_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlsCircleControllerCpp::velocity_callback, this, std::placeholders::_1));
    load_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      load_pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlsCircleControllerCpp::load_pose_callback, this, std::placeholders::_1));
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      state_topic_, 20,
      std::bind(&SlsCircleControllerCpp::state_callback, this, std::placeholders::_1));
    actual_wind_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      actual_wind_topic_, 10,
      std::bind(&SlsCircleControllerCpp::actual_wind_callback, this, std::placeholders::_1));
    nav_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      nav_goal_topic_, 10,
      std::bind(&SlsCircleControllerCpp::nav_goal_callback, this, std::placeholders::_1));

    reference_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      reference_pose_topic_, 10);
    debug_attitude_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(
      debug_attitude_topic_, 10);
    real_attitude_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(
      real_attitude_topic_, 10);
    takeoff_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      takeoff_pose_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    wind_estimate_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      wind_estimate_topic_, 10);
    nav_velocity_setpoint_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      nav_velocity_setpoint_topic_, 10);
    sls_goal_control_pub_ = create_publisher<std_msgs::msg::Bool>(
      sls_goal_control_topic_, 10);

    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(set_mode_service_);
    arming_client_ = create_client<mavros_msgs::srv::CommandBool>(arming_service_);

    const double rate = std::max(1.0, control_rate_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate));
    timer_ = create_wall_timer(period, std::bind(&SlsCircleControllerCpp::control_loop, this));

    RCLCPP_INFO(
      get_logger(),
      "sls_circle_controller_cpp started; mode=%s mission=%s dry_run=%s real_setpoint=%s "
      "target_control_hz=%.1f qsf_core=%s",
      controller_mode_.c_str(), mission_mode_.c_str(),
      dry_run_ ? "true" : "false",
      enable_real_setpoint_ ? "true" : "false",
      control_rate_,
      qsf_core_version_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "QSF coordinate mapping: state/ref use [north=ENU y, east=ENU x, down=-ENU z], "
      "force back to ENU thrust_accel=[F_east,F_north,-F_down]/m");
  }

private:
  void declare_parameters()
  {
    // 参数声明分为安全开关、话题名、任务阶段、控制增益、QSF 参数和抗风估计参数。
    declare_parameter<bool>("dry_run", true);
    declare_parameter<bool>("enable_real_setpoint", false);
    declare_parameter<std::string>("pose_topic", "/mavros/local_position/pose");
    declare_parameter<std::string>("velocity_topic", "/mavros/local_position/velocity_local");
    declare_parameter<bool>("use_velocity_topic", true);
    declare_parameter<double>("velocity_stale_timeout", 0.5);
    declare_parameter<std::string>("load_pose_topic", "/sls_circle/load_pose");
    declare_parameter<bool>("use_load_pose", false);
    declare_parameter<double>("load_pose_stale_timeout", 0.5);
    declare_parameter<std::string>("state_topic", "/mavros/state");
    declare_parameter<std::string>("real_attitude_topic", "/mavros/setpoint_raw/attitude");
    declare_parameter<std::string>("debug_attitude_topic", "/sls_circle/debug/attitude_target");
    declare_parameter<std::string>("takeoff_pose_topic", "/mavros/setpoint_position/local");
    declare_parameter<std::string>("reference_pose_topic", "/sls_circle/reference_pose");
    declare_parameter<std::string>("status_topic", "/sls_circle/status");
    declare_parameter<std::string>("wind_estimate_topic", "/sls_circle/wind_estimate");
    declare_parameter<std::string>("actual_wind_topic", "/sls_circle/wind_actual");
    // nav_goal_velocity 模式：behavior_control 发布最终任务目标，SLS 用自身的
    // 位置/速度闭环将其转换成唯一的 MAVROS 速度 setpoint。
    declare_parameter<std::string>("nav_goal_topic", "/sls_circle/nav_goal");
    declare_parameter<std::string>(
      "nav_velocity_setpoint_topic", "/sls_circle/nav_velocity_setpoint");
    // RViz 直控模式中，SLS 自己选择飞控桥的速度来源；默认关闭以保留 behavior_control
    // 作为唯一控制源的原有链路。
    declare_parameter<bool>("publish_sls_goal_control_active", false);
    declare_parameter<std::string>(
      "sls_goal_control_topic", "/robot/sls_goal_control_active");
    declare_parameter<double>("nav_goal_stale_timeout", 1.0);
    declare_parameter<double>("nav_goal_velocity_lookahead", 0.4);
    declare_parameter<double>("nav_goal_max_speed_xy", 1.0);
    declare_parameter<double>("nav_goal_max_speed_z", 0.5);
    declare_parameter<double>("nav_goal_tolerance", 0.10);
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("control_rate", 100.0);
    declare_parameter<double>("pose_stale_timeout", 0.5);
    declare_parameter<std::string>("mission_mode", "circle_only");
    declare_parameter<double>("preflight_setpoint_time", 2.0);
    declare_parameter<double>("takeoff_altitude", 1.0);
    declare_parameter<double>("takeoff_x", 0.0);
    declare_parameter<double>("takeoff_y", 0.0);
    declare_parameter<bool>("use_current_xy_for_takeoff", true);
    declare_parameter<double>("takeoff_z_tolerance", 0.15);
    declare_parameter<double>("takeoff_settle_time", 2.0);
    declare_parameter<double>("post_takeoff_hold_time", 5.0);
    declare_parameter<bool>("enable_takeoff_position_setpoint", true);
    declare_parameter<bool>("enable_mavros_services", false);
    declare_parameter<bool>("auto_offboard", false);
    declare_parameter<bool>("auto_arm", false);
    declare_parameter<std::string>("set_mode_service", "/mavros/set_mode");
    declare_parameter<std::string>("arming_service", "/mavros/cmd/arming");
    declare_parameter<std::string>("offboard_mode", "OFFBOARD");
    declare_parameter<double>("service_retry_period", 1.0);
    declare_parameter<bool>("use_current_pose_as_start", true);
    declare_parameter<double>("center_x", 0.0);
    declare_parameter<double>("center_y", 0.0);
    declare_parameter<double>("center_z", 1.0);
    declare_parameter<bool>("center_z_from_pose", true);
    // 圆轨迹参数：半径(m)、角速度(rad/s)、绕圈数(0=持续)、初始相位(rad)。
    declare_parameter<double>("radius", 1.0);
    declare_parameter<double>("angular_velocity", 0.35);
    declare_parameter<double>("circle_loops", 0.0);
    declare_parameter<double>("phase", 0.0);
    declare_parameter<double>("yaw", 0.0);
    declare_parameter<double>("kp_xy", 1.8);
    declare_parameter<double>("kp_z", 2.5);
    declare_parameter<double>("kd_xy", 1.4);
    declare_parameter<double>("kd_z", 1.6);
    declare_parameter<double>("max_acc_xy", 2.5);
    declare_parameter<double>("max_acc_z", 2.0);
    declare_parameter<double>("max_total_acc", 4.0);
    declare_parameter<double>("max_tilt_deg", 25.0);
    declare_parameter<double>("gravity", 9.80665);
    declare_parameter<double>("hover_thrust", 0.5);
    declare_parameter<double>("min_thrust", 0.05);
    declare_parameter<double>("max_thrust", 0.85);
    // 定推力标定会绕开位置环，只允许 PX4 SITL launch 在显式确认后开启。
    declare_parameter<bool>("allow_fixed_thrust_calibration", false);
    declare_parameter<bool>("require_connected", true);
    declare_parameter<bool>("require_offboard", false);
    declare_parameter<bool>("require_armed", false);
    declare_parameter<std::string>("controller_mode", "pd");
    declare_parameter<double>("mav_mass", 1.56);
    declare_parameter<double>("load_mass", 0.25);
    declare_parameter<double>("cable_length", 0.85);
    declare_parameter<double>("qsf_kp_x", 10.0);
    declare_parameter<double>("qsf_kv_x", 5.0);
    // QSF horizontal-only mode: use the ordinary PX4-compatible position
    // loop for vertical acceleration instead of the pendulum model's z force.
    declare_parameter<bool>("qsf_position_z_control", false);
    declare_parameter<double>("qsf_ka_x", 0.0);
    declare_parameter<double>("qsf_kj_x", 0.0);
    declare_parameter<double>("qsf_kp_y", 10.0);
    declare_parameter<double>("qsf_kv_y", 5.0);
    declare_parameter<double>("qsf_ka_y", 0.0);
    declare_parameter<double>("qsf_kj_y", 0.0);
    declare_parameter<double>("qsf_kp_z", 20.0);
    declare_parameter<double>("qsf_kv_z", 10.0);
    declare_parameter<double>("qsf_ki_x", 12.0);
    declare_parameter<double>("qsf_ki_y", 12.0);
    declare_parameter<double>("qsf_ki_z", 1.0);
    declare_parameter<double>("qsf_integral_limit", 10.0);
    declare_parameter<bool>("qsf_reference_is_load", false);
    declare_parameter<bool>("enable_anti_wind", false);
    declare_parameter<std::string>("wind_estimator_mode", "residual");
    declare_parameter<double>("wind_observer_bandwidth", 4.0);
    declare_parameter<double>("wind_estimate_filter_tau", 0.5);
    declare_parameter<double>("wind_estimate_force_limit", 5.0);
    declare_parameter<double>("wind_compensation_gain", 0.7);
    declare_parameter<double>("wind_compensation_warmup_time", 0.0);
    declare_parameter<double>("wind_compensation_ramp_time", 0.0);
    declare_parameter<double>("wind_integral_gain", 0.0);
    declare_parameter<double>("wind_integral_limit", 5.0);
    declare_parameter<bool>("enable_wind_debug_log", true);
    declare_parameter<double>("wind_debug_log_period", 1.0);
    declare_parameter<double>("status_period", 0.25);
  }

  void read_parameters()
  {
    // 读取后立即做基本类型转换和 mission/controller mode 小写化，避免 launch 字符串大小写差异。
    dry_run_ = get_parameter("dry_run").as_bool();
    enable_real_setpoint_ = get_parameter("enable_real_setpoint").as_bool();
    pose_topic_ = get_parameter("pose_topic").as_string();
    velocity_topic_ = get_parameter("velocity_topic").as_string();
    use_velocity_topic_ = get_parameter("use_velocity_topic").as_bool();
    velocity_stale_timeout_ = get_parameter("velocity_stale_timeout").as_double();
    load_pose_topic_ = get_parameter("load_pose_topic").as_string();
    use_load_pose_ = get_parameter("use_load_pose").as_bool();
    load_pose_stale_timeout_ = get_parameter("load_pose_stale_timeout").as_double();
    state_topic_ = get_parameter("state_topic").as_string();
    real_attitude_topic_ = get_parameter("real_attitude_topic").as_string();
    debug_attitude_topic_ = get_parameter("debug_attitude_topic").as_string();
    takeoff_pose_topic_ = get_parameter("takeoff_pose_topic").as_string();
    reference_pose_topic_ = get_parameter("reference_pose_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    wind_estimate_topic_ = get_parameter("wind_estimate_topic").as_string();
    actual_wind_topic_ = get_parameter("actual_wind_topic").as_string();
    nav_goal_topic_ = get_parameter("nav_goal_topic").as_string();
    nav_velocity_setpoint_topic_ = get_parameter("nav_velocity_setpoint_topic").as_string();
    publish_sls_goal_control_active_ =
      get_parameter("publish_sls_goal_control_active").as_bool();
    sls_goal_control_topic_ = get_parameter("sls_goal_control_topic").as_string();
    nav_goal_stale_timeout_ = get_parameter("nav_goal_stale_timeout").as_double();
    nav_goal_velocity_lookahead_ = get_parameter("nav_goal_velocity_lookahead").as_double();
    nav_goal_max_speed_xy_ = get_parameter("nav_goal_max_speed_xy").as_double();
    nav_goal_max_speed_z_ = get_parameter("nav_goal_max_speed_z").as_double();
    nav_goal_tolerance_ = get_parameter("nav_goal_tolerance").as_double();
    frame_id_ = get_parameter("frame_id").as_string();
    control_rate_ = get_parameter("control_rate").as_double();
    pose_stale_timeout_ = get_parameter("pose_stale_timeout").as_double();
    mission_mode_ = get_parameter("mission_mode").as_string();
    std::transform(mission_mode_.begin(), mission_mode_.end(), mission_mode_.begin(), ::tolower);
    preflight_setpoint_time_ = std::max(0.0, get_parameter("preflight_setpoint_time").as_double());
    takeoff_altitude_ = get_parameter("takeoff_altitude").as_double();
    takeoff_x_ = get_parameter("takeoff_x").as_double();
    takeoff_y_ = get_parameter("takeoff_y").as_double();
    use_current_xy_for_takeoff_ = get_parameter("use_current_xy_for_takeoff").as_bool();
    takeoff_z_tolerance_ = get_parameter("takeoff_z_tolerance").as_double();
    takeoff_settle_time_ = std::max(0.0, get_parameter("takeoff_settle_time").as_double());
    post_takeoff_hold_time_ = std::max(0.0, get_parameter("post_takeoff_hold_time").as_double());
    enable_takeoff_position_setpoint_ = get_parameter("enable_takeoff_position_setpoint").as_bool();
    enable_mavros_services_ = get_parameter("enable_mavros_services").as_bool();
    auto_offboard_ = get_parameter("auto_offboard").as_bool();
    auto_arm_ = get_parameter("auto_arm").as_bool();
    set_mode_service_ = get_parameter("set_mode_service").as_string();
    arming_service_ = get_parameter("arming_service").as_string();
    offboard_mode_ = get_parameter("offboard_mode").as_string();
    service_retry_period_ = std::max(0.1, get_parameter("service_retry_period").as_double());
    use_current_pose_as_start_ = get_parameter("use_current_pose_as_start").as_bool();
    center_ = {
      get_parameter("center_x").as_double(),
      get_parameter("center_y").as_double(),
      get_parameter("center_z").as_double(),
    };
    center_z_from_pose_ = get_parameter("center_z_from_pose").as_bool();
    radius_ = std::max(0.05, get_parameter("radius").as_double());
    angular_velocity_ = get_parameter("angular_velocity").as_double();
    circle_loops_ = get_parameter("circle_loops").as_double();
    phase_ = get_parameter("phase").as_double();
    yaw_ = get_parameter("yaw").as_double();
    kp_xy_ = get_parameter("kp_xy").as_double();
    kp_z_ = get_parameter("kp_z").as_double();
    kd_xy_ = get_parameter("kd_xy").as_double();
    kd_z_ = get_parameter("kd_z").as_double();
    max_acc_xy_ = get_parameter("max_acc_xy").as_double();
    max_acc_z_ = get_parameter("max_acc_z").as_double();
    max_total_acc_ = get_parameter("max_total_acc").as_double();
    max_tilt_deg_ = get_parameter("max_tilt_deg").as_double();
    max_tilt_rad_ = max_tilt_deg_ * M_PI / 180.0;
    gravity_ = get_parameter("gravity").as_double();
    hover_thrust_ = std::max(0.05, get_parameter("hover_thrust").as_double());
    min_thrust_ = get_parameter("min_thrust").as_double();
    max_thrust_ = get_parameter("max_thrust").as_double();
    allow_fixed_thrust_calibration_ =
      get_parameter("allow_fixed_thrust_calibration").as_bool();
    require_connected_ = get_parameter("require_connected").as_bool();
    require_offboard_ = get_parameter("require_offboard").as_bool();
    require_armed_ = get_parameter("require_armed").as_bool();
    controller_mode_ = get_parameter("controller_mode").as_string();
    std::transform(controller_mode_.begin(), controller_mode_.end(), controller_mode_.begin(), ::tolower);
    mav_mass_ = std::max(0.05, get_parameter("mav_mass").as_double());
    load_mass_ = std::max(0.0, get_parameter("load_mass").as_double());
    cable_length_ = std::max(0.05, get_parameter("cable_length").as_double());
    qsf_kp_x_ = get_parameter("qsf_kp_x").as_double();
    qsf_kv_x_ = get_parameter("qsf_kv_x").as_double();
    qsf_position_z_control_ = get_parameter("qsf_position_z_control").as_bool();
    qsf_ka_x_ = get_parameter("qsf_ka_x").as_double();
    qsf_kj_x_ = get_parameter("qsf_kj_x").as_double();
    qsf_kp_y_ = get_parameter("qsf_kp_y").as_double();
    qsf_kv_y_ = get_parameter("qsf_kv_y").as_double();
    qsf_ka_y_ = get_parameter("qsf_ka_y").as_double();
    qsf_kj_y_ = get_parameter("qsf_kj_y").as_double();
    qsf_kp_z_ = get_parameter("qsf_kp_z").as_double();
    qsf_kv_z_ = get_parameter("qsf_kv_z").as_double();
    qsf_ki_x_ = get_parameter("qsf_ki_x").as_double();
    qsf_ki_y_ = get_parameter("qsf_ki_y").as_double();
    qsf_ki_z_ = get_parameter("qsf_ki_z").as_double();
    qsf_integral_limit_ = get_parameter("qsf_integral_limit").as_double();
    qsf_reference_is_load_ = get_parameter("qsf_reference_is_load").as_bool();
    enable_anti_wind_ = get_parameter("enable_anti_wind").as_bool();
    wind_estimator_mode_ = get_parameter("wind_estimator_mode").as_string();
    wind_observer_bandwidth_ = get_parameter("wind_observer_bandwidth").as_double();
    wind_estimate_filter_tau_ = get_parameter("wind_estimate_filter_tau").as_double();
    wind_estimate_force_limit_ = get_parameter("wind_estimate_force_limit").as_double();
    wind_compensation_gain_ = get_parameter("wind_compensation_gain").as_double();
    wind_compensation_warmup_time_ = get_parameter("wind_compensation_warmup_time").as_double();
    wind_compensation_ramp_time_ = get_parameter("wind_compensation_ramp_time").as_double();
    wind_integral_gain_ = get_parameter("wind_integral_gain").as_double();
    wind_integral_limit_ = get_parameter("wind_integral_limit").as_double();
    enable_wind_debug_log_ = get_parameter("enable_wind_debug_log").as_bool();
    wind_debug_log_period_ = std::max(0.1, get_parameter("wind_debug_log_period").as_double());
    status_period_ = std::max(0.05, get_parameter("status_period").as_double());
    nav_goal_stale_timeout_ = std::max(0.05, nav_goal_stale_timeout_);
    nav_goal_velocity_lookahead_ = std::max(0.01, nav_goal_velocity_lookahead_);
    nav_goal_max_speed_xy_ = std::max(0.0, nav_goal_max_speed_xy_);
    nav_goal_max_speed_z_ = std::max(0.0, nav_goal_max_speed_z_);
    nav_goal_tolerance_ = std::max(0.0, nav_goal_tolerance_);
    min_thrust_ = clamp_value(min_thrust_, 0.0, 1.0);
    max_thrust_ = clamp_value(max_thrust_, min_thrust_, 1.0);
  }

  static std::string lower_copy(std::string value)
  {
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    return value;
  }

  static bool parse_bool_value(const std::string & value)
  {
    const std::string lowered = lower_copy(value);
    return lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on";
  }

  void set_bool_override(const std::string & name, bool value)
  {
    set_parameter(rclcpp::Parameter(name, value));
  }

  void set_double_override(const std::string & name, double value)
  {
    set_parameter(rclcpp::Parameter(name, value));
  }

  void set_string_override(const std::string & name, const std::string & value)
  {
    set_parameter(rclcpp::Parameter(name, value));
  }

  void apply_cli_parameter_token(const std::string & token)
  {
    const auto split = token.find(":=");
    if (split == std::string::npos) {
      return;
    }

    std::string name = token.substr(0, split);
    const std::string value = token.substr(split + 2);
    const auto dot = name.find_last_of('.');
    if (dot != std::string::npos) {
      name = name.substr(dot + 1);
    }

    try {
      if (name == "dry_run") {
        dry_run_ = parse_bool_value(value);
        set_bool_override(name, dry_run_);
      } else if (name == "enable_real_setpoint") {
        enable_real_setpoint_ = parse_bool_value(value);
        set_bool_override(name, enable_real_setpoint_);
      } else if (name == "mission_mode") {
        mission_mode_ = value;
        set_string_override(name, mission_mode_);
      } else if (name == "nav_goal_topic") {
        nav_goal_topic_ = value;
        set_string_override(name, nav_goal_topic_);
      } else if (name == "nav_velocity_setpoint_topic") {
        nav_velocity_setpoint_topic_ = value;
        set_string_override(name, nav_velocity_setpoint_topic_);
      } else if (name == "publish_sls_goal_control_active") {
        publish_sls_goal_control_active_ = parse_bool_value(value);
        set_bool_override(name, publish_sls_goal_control_active_);
      } else if (name == "sls_goal_control_topic") {
        sls_goal_control_topic_ = value;
        set_string_override(name, sls_goal_control_topic_);
      } else if (name == "controller_mode") {
        controller_mode_ = value;
        set_string_override(name, controller_mode_);
      } else if (name == "pose_topic") {
        pose_topic_ = value;
        set_string_override(name, pose_topic_);
      } else if (name == "velocity_topic") {
        velocity_topic_ = value;
        set_string_override(name, velocity_topic_);
      } else if (name == "takeoff_pose_topic") {
        takeoff_pose_topic_ = value;
        set_string_override(name, takeoff_pose_topic_);
      } else if (name == "real_attitude_topic") {
        real_attitude_topic_ = value;
        set_string_override(name, real_attitude_topic_);
      } else if (name == "arming_service") {
        arming_service_ = value;
        set_string_override(name, arming_service_);
      } else if (name == "enable_anti_wind") {
        enable_anti_wind_ = parse_bool_value(value);
        set_bool_override(name, enable_anti_wind_);
      } else if (name == "use_velocity_topic") {
        use_velocity_topic_ = parse_bool_value(value);
        set_bool_override(name, use_velocity_topic_);
      } else if (name == "wind_estimator_mode") {
        wind_estimator_mode_ = value;
        set_string_override(name, wind_estimator_mode_);
      } else if (name == "require_connected") {
        require_connected_ = parse_bool_value(value);
        set_bool_override(name, require_connected_);
      } else if (name == "require_offboard") {
        require_offboard_ = parse_bool_value(value);
        set_bool_override(name, require_offboard_);
      } else if (name == "require_armed") {
        require_armed_ = parse_bool_value(value);
        set_bool_override(name, require_armed_);
      } else if (name == "enable_mavros_services") {
        enable_mavros_services_ = parse_bool_value(value);
        set_bool_override(name, enable_mavros_services_);
      } else if (name == "auto_offboard") {
        auto_offboard_ = parse_bool_value(value);
        set_bool_override(name, auto_offboard_);
      } else if (name == "auto_arm") {
        auto_arm_ = parse_bool_value(value);
        set_bool_override(name, auto_arm_);
      } else if (name == "use_load_pose") {
        use_load_pose_ = parse_bool_value(value);
        set_bool_override(name, use_load_pose_);
      } else if (name == "qsf_reference_is_load") {
        qsf_reference_is_load_ = parse_bool_value(value);
        set_bool_override(name, qsf_reference_is_load_);
      } else if (name == "preflight_setpoint_time") {
        preflight_setpoint_time_ = std::stod(value);
        set_double_override(name, preflight_setpoint_time_);
      } else if (name == "control_rate") {
        control_rate_ = std::stod(value);
        set_double_override(name, control_rate_);
      } else if (name == "velocity_stale_timeout") {
        velocity_stale_timeout_ = std::stod(value);
        set_double_override(name, velocity_stale_timeout_);
      } else if (name == "takeoff_altitude") {
        takeoff_altitude_ = std::stod(value);
        set_double_override(name, takeoff_altitude_);
      } else if (name == "takeoff_z_tolerance") {
        takeoff_z_tolerance_ = std::stod(value);
        set_double_override(name, takeoff_z_tolerance_);
      } else if (name == "takeoff_x") {
        takeoff_x_ = std::stod(value);
        set_double_override(name, takeoff_x_);
      } else if (name == "takeoff_y") {
        takeoff_y_ = std::stod(value);
        set_double_override(name, takeoff_y_);
      } else if (name == "takeoff_settle_time") {
        takeoff_settle_time_ = std::stod(value);
        set_double_override(name, takeoff_settle_time_);
      } else if (name == "post_takeoff_hold_time") {
        post_takeoff_hold_time_ = std::stod(value);
        set_double_override(name, post_takeoff_hold_time_);
      } else if (name == "nav_goal_stale_timeout") {
        nav_goal_stale_timeout_ = std::stod(value);
        set_double_override(name, nav_goal_stale_timeout_);
      } else if (name == "nav_goal_velocity_lookahead") {
        nav_goal_velocity_lookahead_ = std::stod(value);
        set_double_override(name, nav_goal_velocity_lookahead_);
      } else if (name == "nav_goal_max_speed_xy") {
        nav_goal_max_speed_xy_ = std::stod(value);
        set_double_override(name, nav_goal_max_speed_xy_);
      } else if (name == "nav_goal_max_speed_z") {
        nav_goal_max_speed_z_ = std::stod(value);
        set_double_override(name, nav_goal_max_speed_z_);
      } else if (name == "nav_goal_tolerance") {
        nav_goal_tolerance_ = std::stod(value);
        set_double_override(name, nav_goal_tolerance_);
      } else if (name == "hover_thrust") {
        hover_thrust_ = std::stod(value);
        set_double_override(name, hover_thrust_);
      } else if (name == "radius") {
        radius_ = std::stod(value);
        set_double_override(name, radius_);
      } else if (name == "angular_velocity") {
        angular_velocity_ = std::stod(value);
        set_double_override(name, angular_velocity_);
      } else if (name == "circle_loops") {
        circle_loops_ = std::stod(value);
        set_double_override(name, circle_loops_);
      } else if (name == "mav_mass") {
        mav_mass_ = std::stod(value);
        set_double_override(name, mav_mass_);
      } else if (name == "load_mass") {
        load_mass_ = std::stod(value);
        set_double_override(name, load_mass_);
      } else if (name == "cable_length") {
        cable_length_ = std::stod(value);
        set_double_override(name, cable_length_);
      } else if (name == "max_acc_xy") {
        max_acc_xy_ = std::stod(value);
        set_double_override(name, max_acc_xy_);
      } else if (name == "max_acc_z") {
        max_acc_z_ = std::stod(value);
        set_double_override(name, max_acc_z_);
      } else if (name == "max_total_acc") {
        max_total_acc_ = std::stod(value);
        set_double_override(name, max_total_acc_);
      } else if (name == "max_tilt_deg") {
        max_tilt_deg_ = std::stod(value);
        set_double_override(name, max_tilt_deg_);
      } else if (name == "wind_compensation_gain") {
        wind_compensation_gain_ = std::stod(value);
        set_double_override(name, wind_compensation_gain_);
      } else if (name == "wind_compensation_warmup_time") {
        wind_compensation_warmup_time_ = std::stod(value);
        set_double_override(name, wind_compensation_warmup_time_);
      } else if (name == "wind_compensation_ramp_time") {
        wind_compensation_ramp_time_ = std::stod(value);
        set_double_override(name, wind_compensation_ramp_time_);
      } else if (name == "wind_integral_gain") {
        wind_integral_gain_ = std::stod(value);
        set_double_override(name, wind_integral_gain_);
      } else if (name == "wind_observer_bandwidth") {
        wind_observer_bandwidth_ = std::stod(value);
        set_double_override(name, wind_observer_bandwidth_);
      } else if (name == "wind_estimate_filter_tau") {
        wind_estimate_filter_tau_ = std::stod(value);
        set_double_override(name, wind_estimate_filter_tau_);
      } else if (name == "wind_estimate_force_limit") {
        wind_estimate_force_limit_ = std::stod(value);
        set_double_override(name, wind_estimate_force_limit_);
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        get_logger(), "ignored CLI parameter override %s:=%s: %s",
        name.c_str(), value.c_str(), exc.what());
    }
  }

  void apply_cli_parameter_overrides(int argc, char ** argv)
  {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if ((arg == "-p" || arg == "--param") && i + 1 < argc) {
        apply_cli_parameter_token(argv[++i]);
      }
    }
  }

  void normalize_parameters()
  {
    mission_mode_ = lower_copy(mission_mode_);
    controller_mode_ = lower_copy(controller_mode_);
    preflight_setpoint_time_ = std::max(0.0, preflight_setpoint_time_);
    takeoff_z_tolerance_ = std::max(0.0, takeoff_z_tolerance_);
    takeoff_settle_time_ = std::max(0.0, takeoff_settle_time_);
    post_takeoff_hold_time_ = std::max(0.0, post_takeoff_hold_time_);
    service_retry_period_ = std::max(0.1, service_retry_period_);
    control_rate_ = std::max(1.0, control_rate_);
    velocity_stale_timeout_ = std::max(0.02, velocity_stale_timeout_);
    radius_ = std::max(0.05, radius_);
    max_tilt_rad_ = max_tilt_deg_ * M_PI / 180.0;
    hover_thrust_ = std::max(0.05, hover_thrust_);
    mav_mass_ = std::max(0.05, mav_mass_);
    load_mass_ = std::max(0.0, load_mass_);
    cable_length_ = std::max(0.05, cable_length_);
    wind_estimator_mode_ = lower_copy(wind_estimator_mode_);
    if (wind_estimator_mode_ != "leso" && wind_estimator_mode_ != "actual_feedback") {
      wind_estimator_mode_ = "residual";
    }
    wind_debug_log_period_ = std::max(0.1, wind_debug_log_period_);
    wind_estimate_filter_tau_ = std::max(0.02, wind_estimate_filter_tau_);
    wind_estimate_force_limit_ = std::max(0.0, wind_estimate_force_limit_);
    wind_compensation_warmup_time_ = std::max(0.0, wind_compensation_warmup_time_);
    wind_compensation_ramp_time_ = std::max(0.0, wind_compensation_ramp_time_);
    status_period_ = std::max(0.05, status_period_);
    nav_goal_stale_timeout_ = std::max(0.05, nav_goal_stale_timeout_);
    nav_goal_velocity_lookahead_ = std::max(0.01, nav_goal_velocity_lookahead_);
    nav_goal_max_speed_xy_ = std::max(0.0, nav_goal_max_speed_xy_);
    nav_goal_max_speed_z_ = std::max(0.0, nav_goal_max_speed_z_);
    nav_goal_tolerance_ = std::max(0.0, nav_goal_tolerance_);
    min_thrust_ = clamp_value(min_thrust_, 0.0, 1.0);
    max_thrust_ = clamp_value(max_thrust_, min_thrust_, 1.0);
  }

  double now_seconds()
  {
    return get_clock()->now().nanoseconds() * 1.0e-9;
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const double now = now_seconds();
    pose_input_rate_.tick(now);
    const Vec3 new_pose{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    if (!finite3(new_pose)) {
      return;
    }
    if (pose_valid_) {
      const double pose_dt = now - pose_time_;
      if (pose_dt > 1.0e-4 && norm3(new_pose - pose_) / pose_dt > kMaxAcceptedStateSpeed) {
        return;
      }
    }
    const bool velocity_topic_fresh = use_velocity_topic_ && velocity_topic_valid_ &&
      now - velocity_time_ <= velocity_stale_timeout_;
    if (pose_valid_ && !velocity_topic_fresh) {
      const double dt = std::max(1.0e-6, now - pose_time_);
      const Vec3 new_velocity{
        (new_pose.x - pose_.x) / dt,
        (new_pose.y - pose_.y) / dt,
        (new_pose.z - pose_.z) / dt,
      };
      if (velocity_valid_) {
        observed_accel_ = {
          (new_velocity.x - velocity_.x) / dt,
          (new_velocity.y - velocity_.y) / dt,
          (new_velocity.z - velocity_.z) / dt,
        };
        observed_accel_valid_ = true;
      }
      velocity_ = new_velocity;
      velocity_valid_ = true;
    }
    pose_ = new_pose;
    pose_time_ = now;
    pose_valid_ = true;

    if (!home_position_valid_) {
      home_position_ = pose_;
      home_position_valid_ = true;
    }
    if (!start_time_valid_) {
      start_time_ = now;
      circle_start_time_ = now;
      start_time_valid_ = true;
      circle_start_time_valid_ = true;
      leso_[0].reset(pose_.x, velocity_.x);
      leso_[1].reset(pose_.y, velocity_.y);
      leso_[2].reset(pose_.z, velocity_.z);
    }
  }

  void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (!use_velocity_topic_) {
      return;
    }
    const double now = now_seconds();
    velocity_input_rate_.tick(now);
    const Vec3 new_velocity{
      msg->twist.linear.x,
      msg->twist.linear.y,
      msg->twist.linear.z,
    };
    // Gazebo p3d 和真实估计器都可能在初始化/复位瞬间给出 NaN；绝不能污染控制状态。
    if (!finite3(new_velocity) || norm3(new_velocity) > kMaxAcceptedStateSpeed) {
      observed_accel_valid_ = false;
      return;
    }
    if (velocity_topic_valid_) {
      const double dt = std::max(1.0e-6, now - velocity_time_);
      observed_accel_ = {
        (new_velocity.x - velocity_.x) / dt,
        (new_velocity.y - velocity_.y) / dt,
        (new_velocity.z - velocity_.z) / dt,
      };
      observed_accel_valid_ = true;
    }
    velocity_ = new_velocity;
    velocity_time_ = now;
    velocity_valid_ = true;
    velocity_topic_valid_ = true;
  }

  void load_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const double now = now_seconds();
    load_pose_input_rate_.tick(now);
    const Vec3 new_pose{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    if (!finite3(new_pose)) {
      load_observed_accel_valid_ = false;
      return;
    }
    if (load_pose_valid_) {
      const double pose_dt = now - load_pose_time_;
      if (pose_dt > 1.0e-4 && norm3(new_pose - load_pose_) / pose_dt > kMaxAcceptedStateSpeed) {
        load_observed_accel_valid_ = false;
        return;
      }
    }
    if (load_pose_valid_) {
      const double dt = std::max(1.0e-6, now - load_pose_time_);
      const Vec3 new_velocity{
        (new_pose.x - load_pose_.x) / dt,
        (new_pose.y - load_pose_.y) / dt,
        (new_pose.z - load_pose_.z) / dt,
      };
      if (load_velocity_valid_) {
        load_observed_accel_ = {
          (new_velocity.x - load_velocity_.x) / dt,
          (new_velocity.y - load_velocity_.y) / dt,
          (new_velocity.z - load_velocity_.z) / dt,
        };
        load_observed_accel_valid_ = true;
      }
      load_velocity_ = new_velocity;
      load_velocity_valid_ = true;
    }
    load_pose_ = new_pose;
    load_pose_time_ = now;
    load_pose_valid_ = true;
  }

  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    const bool had_state = mavros_state_received_;
    const auto previous_state = mavros_state_;
    mavros_state_ = *msg;
    mavros_state_received_ = true;

    // 自动 OFFBOARD/解锁只负责首次接管。进入过 OFFBOARD 后，任何退出都按
    // 操作者/飞控接管处理；进入过 armed 后，任何解除解锁也按人工接管处理。
    // 锁存后本进程不再重发请求，避免与遥控器模式开关或急停竞争。
    if (msg->mode == offboard_mode_) {
      offboard_seen_ = true;
    } else if (
      had_state && offboard_seen_ && previous_state.mode == offboard_mode_ &&
      !manual_mode_override_latched_)
    {
      manual_mode_override_latched_ = true;
      RCLCPP_WARN(
        get_logger(),
        "FCU left %s for %s; operator/failsafe override latched, automatic OFFBOARD disabled "
        "until controller restart",
        offboard_mode_.c_str(), msg->mode.c_str());
    }

    if (msg->armed) {
      armed_seen_ = true;
    } else if (had_state && armed_seen_ && previous_state.armed && !manual_disarm_latched_) {
      manual_disarm_latched_ = true;
      RCLCPP_WARN(
        get_logger(),
        "FCU disarmed after initial arm; operator/failsafe override latched, automatic arm "
        "disabled until controller restart");
    }
  }

  void actual_wind_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    const double now = now_seconds();
    actual_wind_input_rate_.tick(now);
    actual_wind_force_ = {msg->vector.x, msg->vector.y, msg->vector.z};
    actual_wind_magnitude_ = norm3(actual_wind_force_);
    actual_wind_direction_ = unit_vector(actual_wind_force_);
    actual_wind_time_ = now;
    actual_wind_time_valid_ = true;
  }

  void nav_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const Vec3 goal{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    if (!finite3(goal)) {
      RCLCPP_WARN(get_logger(), "ignored non-finite SLS navigation goal");
      return;
    }
    nav_goal_ = goal;
    nav_goal_time_ = now_seconds();
    nav_goal_valid_ = true;
  }

  void lock_center_from_pose()
  {
    // 第一次进入圆周阶段时锁定圆心，避免起飞/定点期间位姿变化导致圆轨迹漂移。
    if (center_locked_ || !pose_valid_) {
      return;
    }
    if (use_current_pose_as_start_) {
      center_.x = pose_.x - radius_ * std::cos(phase_);
      center_.y = pose_.y - radius_ * std::sin(phase_);
    }
    if (center_z_from_pose_) {
      center_.z = pose_.z;
    }
    center_locked_ = true;
    RCLCPP_INFO(
      get_logger(), "circle center locked at x=%.3f y=%.3f z=%.3f",
      center_.x, center_.y, center_.z);
  }

  Reference static_reference(double x, double y, double z) const
  {
    return {{x, y, z}, {}, {}, {}, {}};
  }

  Reference takeoff_reference() const
  {
    double x = takeoff_x_;
    double y = takeoff_y_;
    if (home_position_valid_ && use_current_xy_for_takeoff_) {
      x = home_position_.x;
      y = home_position_.y;
    }
    return static_reference(x, y, takeoff_altitude_);
  }

  Reference circle_reference_at(double elapsed) const
  {
    // 圆轨迹生成器：以锁定圆心为基准，按 phase + angular_velocity * t
    // 计算位置、速度、加速度、jerk、snap，供 PD 和 QSF 共用。
    // radius 是水平半径；angular_velocity 单位为 rad/s；circle_loops=0 表示无限绕圈。
    if (circle_loops_ > 0.0 && std::abs(angular_velocity_) > 1.0e-6) {
      const double max_elapsed = circle_loops_ * 2.0 * M_PI / std::abs(angular_velocity_);
      elapsed = std::min(elapsed, max_elapsed);
    }

    const double theta = phase_ + angular_velocity_ * elapsed;
    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);
    const double w = angular_velocity_;
    const double r = radius_;
    Reference ref;
    ref.pos = {center_.x + r * cos_t, center_.y + r * sin_t, center_.z};
    ref.vel = {-r * w * sin_t, r * w * cos_t, 0.0};
    ref.acc = {-r * w * w * cos_t, -r * w * w * sin_t, 0.0};
    ref.jerk = {r * w * w * w * sin_t, -r * w * w * w * cos_t, 0.0};
    ref.snap = {r * w * w * w * w * cos_t, r * w * w * w * w * sin_t, 0.0};
    return ref;
  }

  void update_flight_stage(double now)
  {
    // 绕圈状态机：preflight 预热 setpoint -> takeoff 起飞 -> hold 定点稳定 -> circle。
    // 只有进入 circle 后，active_reference() 才会返回圆轨迹；之前始终保持起飞点。
    if (mission_mode_ != "takeoff_then_circle" &&
      mission_mode_ != "takeoff_then_hold" &&
      mission_mode_ != "takeoff_then_wind_hold")
    {
      flight_stage_ = "circle";
      lock_center_from_pose();
      return;
    }

    if (flight_stage_ == "circle") {
      return;
    }
    if ((mission_mode_ == "takeoff_then_hold" ||
      mission_mode_ == "takeoff_then_wind_hold") && flight_stage_ == "hold")
    {
      maybe_request_offboard_and_arm(now);
      return;
    }
    if (mission_mode_ == "takeoff_then_wind_hold" &&
      flight_stage_ == "hold_settling")
    {
      const double required_hold = takeoff_settle_time_ + post_takeoff_hold_time_;
      if (takeoff_hold_start_valid_ && now - takeoff_hold_start_ >= required_hold) {
        flight_stage_ = "hold";
        RCLCPP_INFO(get_logger(), "position hold settled; handing over to QSF wind hold");
      }
      return;
    }
    if (!start_time_valid_) {
      flight_stage_ = "preflight";
      return;
    }
    if (now - start_time_ < preflight_setpoint_time_) {
      flight_stage_ = "preflight";
      return;
    }

    maybe_request_offboard_and_arm(now);
    if (!pose_valid_) {
      flight_stage_ = "takeoff";
      return;
    }

    if (pose_.z >= takeoff_altitude_ - takeoff_z_tolerance_) {
      if (!takeoff_hold_start_valid_) {
        takeoff_hold_start_ = now;
        takeoff_hold_start_valid_ = true;
        flight_stage_ = mission_mode_ == "takeoff_then_wind_hold" ?
          "hold_settling" : "hold";
        RCLCPP_INFO(get_logger(), "takeoff altitude reached; entering fixed-position hold");
        return;
      }
      if (mission_mode_ == "takeoff_then_wind_hold") {
        flight_stage_ = "hold_settling";
        return;
      }
      flight_stage_ = "hold";
      const double required_hold = takeoff_settle_time_ + post_takeoff_hold_time_;
      if (mission_mode_ == "takeoff_then_hold") {
        return;
      }
      if (now - takeoff_hold_start_ >= required_hold) {
        enter_circle(now);
      }
      return;
    }

    takeoff_hold_start_valid_ = false;
    flight_stage_ = "takeoff";
  }

  void enter_circle(double now)
  {
    // 锁定当前圆周起点和圆心，避免起飞阶段位姿变化导致圆轨迹漂移。
    flight_stage_ = "circle";
    circle_start_time_ = now;
    circle_start_time_valid_ = true;
    center_locked_ = false;
    lock_center_from_pose();
    RCLCPP_INFO(get_logger(), "takeoff hold confirmed; entering circle tracking");
  }

  Reference active_reference(double now)
  {
    // 非 circle 阶段返回固定起飞/悬停点；circle 阶段返回随时间变化的圆轨迹点。
    if (flight_stage_ != "circle") {
      return takeoff_reference();
    }
    lock_center_from_pose();
    const double start = circle_start_time_valid_ ? circle_start_time_ : start_time_;
    return circle_reference_at(std::max(0.0, now - start));
  }

  bool nav_goal_velocity_mode() const
  {
    return mission_mode_ == "nav_goal_velocity";
  }

  bool position_setpoint_mode() const
  {
    return controller_mode_ == "position" || controller_mode_ == "position_setpoint";
  }

  std::pair<bool, std::string> position_setpoint_gate() const
  {
    if (dry_run_) {
      return {false, "dry_run"};
    }
    if (!enable_real_setpoint_) {
      return {false, "real_setpoint_disabled"};
    }
    if (require_connected_ && !mavros_state_.connected) {
      return {false, "mavros_not_connected"};
    }
    // Position setpoints must already be streaming before PX4 accepts
    // OFFBOARD, so intentionally do not require OFFBOARD/armed here.
    return {true, "position_setpoint_active"};
  }

  std::pair<bool, std::string> nav_goal_velocity_gate() const
  {
    if (dry_run_) {
      return {false, "dry_run"};
    }
    if (!enable_real_setpoint_) {
      return {false, "real_setpoint_disabled"};
    }
    if (require_connected_ && !mavros_state_.connected) {
      return {false, "mavros_not_connected"};
    }
    if (require_offboard_ && mavros_state_.mode != offboard_mode_) {
      return {false, "not_offboard"};
    }
    if (require_armed_ && !mavros_state_.armed) {
      return {false, "not_armed"};
    }
    return {true, "nav_goal_velocity_active"};
  }

  geometry_msgs::msg::TwistStamped build_nav_goal_velocity_setpoint(
    const Vec3 & accel, const Vec3 & goal)
  {
    // 将 SLS 的受限加速度命令投影为短前视时间内的速度目标。位置误差已经在
    // compute_accel() 中进入 PD/QSF/LESO 抗风链路，避免额外复制一套控制律。
    Vec3 velocity_ref{
      velocity_.x + accel.x * nav_goal_velocity_lookahead_,
      velocity_.y + accel.y * nav_goal_velocity_lookahead_,
      velocity_.z + accel.z * nav_goal_velocity_lookahead_,
    };
    const Vec3 error = goal - pose_;
    if (std::abs(error.x) <= nav_goal_tolerance_) {
      velocity_ref.x = 0.0;
    }
    if (std::abs(error.y) <= nav_goal_tolerance_) {
      velocity_ref.y = 0.0;
    }
    if (std::abs(error.z) <= nav_goal_tolerance_) {
      velocity_ref.z = 0.0;
    }
    const double horizontal_speed = std::hypot(velocity_ref.x, velocity_ref.y);
    if (nav_goal_max_speed_xy_ > 0.0 && horizontal_speed > nav_goal_max_speed_xy_) {
      const double ratio = nav_goal_max_speed_xy_ / horizontal_speed;
      velocity_ref.x *= ratio;
      velocity_ref.y *= ratio;
    }
    velocity_ref.z = clamp_value(
      velocity_ref.z, -nav_goal_max_speed_z_, nav_goal_max_speed_z_);

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.twist.linear.x = velocity_ref.x;
    msg.twist.linear.y = velocity_ref.y;
    msg.twist.linear.z = velocity_ref.z;
    return msg;
  }

  void control_nav_goal_velocity(double now, double dt)
  {
    flight_stage_ = "nav_goal_velocity";
    if (!nav_goal_valid_ || now - nav_goal_time_ > nav_goal_stale_timeout_) {
      publish_sls_goal_control_active(false);
      publish_status(now, "nav_goal_stale", nullptr, nullptr, false);
      return;
    }

    const Reference ref = static_reference(nav_goal_.x, nav_goal_.y, nav_goal_.z);
    const Vec3 accel = compute_accel(ref, dt);
    last_commanded_accel_ = accel;
    last_commanded_accel_valid_ = true;
    reference_pub_->publish(build_reference_pose(ref.pos));
    // 此话题仅是交给 mavros_adapter 的内部速度参考，不是 MAVROS 真实输出；
    // 因而 dry-run 也发布，便于验证最终目标到桥接器的完整数据链。
    nav_velocity_setpoint_pub_->publish(build_nav_goal_velocity_setpoint(accel, nav_goal_));
    publish_sls_goal_control_active(true);
    command_publish_rate_.tick(now);

    const auto gate = nav_goal_velocity_gate();
    publish_status(now, gate.second, &ref, &accel, gate.first);
  }

  void publish_sls_goal_control_active(bool active)
  {
    if (!publish_sls_goal_control_active_) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = active;
    sls_goal_control_pub_->publish(msg);
  }

  void control_loop()
  {
    // 主循环只在位姿新鲜时输出控制，状态 JSON 会说明等待原因。
    const double now = now_seconds();
    control_loop_rate_.tick(now);
    if (!pose_valid_) {
      if (nav_goal_velocity_mode() || position_setpoint_mode() ||
        publish_sls_goal_control_active_)
      {
        publish_sls_goal_control_active(false);
      }
      publish_status(now, "waiting_for_pose", nullptr, nullptr, false);
      return;
    }
    const double pose_age = now - pose_time_;
    if (pose_age > pose_stale_timeout_) {
      if (nav_goal_velocity_mode() || position_setpoint_mode() ||
        publish_sls_goal_control_active_)
      {
        publish_sls_goal_control_active(false);
      }
      publish_status(now, "pose_stale", nullptr, nullptr, false);
      return;
    }

    double dt = 0.0;
    if (last_control_time_valid_) {
      dt = std::max(0.0, now - last_control_time_);
    }
    last_control_time_ = now;
    last_control_time_valid_ = true;

    if (nav_goal_velocity_mode()) {
      control_nav_goal_velocity(now, dt);
      return;
    }

    update_flight_stage(now);
    maybe_log_flight_progress(now);
    const Reference ref = active_reference(now);
    const auto reference_msg = build_reference_pose(ref.pos);
    reference_pub_->publish(reference_msg);

    // Pure PX4 position-loop mode: this node is only a circle trajectory
    // generator.  Never compute or publish AttitudeTarget/QSF/PD commands.
    if (position_setpoint_mode()) {
      const auto gate = position_setpoint_gate();
      // Renew exclusive setpoint ownership.  mavros_adapter suppresses its
      // idle velocity stream while this lease is alive.
      publish_sls_goal_control_active(gate.first);
      if (gate.first) {
        takeoff_pose_pub_->publish(reference_msg);
        command_publish_rate_.tick(now);
      }
      publish_status(now, gate.second, &ref, nullptr, gate.first);
      return;
    }

    const Vec3 accel = compute_accel(ref, dt);
    last_commanded_accel_ = accel;
    last_commanded_accel_valid_ = true;
    const auto attitude_msg = build_attitude_target(accel);

    debug_attitude_pub_->publish(attitude_msg);
    command_publish_rate_.tick(now);
    publish_takeoff_position(reference_msg);

    const auto gate = real_setpoint_gate();
    if (publish_sls_goal_control_active_) {
      // Renew exclusive ownership while publishing attitude/thrust, so the
      // MAVROS adapter cannot interleave its idle velocity setpoint stream.
      publish_sls_goal_control_active(gate.first);
    }
    // debug_attitude 永远发布，真实 MAVROS setpoint 只有通过 gate 后才发布。
    if (gate.first) {
      real_attitude_pub_->publish(attitude_msg);
    }
    publish_status(now, gate.second, &ref, &accel, gate.first);
  }

  Vec3 compute_accel(const Reference & ref, double dt)
  {
    // 仅供 PX4 SITL 标定悬停归一化推力：姿态保持水平，推力固定为 hover_thrust_。
    // 该模式必须由 launch 显式授权，防止被误用于正常控制或实机。
    if (controller_mode_ == "thrust_calibration" && allow_fixed_thrust_calibration_) {
      controller_source_ = "thrust_calibration";
      update_wind_compensation_debug({});
      return {};
    }

    // 控制优先走 QSF；QSF 出错时退回 PD，避免仿真或实机突然断 setpoint。
    Vec3 accel;
    bool have_accel = false;
    if (starts_with(controller_mode_, "qsf")) {
      have_accel = compute_qsf_accel(ref, dt, &accel);
    }
    if (!have_accel) {
      accel = compute_pd_accel(ref);
      controller_source_ = starts_with(controller_mode_, "qsf") ?
        "pd_fallback_qsf_error" : "pd";
    }

    if (enable_anti_wind_ || controller_mode_ == "leso_pd" || controller_mode_ == "leso") {
      accel = apply_anti_wind(ref, accel, dt);
    } else {
      update_wind_compensation_debug({});
    }
    // For an aircraft without a suspended load, the QSF pendulum model is
    // not an appropriate vertical controller.  Keep QSF for horizontal
    // motion, but use the ordinary position loop for z when requested.
    if (starts_with(controller_mode_, "qsf") && qsf_position_z_control_) {
      const Vec3 pd_accel = compute_pd_accel(ref);
      accel.z = pd_accel.z;
    }
    return limit_accel(accel);
  }

  Vec3 compute_pd_accel(const Reference & ref) const
  {
    const Vec3 err = ref.pos - pose_;
    const Vec3 vel_err = ref.vel - velocity_;
    return {
      ref.acc.x + kp_xy_ * err.x + kd_xy_ * vel_err.x,
      ref.acc.y + kp_xy_ * err.y + kd_xy_ * vel_err.y,
      ref.acc.z + kp_z_ * err.z + kd_z_ * vel_err.z,
    };
  }

  bool compute_qsf_accel(const Reference & ref, double dt, Vec3 * accel)
  {
    // QSF 生成代码使用 north/east/down 语义，外层状态来自 ROS ENU，需要显式映射。
    std::array<double, 12> state = build_sls_state();
    const std::array<double, 4> params{{load_mass_, mav_mass_, cable_length_, gravity_}};
    std::array<double, 5> ref_x{};
    std::array<double, 5> ref_y{};
    std::array<double, 5> ref_z{};
    build_qsf_refs(ref, &ref_x, &ref_y, &ref_z);
    qsf_state_ = state;
    qsf_ref_x_ = ref_x;
    qsf_ref_y_ = ref_y;
    qsf_ref_z_ = ref_z;

    std::array<double, 3> force{{0.0, 0.0, 0.0}};
    int rc = 0;
    if (controller_mode_ == "qsf_integral") {
      std::array<double, 15> qsf_state{};
      for (size_t i = 0; i < state.size(); ++i) {
        qsf_state[i] = state[i];
      }
      for (size_t i = 0; i < qsf_xi_.size(); ++i) {
        qsf_state[12 + i] = qsf_xi_[i];
      }
      const std::array<double, 13> gains{{
        qsf_ki_x_, qsf_kp_x_, qsf_kv_x_, qsf_ka_x_, qsf_kj_x_,
        qsf_ki_y_, qsf_kp_y_, qsf_kv_y_, qsf_ka_y_, qsf_kj_y_,
        qsf_ki_z_, qsf_kp_z_, qsf_kv_z_,
      }};
      std::array<double, 3> xi_dot{{0.0, 0.0, 0.0}};
      rc = sls_qsf_integral_controller(
        qsf_state.data(), gains.data(), params.data(), ref_x.data(), ref_y.data(),
        ref_z.data(), force.data(), xi_dot.data());
      integrate_qsf_xi(xi_dot, dt);
    } else {
      const std::array<double, 10> gains{{
        qsf_kp_x_, qsf_kv_x_, qsf_ka_x_, qsf_kj_x_,
        qsf_kp_y_, qsf_kv_y_, qsf_ka_y_, qsf_kj_y_,
        qsf_kp_z_, qsf_kv_z_,
      }};
      rc = sls_qsf_controller(
        state.data(), gains.data(), params.data(), ref_x.data(), ref_y.data(),
        ref_z.data(), force.data());
    }

    if (rc != 0 || !std::isfinite(force[0]) || !std::isfinite(force[1]) ||
      !std::isfinite(force[2]))
    {
      qsf_error_ = "QSF returned invalid force";
      const double now = now_seconds();
      if (now - last_qsf_warning_time_ > 1.0) {
        last_qsf_warning_time_ = now;
        RCLCPP_WARN(get_logger(), "QSF fallback to PD: %s", qsf_error_.c_str());
      }
      return false;
    }

    qsf_force_ned_ = force;
    qsf_error_.clear();
    controller_source_ = controller_mode_;

    // ROS1 中 a_des=(F_ned_y, F_ned_x, -F_ned_z)/m，然后几何控制器再减重力。
    // 这里保持 MAVROS 姿态 setpoint 的“净加速度”接口，所以先得到总推力加速度再减 g。
    const Vec3 thrust_accel_enu{
      force[1] / mav_mass_,
      force[0] / mav_mass_,
      -force[2] / mav_mass_,
    };
    qsf_thrust_accel_enu_ = thrust_accel_enu;
    *accel = {thrust_accel_enu.x, thrust_accel_enu.y, thrust_accel_enu.z - gravity_};
    qsf_net_accel_enu_ = *accel;
    return true;
  }

  std::array<double, 12> build_sls_state()
  {
    // 状态顺序来自 ROS1/drown QSF 代码：负载位置、摆杆方向、负载速度、摆杆角速度。
    const LoadState load = current_load_state();
    Vec3 pend = load.pos - pose_;
    pend = normalize3(pend, {0.0, 0.0, -1.0});
    Vec3 pend_rate{};
    if (load.active) {
      pend_rate = cross(pend, load.vel - velocity_);
    }

    return {{
      load.pos.y, load.pos.x, -load.pos.z,
      pend.y, pend.x, -pend.z,
      load.vel.y, load.vel.x, -load.vel.z,
      pend_rate.y, pend_rate.x, -pend_rate.z,
    }};
  }

  void build_qsf_refs(
    const Reference & ref, std::array<double, 5> * ref_x,
    std::array<double, 5> * ref_y, std::array<double, 5> * ref_z) const
  {
    // ref_x/ref_y/ref_z 分别对应 QSF 的 north/east/down 轴，不是 ROS ENU 的 x/y/z。
    double pos_z = ref.pos.z;
    if (!qsf_reference_is_load_) {
      pos_z -= cable_length_;
    }
    *ref_x = {{ref.pos.y, ref.vel.y, ref.acc.y, ref.jerk.y, ref.snap.y}};
    *ref_y = {{ref.pos.x, ref.vel.x, ref.acc.x, ref.jerk.x, ref.snap.x}};
    *ref_z = {{-pos_z, -ref.vel.z, -ref.acc.z, -ref.jerk.z, -ref.snap.z}};
  }

  LoadState current_load_state()
  {
    // 有 Gazebo/传感器负载位姿时使用真实负载，否则用机体下方 cable_length 的虚拟负载。
    const double now = now_seconds();
    if (use_load_pose_ && load_pose_valid_ &&
      now - load_pose_time_ <= load_pose_stale_timeout_)
    {
      return {load_pose_, load_velocity_, true};
    }
    return {{pose_.x, pose_.y, pose_.z - cable_length_}, velocity_, false};
  }

  void integrate_qsf_xi(const std::array<double, 3> & xi_dot, double dt)
  {
    if (dt <= 0.0) {
      return;
    }
    for (size_t i = 0; i < qsf_xi_.size(); ++i) {
      qsf_xi_[i] = clamp_value(
        qsf_xi_[i] + xi_dot[i] * dt,
        -qsf_integral_limit_, qsf_integral_limit_);
    }
  }

  std::pair<Vec3, bool> estimate_cable_force_on_drone()
  {
    // residual 风估计需要扣除吊载对机体的水平拉力，否则会把绳摆误判成风。
    const double now = now_seconds();
    if (!pose_valid_ || !load_pose_valid_ || load_mass_ <= 0.0 ||
      now - load_pose_time_ > load_pose_stale_timeout_)
    {
      return {{}, false};
    }
    const Vec3 cable = load_pose_ - pose_;
    const Vec3 dir = normalize3(cable, {0.0, 0.0, -1.0});
    const double down_component = std::max(0.15, -dir.z);
    const double tension = load_mass_ * gravity_ / down_component;
    return {{tension * dir.x, tension * dir.y, 0.0}, true};
  }

  double effective_wind_compensation_gain(double now)
  {
    const double target_gain = std::max(0.0, wind_compensation_gain_);
    if (target_gain <= 0.0 || !start_time_valid_) {
      effective_wind_compensation_gain_ = 0.0;
      return effective_wind_compensation_gain_;
    }
    const double elapsed = std::max(0.0, now - start_time_);
    if (elapsed < wind_compensation_warmup_time_) {
      effective_wind_compensation_gain_ = 0.0;
      return effective_wind_compensation_gain_;
    }
    if (wind_compensation_ramp_time_ <= 0.0) {
      effective_wind_compensation_gain_ = target_gain;
      return effective_wind_compensation_gain_;
    }
    const double ramp =
      clamp_value((elapsed - wind_compensation_warmup_time_) / wind_compensation_ramp_time_, 0.0, 1.0);
    effective_wind_compensation_gain_ = target_gain * ramp;
    return effective_wind_compensation_gain_;
  }

  Vec3 apply_anti_wind(const Reference & ref, const Vec3 & accel, double dt)
  {
    // 抗风只处理 ENU x/y；z 轴扰动和积分强制清零，避免风估计改高度通道。
    if (dt <= 0.0) {
      return accel;
    }
    const double now = now_seconds();
    const double compensation_gain = effective_wind_compensation_gain(now);
    const Vec3 observed = pose_;
    Vec3 compensated = accel;
    Vec3 compensation_accel{};
    double observed_axis[3] = {observed.x, observed.y, observed.z};
    double ref_axis[3] = {ref.pos.x, ref.pos.y, ref.pos.z};
    double observed_accel_axis[3] = {observed_accel_.x, observed_accel_.y, observed_accel_.z};
    const auto cable_force = estimate_cable_force_on_drone();
    double cable_force_axis[3] = {
      cable_force.first.x, cable_force.first.y, cable_force.first.z};
    double last_command_axis[3] = {
      last_commanded_accel_valid_ ? last_commanded_accel_.x : accel.x,
      last_commanded_accel_valid_ ? last_commanded_accel_.y : accel.y,
      last_commanded_accel_valid_ ? last_commanded_accel_.z : accel.z,
    };
    for (size_t axis = 0; axis < 3; ++axis) {
      if (axis == 2) {
        disturbance_[axis] = 0.0;
        wind_integral_[axis] = 0.0;
        continue;
      }
      if (wind_estimator_mode_ == "actual_feedback" && actual_wind_time_valid_) {
        update_actual_feedback_wind_axis(axis, dt);
      } else if (wind_estimator_mode_ == "residual" || wind_estimator_mode_ == "actual_feedback") {
        update_residual_wind_axis(
          axis, observed_accel_axis[axis], last_command_axis[axis],
          cable_force_axis[axis], cable_force.second, dt);
      } else {
        disturbance_[axis] = leso_[axis].update(
          observed_axis[axis], last_command_axis[axis], dt);
      }
      wind_integral_[axis] += (ref_axis[axis] - observed_axis[axis]) * dt;
      wind_integral_[axis] = clamp_value(
        wind_integral_[axis], -wind_integral_limit_, wind_integral_limit_);
      const double residual =
        disturbance_[axis] - wind_integral_gain_ * wind_integral_[axis];
      const double comp = -compensation_gain * residual;
      if (axis == 0) {
        compensation_accel.x = comp;
        compensated.x += comp;
      } else if (axis == 1) {
        compensation_accel.y = comp;
        compensated.y += comp;
      }
    }

    update_wind_compensation_debug(compensation_accel);
    auto msg = geometry_msgs::msg::Vector3Stamped();
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;
    const Vec3 estimate_force = wind_estimate_force_n();
    msg.vector.x = estimate_force.x;
    msg.vector.y = estimate_force.y;
    msg.vector.z = estimate_force.z;
    wind_estimate_pub_->publish(msg);
    maybe_log_wind_debug();
    return compensated;
  }

  void update_residual_wind_axis(
    size_t axis, double observed_accel, double commanded_accel,
    double cable_force_on_drone, bool cable_force_valid, double dt)
  {
    // residual 模式用“观测加速度 - 上一拍命令加速度 - 绳力影响”估计外界水平扰动。
    if (!observed_accel_valid_ || !last_commanded_accel_valid_) {
      return;
    }
    double raw = observed_accel - commanded_accel;
    if (cable_force_valid && axis < 2) {
      raw -= cable_force_on_drone / mav_mass_;
    }
    if (wind_estimate_force_limit_ > 0.0) {
      const double accel_limit = wind_estimate_force_limit_ / mav_mass_;
      raw = clamp_value(raw, -accel_limit, accel_limit);
    }
    const double alpha = 1.0 - std::exp(-dt / wind_estimate_filter_tau_);
    disturbance_[axis] += alpha * (raw - disturbance_[axis]);
  }

  void update_actual_feedback_wind_axis(size_t axis, double dt)
  {
    // actual_feedback 只用于仿真调参：直接跟踪桥接节点发布的真实模拟风力。
    double target_force = axis == 0 ? actual_wind_force_.x : actual_wind_force_.y;
    if (wind_estimate_force_limit_ > 0.0) {
      target_force = clamp_value(
        target_force, -wind_estimate_force_limit_, wind_estimate_force_limit_);
    }
    const double target_accel = target_force / mav_mass_;
    const double alpha = 1.0 - std::exp(-dt / wind_estimate_filter_tau_);
    disturbance_[axis] += alpha * (target_accel - disturbance_[axis]);
  }

  Vec3 wind_estimate_force_n() const
  {
    return {
      disturbance_[0] * mav_mass_,
      disturbance_[1] * mav_mass_,
      0.0,
    };
  }

  Vec3 wind_estimate_rel_error_xy() const
  {
    const Vec3 estimate = wind_estimate_force_n();
    return {
      std::abs(actual_wind_force_.x) > 1.0e-6 ?
      std::abs(estimate.x - actual_wind_force_.x) / std::abs(actual_wind_force_.x) : 0.0,
      std::abs(actual_wind_force_.y) > 1.0e-6 ?
      std::abs(estimate.y - actual_wind_force_.y) / std::abs(actual_wind_force_.y) : 0.0,
      0.0,
    };
  }

  void update_wind_compensation_debug(const Vec3 & compensation_accel)
  {
    wind_compensation_accel_ = compensation_accel;
    wind_compensation_force_ = compensation_accel * mav_mass_;
    wind_compensation_magnitude_ = norm3(wind_compensation_force_);
    wind_compensation_direction_ = unit_vector(wind_compensation_force_);
  }

  void maybe_log_wind_debug()
  {
    if (!enable_wind_debug_log_) {
      return;
    }
    const double now = now_seconds();
    if (now - last_wind_debug_log_time_ < wind_debug_log_period_) {
      return;
    }
    last_wind_debug_log_time_ = now;
    const std::string actual_age = actual_wind_time_valid_ ?
      std::to_string(now - actual_wind_time_) + "s" : "none";
    const Vec3 estimate_force = wind_estimate_force_n();
    const Vec3 estimate_error = wind_estimate_rel_error_xy();
    RCLCPP_INFO(
      get_logger(),
      "wind estimate force_n=(%.3f, %.3f, %.3f) err_xy=(%.2f, %.2f); "
      "wind compensation accel_mps2=(%.3f, %.3f, %.3f) equiv_force_n=(%.3f, %.3f, %.3f) "
      "mag_n=%.3f dir=(%.3f, %.3f, %.3f); actual_wind_n=(%.3f, %.3f, %.3f) "
      "mag_n=%.3f dir=(%.3f, %.3f, %.3f) age=%s; rt_hz target=%.1f control=%.1f "
      "cmd=%.1f pose=%.1f velocity=%.1f load_pose=%.1f actual_wind=%.1f",
      estimate_force.x, estimate_force.y, estimate_force.z,
      estimate_error.x, estimate_error.y,
      wind_compensation_accel_.x, wind_compensation_accel_.y, wind_compensation_accel_.z,
      wind_compensation_force_.x, wind_compensation_force_.y, wind_compensation_force_.z,
      wind_compensation_magnitude_,
      wind_compensation_direction_.x, wind_compensation_direction_.y,
      wind_compensation_direction_.z,
      actual_wind_force_.x, actual_wind_force_.y, actual_wind_force_.z,
      actual_wind_magnitude_,
      actual_wind_direction_.x, actual_wind_direction_.y, actual_wind_direction_.z,
      actual_age.c_str(),
      control_rate_, control_loop_rate_.hz(), command_publish_rate_.hz(),
      pose_input_rate_.hz(), velocity_input_rate_.hz(),
      load_pose_input_rate_.hz(), actual_wind_input_rate_.hz());
  }

  void maybe_log_flight_progress(double now)
  {
    const bool stage_changed = flight_stage_ != last_logged_flight_stage_;
    if (!stage_changed && now - last_flight_log_time_ < 1.0) {
      return;
    }
    last_flight_log_time_ = now;
    last_logged_flight_stage_ = flight_stage_;

    double elapsed = 0.0;
    double completed_loops = 0.0;
    double progress = 0.0;
    if (flight_stage_ == "circle" && circle_start_time_valid_) {
      elapsed = std::max(0.0, now - circle_start_time_);
      completed_loops = std::abs(angular_velocity_) * elapsed / (2.0 * M_PI);
      if (circle_loops_ > 0.0) {
        completed_loops = std::min(completed_loops, circle_loops_);
        progress = 100.0 * completed_loops / circle_loops_;
      }
    }

    if (circle_loops_ > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "flight stage=%s circle_progress=%.1f%% loops=%.2f/%.2f elapsed=%.1fs",
        flight_stage_.c_str(), progress, completed_loops, circle_loops_, elapsed);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "flight stage=%s circle_progress=continuous loops=%.2f elapsed=%.1fs",
        flight_stage_.c_str(), completed_loops, elapsed);
    }
  }

  Vec3 limit_accel(Vec3 accel) const
  {
    // 先限各轴和总加速度，再用最大倾角约束水平加速度，避免姿态目标过激。
    accel.x = clamp_value(accel.x, -max_acc_xy_, max_acc_xy_);
    accel.y = clamp_value(accel.y, -max_acc_xy_, max_acc_xy_);
    accel.z = clamp_value(accel.z, -max_acc_z_, max_acc_z_);

    const double total = norm3(accel);
    if (max_total_acc_ > 0.0 && total > max_total_acc_) {
      accel = accel * (max_total_acc_ / total);
    }

    const double horizontal = std::sqrt(accel.x * accel.x + accel.y * accel.y);
    const double vertical_thrust = std::max(1.0e-6, gravity_ + accel.z);
    const double max_horizontal = std::tan(max_tilt_rad_) * vertical_thrust;
    if (horizontal > max_horizontal) {
      const double scale = max_horizontal / horizontal;
      accel.x *= scale;
      accel.y *= scale;
    }
    return accel;
  }

  mavros_msgs::msg::AttitudeTarget build_attitude_target(const Vec3 & accel)
  {
    // 把净加速度命令转成 MAVROS attitude target，角速度通道全部忽略。
    const auto quat = attitude_from_net_accel(accel, yaw_, gravity_);
    const double thrust_norm = norm3({accel.x, accel.y, accel.z + gravity_});
    const double thrust = clamp_value(
      hover_thrust_ * thrust_norm / gravity_, min_thrust_, max_thrust_);

    auto msg = mavros_msgs::msg::AttitudeTarget();
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.type_mask =
      mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
      mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
      mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
    msg.orientation.x = quat[0];
    msg.orientation.y = quat[1];
    msg.orientation.z = quat[2];
    msg.orientation.w = quat[3];
    msg.thrust = thrust;
    return msg;
  }

  geometry_msgs::msg::PoseStamped build_reference_pose(const Vec3 & pos)
  {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = pos.x;
    msg.pose.position.y = pos.y;
    msg.pose.position.z = pos.z;
    msg.pose.orientation.w = 1.0;
    return msg;
  }

  void publish_takeoff_position(const geometry_msgs::msg::PoseStamped & msg)
  {
    // 起飞前以位置目标给 PX4 预热 OFFBOARD；一旦已解锁并进入 OFFBOARD，
    // 必须停止位置目标，避免和 PD/QSF 的姿态+推力目标并发竞争。
    // 起飞瞬态仍使用 PX4 位置控制，确认已到达目标高度后再进行姿态控制交接。
    // 这避免带吊载模型在离地瞬间受到未标定推力的冲击。
    const bool attitude_handover_stage =
      flight_stage_ == "hold" || flight_stage_ == "circle";
    const bool attitude_handover_active = attitude_handover_stage &&
      !dry_run_ && enable_real_setpoint_ && mavros_state_.connected &&
      mavros_state_.armed && mavros_state_.mode == offboard_mode_;
    if (flight_stage_ == "circle" || attitude_handover_active || dry_run_ ||
      !enable_takeoff_position_setpoint_)
    {
      return;
    }
    takeoff_pose_pub_->publish(msg);
  }

  void maybe_request_offboard_and_arm(double now)
  {
    // 只在非 dry-run 且显式启用服务控制时自动切 OFFBOARD/ARM。
    if (dry_run_ || !enable_mavros_services_) {
      return;
    }
    if (
      auto_offboard_ && !offboard_seen_ && !manual_mode_override_latched_ &&
      mavros_state_.mode != offboard_mode_)
    {
      if (!mode_request_pending_ && now - last_mode_request_time_ >= service_retry_period_) {
        last_mode_request_time_ = now;
        if (set_mode_client_->service_is_ready()) {
          mode_request_pending_ = true;
          auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
          req->custom_mode = offboard_mode_;
          RCLCPP_INFO(
            get_logger(), "requesting FCU mode switch to %s", offboard_mode_.c_str());
          set_mode_client_->async_send_request(
            req,
            [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
              const auto response = future.get();
              mode_request_pending_ = false;
              RCLCPP_INFO(
                get_logger(), "set_mode response: mode_sent=%s",
                response->mode_sent ? "true" : "false");
            });
        } else {
          RCLCPP_WARN(get_logger(), "set_mode service is not ready");
        }
      }
    }
    if (auto_arm_ && !armed_seen_ && !manual_disarm_latched_ && !mavros_state_.armed) {
      if (!arm_request_pending_ && now - last_arm_request_time_ >= service_retry_period_) {
        last_arm_request_time_ = now;
        if (arming_client_->service_is_ready()) {
          arm_request_pending_ = true;
          auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
          req->value = true;
          RCLCPP_INFO(get_logger(), "requesting FCU arm");
          arming_client_->async_send_request(
            req,
            [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
              const auto response = future.get();
              arm_request_pending_ = false;
              RCLCPP_INFO(
                get_logger(), "arming response: success=%s result=%u",
                response->success ? "true" : "false",
                static_cast<unsigned>(response->result));
            });
        } else {
          RCLCPP_WARN(get_logger(), "arming service is not ready");
        }
      }
    }
  }

  std::pair<bool, std::string> real_setpoint_gate() const
  {
    // 真实 setpoint gate 是最后一道保护，状态字符串会写入 /sls_circle/status。
    // preflight/takeoff 仅使用位置设定值：先让 PX4 平稳离地，随后在 hold/circle
    // 才交给 PD/QSF 姿态+推力控制。
    if (flight_stage_ != "hold" && flight_stage_ != "circle") {
      return {false, flight_stage_};
    }
    if (dry_run_) {
      return {false, "dry_run"};
    }
    if (!enable_real_setpoint_) {
      return {false, "real_setpoint_disabled"};
    }
    if (require_connected_ && !mavros_state_.connected) {
      return {false, "mavros_not_connected"};
    }
    if (require_offboard_ && mavros_state_.mode != offboard_mode_) {
      return {false, "not_offboard"};
    }
    if (require_armed_ && !mavros_state_.armed) {
      return {false, "not_armed"};
    }
    return {true, "real_setpoint_active"};
  }

  void publish_status(
    double now, const std::string & reason, const Reference * ref,
    const Vec3 * accel, bool real_active)
  {
    // 状态以 JSON 字符串发布，便于 ros2 topic echo、日志记录和后续脚本解析。
    if (now - last_status_time_ < status_period_) {
      return;
    }
    last_status_time_ = now;
    const LoadState load = pose_valid_ ? current_load_state() : LoadState{};
    const Vec3 estimate_force = wind_estimate_force_n();
    const Vec3 estimate_error = wind_estimate_rel_error_xy();
    const double estimate_error_max = std::max(estimate_error.x, estimate_error.y);
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "{"
        << "\"reason\":\"" << json_escape(reason) << "\","
        << "\"flight_stage\":\"" << json_escape(flight_stage_) << "\","
        << "\"dry_run\":" << json_bool(dry_run_) << ","
        << "\"enable_real_setpoint\":" << json_bool(enable_real_setpoint_) << ","
        << "\"real_setpoint_active\":" << json_bool(real_active) << ","
        << "\"mission_mode\":\"" << json_escape(mission_mode_) << "\","
        << "\"post_takeoff_hold_time\":" << post_takeoff_hold_time_ << ","
        << "\"controller_mode\":\"" << json_escape(controller_mode_) << "\","
        << "\"controller_source\":\"" << json_escape(controller_source_) << "\","
        << "\"mav_mass\":" << mav_mass_ << ","
        << "\"target_control_hz\":" << control_rate_ << ","
        << "\"actual_control_hz\":" << control_loop_rate_.hz() << ","
        << "\"command_publish_hz\":" << command_publish_rate_.hz() << ","
        << "\"pose_input_hz\":" << pose_input_rate_.hz() << ","
        << "\"velocity_input_hz\":" << velocity_input_rate_.hz() << ","
        << "\"load_pose_input_hz\":" << load_pose_input_rate_.hz() << ","
        << "\"actual_wind_input_hz\":" << actual_wind_input_rate_.hz() << ","
        << "\"qsf_core_loaded\":true,"
        << "\"qsf_core_version\":\"" << json_escape(qsf_core_version_) << "\","
        << "\"qsf_error\":\"" << json_escape(qsf_error_) << "\","
        << "\"qsf_force_ned\":" << json_array(qsf_force_ned_) << ","
        << "\"qsf_xi\":" << json_array(qsf_xi_) << ","
        << "\"qsf_state_ned_load_semantics\":" << json_array(qsf_state_) << ","
        << "\"qsf_ref_x_north\":" << json_array(qsf_ref_x_) << ","
        << "\"qsf_ref_y_east\":" << json_array(qsf_ref_y_) << ","
        << "\"qsf_ref_z_down\":" << json_array(qsf_ref_z_) << ","
        << "\"qsf_thrust_accel_enu\":" << json_vec(qsf_thrust_accel_enu_) << ","
        << "\"qsf_net_accel_enu\":" << json_vec(qsf_net_accel_enu_) << ","
        << "\"pose_received\":" << json_bool(pose_valid_) << ","
        << "\"pose_age\":" << (pose_valid_ ? std::to_string(now - pose_time_) : "null") << ","
        << "\"velocity_topic_active\":" << json_bool(
             use_velocity_topic_ && velocity_topic_valid_ &&
             now - velocity_time_ <= velocity_stale_timeout_) << ","
        << "\"velocity_age\":"
        << (velocity_topic_valid_ ? std::to_string(now - velocity_time_) : "null") << ","
        << "\"load_pose_active\":" << json_bool(load.active) << ","
        << "\"mavros_connected\":" << json_bool(mavros_state_.connected) << ","
        << "\"mavros_armed\":" << json_bool(mavros_state_.armed) << ","
        << "\"mavros_mode\":\"" << json_escape(mavros_state_.mode) << "\","
        << "\"offboard_seen\":" << json_bool(offboard_seen_) << ","
        << "\"manual_mode_override_latched\":" <<
          json_bool(manual_mode_override_latched_) << ","
        << "\"armed_seen\":" << json_bool(armed_seen_) << ","
        << "\"manual_disarm_latched\":" << json_bool(manual_disarm_latched_) << ","
        << "\"center\":" << json_vec(center_) << ","
        << "\"reference\":" << (ref ? json_vec(ref->pos) : "null") << ","
        << "\"reference_velocity\":" << (ref ? json_vec(ref->vel) : "null") << ","
        << "\"position\":" << (pose_valid_ ? json_vec(pose_) : "null") << ","
        << "\"velocity\":" << json_vec(velocity_) << ","
        << "\"accel_cmd\":" << (accel ? json_vec(*accel) : "null") << ","
        << "\"wind_estimator_mode\":\"" << json_escape(wind_estimator_mode_) << "\","
        << "\"wind_compensation_gain_target\":" << wind_compensation_gain_ << ","
        << "\"wind_compensation_gain_effective\":" << effective_wind_compensation_gain_ << ","
        << "\"wind_compensation_warmup_time\":" << wind_compensation_warmup_time_ << ","
        << "\"wind_compensation_ramp_time\":" << wind_compensation_ramp_time_ << ","
        << "\"wind_estimate_force_n\":" << json_vec(estimate_force) << ","
        << "\"wind_estimate_rel_error_xy\":" << json_vec(estimate_error) << ","
        << "\"wind_estimate_rel_error_xy_max\":" << estimate_error_max << ","
        << "\"disturbance_estimate\":" << json_array(disturbance_) << ","
        << "\"wind_integral\":" << json_array(wind_integral_) << ","
        << "\"wind_compensation_accel_mps2\":" << json_vec(wind_compensation_accel_) << ","
        << "\"wind_compensation_force_n\":" << json_vec(wind_compensation_force_) << ","
        << "\"wind_compensation_magnitude_n\":" << wind_compensation_magnitude_ << ","
        << "\"wind_compensation_direction\":" << json_vec(wind_compensation_direction_) << ","
        << "\"actual_wind_force_n\":" << json_vec(actual_wind_force_) << ","
        << "\"actual_wind_magnitude_n\":" << actual_wind_magnitude_ << ","
        << "\"actual_wind_direction\":" << json_vec(actual_wind_direction_) << ","
        << "\"actual_wind_age\":"
        << (actual_wind_time_valid_ ? std::to_string(now - actual_wind_time_) : "null")
        << "}";
    auto msg = std_msgs::msg::String();
    msg.data = out.str();
    status_pub_->publish(msg);
  }

  // 参数缓存。
  bool dry_run_{true};
  bool enable_real_setpoint_{false};
  std::string pose_topic_;
  std::string velocity_topic_;
  bool use_velocity_topic_{true};
  double velocity_stale_timeout_{0.5};
  std::string load_pose_topic_;
  bool use_load_pose_{false};
  double load_pose_stale_timeout_{0.5};
  std::string state_topic_;
  std::string real_attitude_topic_;
  std::string debug_attitude_topic_;
  std::string takeoff_pose_topic_;
  std::string reference_pose_topic_;
  std::string status_topic_;
  std::string wind_estimate_topic_;
  std::string actual_wind_topic_;
  std::string nav_goal_topic_;
  std::string nav_velocity_setpoint_topic_;
  bool publish_sls_goal_control_active_{false};
  std::string sls_goal_control_topic_;
  double nav_goal_stale_timeout_{1.0};
  double nav_goal_velocity_lookahead_{0.4};
  double nav_goal_max_speed_xy_{1.0};
  double nav_goal_max_speed_z_{0.5};
  double nav_goal_tolerance_{0.10};
  std::string frame_id_;
  double control_rate_{100.0};
  double pose_stale_timeout_{0.5};
  std::string mission_mode_{"circle_only"};
  double preflight_setpoint_time_{2.0};
  double takeoff_altitude_{1.0};
  double takeoff_x_{0.0};
  double takeoff_y_{0.0};
  bool use_current_xy_for_takeoff_{true};
  double takeoff_z_tolerance_{0.15};
  double takeoff_settle_time_{2.0};
  double post_takeoff_hold_time_{5.0};
  bool enable_takeoff_position_setpoint_{true};
  bool enable_mavros_services_{false};
  bool auto_offboard_{false};
  bool auto_arm_{false};
  std::string set_mode_service_;
  std::string arming_service_;
  std::string offboard_mode_{"OFFBOARD"};
  double service_retry_period_{1.0};
  bool use_current_pose_as_start_{true};
  Vec3 center_{0.0, 0.0, 1.0};
  bool center_z_from_pose_{true};
  double radius_{1.0};
  double angular_velocity_{0.35};
  double circle_loops_{0.0};
  double phase_{0.0};
  double yaw_{0.0};
  double kp_xy_{1.8};
  double kp_z_{2.5};
  double kd_xy_{1.4};
  double kd_z_{1.6};
  double max_acc_xy_{2.5};
  double max_acc_z_{2.0};
  double max_total_acc_{4.0};
  double max_tilt_deg_{25.0};
  double max_tilt_rad_{25.0 * M_PI / 180.0};
  double gravity_{9.80665};
  double hover_thrust_{0.5};
  double min_thrust_{0.05};
  double max_thrust_{0.85};
  bool allow_fixed_thrust_calibration_{false};
  bool require_connected_{true};
  bool require_offboard_{false};
  bool require_armed_{false};
  std::string controller_mode_{"pd"};
  double mav_mass_{1.56};
  double load_mass_{0.25};
  double cable_length_{0.85};
  double qsf_kp_x_{10.0};
  double qsf_kv_x_{5.0};
  bool qsf_position_z_control_{false};
  double qsf_ka_x_{0.0};
  double qsf_kj_x_{0.0};
  double qsf_kp_y_{10.0};
  double qsf_kv_y_{5.0};
  double qsf_ka_y_{0.0};
  double qsf_kj_y_{0.0};
  double qsf_kp_z_{20.0};
  double qsf_kv_z_{10.0};
  double qsf_ki_x_{12.0};
  double qsf_ki_y_{12.0};
  double qsf_ki_z_{1.0};
  double qsf_integral_limit_{10.0};
  bool qsf_reference_is_load_{false};
  bool enable_anti_wind_{false};
  std::string wind_estimator_mode_{"residual"};
  double wind_observer_bandwidth_{4.0};
  double wind_estimate_filter_tau_{0.5};
  double wind_estimate_force_limit_{5.0};
  double wind_compensation_gain_{0.7};
  double wind_compensation_warmup_time_{0.0};
  double wind_compensation_ramp_time_{0.0};
  double effective_wind_compensation_gain_{0.0};
  double wind_integral_gain_{0.0};
  double wind_integral_limit_{5.0};
  bool enable_wind_debug_log_{true};
  double wind_debug_log_period_{1.0};
  double status_period_{0.25};

  // 输入状态缓存。
  Vec3 pose_;
  Vec3 velocity_;
  Vec3 observed_accel_;
  double pose_time_{0.0};
  double velocity_time_{0.0};
  bool pose_valid_{false};
  bool velocity_valid_{false};
  bool velocity_topic_valid_{false};
  bool observed_accel_valid_{false};
  Vec3 nav_goal_;
  double nav_goal_time_{0.0};
  bool nav_goal_valid_{false};
  Vec3 load_pose_;
  Vec3 load_velocity_;
  Vec3 load_observed_accel_;
  double load_pose_time_{0.0};
  bool load_pose_valid_{false};
  bool load_velocity_valid_{false};
  bool load_observed_accel_valid_{false};
  mavros_msgs::msg::State mavros_state_;
  bool mavros_state_received_{false};
  bool offboard_seen_{false};
  bool armed_seen_{false};
  bool manual_mode_override_latched_{false};
  bool manual_disarm_latched_{false};
  Vec3 home_position_;
  bool home_position_valid_{false};
  double start_time_{0.0};
  bool start_time_valid_{false};
  double circle_start_time_{0.0};
  bool circle_start_time_valid_{false};
  double takeoff_hold_start_{0.0};
  bool takeoff_hold_start_valid_{false};
  double last_control_time_{0.0};
  bool last_control_time_valid_{false};
  Vec3 last_commanded_accel_;
  bool last_commanded_accel_valid_{false};
  std::string flight_stage_{"preflight"};
  bool center_locked_{false};
  double last_mode_request_time_{0.0};
  double last_arm_request_time_{0.0};
  bool mode_request_pending_{false};
  bool arm_request_pending_{false};
  double last_status_time_{0.0};
  double last_qsf_warning_time_{0.0};

  // 抗风估计、补偿和频率调试缓存。
  std::array<LesoAxis, 3> leso_;
  std::array<double, 3> disturbance_{{0.0, 0.0, 0.0}};
  std::array<double, 3> wind_integral_{{0.0, 0.0, 0.0}};
  Vec3 wind_compensation_accel_;
  Vec3 wind_compensation_force_;
  double wind_compensation_magnitude_{0.0};
  Vec3 wind_compensation_direction_;
  Vec3 actual_wind_force_;
  double actual_wind_magnitude_{0.0};
  Vec3 actual_wind_direction_;
  double actual_wind_time_{0.0};
  bool actual_wind_time_valid_{false};
  double last_wind_debug_log_time_{0.0};
  double last_flight_log_time_{0.0};
  std::string last_logged_flight_stage_;
  RateMeter control_loop_rate_;
  RateMeter command_publish_rate_;
  RateMeter pose_input_rate_;
  RateMeter velocity_input_rate_;
  RateMeter load_pose_input_rate_;
  RateMeter actual_wind_input_rate_;
  // QSF 调用快照，全部写入 status 方便对照 ROS1/drown 公式和坐标映射。
  std::array<double, 3> qsf_force_ned_{{0.0, 0.0, 0.0}};
  std::array<double, 3> qsf_xi_{{0.0, 0.0, 0.0}};
  std::array<double, 12> qsf_state_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 5> qsf_ref_x_{{0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 5> qsf_ref_y_{{0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 5> qsf_ref_z_{{0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec3 qsf_thrust_accel_enu_;
  Vec3 qsf_net_accel_enu_;
  std::string qsf_error_;
  std::string controller_source_{"startup"};
  std::string qsf_core_version_;

  // ROS 通信对象。
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr load_pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr actual_wind_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr debug_attitude_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr real_attitude_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr takeoff_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_estimate_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr nav_velocity_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sls_goal_control_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlsCircleControllerCpp>(argc, argv);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
