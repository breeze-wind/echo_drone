#include <chrono>
#include <memory>

#include "robot_behaviors/plugins/keep_away_from_obstacles.hpp"
#include <nav2_util/node_utils.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace nav2_behaviors
{

    KeepAwayFromObstacles::KeepAwayFromObstacles()
        : TimedBehavior<robot_msgs::action::KeepAwayFromObstacles>(),
          feedback_(std::make_shared<robot_msgs::action::KeepAwayFromObstacles::Feedback>()),
          speed_(1.0),
          distance_(1.0),
          robot_radius_(1.0),
          is_first_cycle_(true)
    {
    }

    void KeepAwayFromObstacles::onConfigure()
    {
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node"};
        }
        nav2_util::declare_parameter_if_not_declared(node, "point_topic", rclcpp::ParameterValue(std::string("point_cloud")));
        nav2_util::declare_parameter_if_not_declared(node, "robot_radius", rclcpp::ParameterValue(1.0));
        node->get_parameter("point_topic", point_topic_);
        node->get_parameter("robot_radius", robot_radius_);
        point_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&KeepAwayFromObstacles::point_callback, this, std::placeholders::_1));
    }

    void KeepAwayFromObstacles::point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        point_cloud_ = *msg;
    }

    KeepAwayFromObstacles::~KeepAwayFromObstacles() = default;

    Status KeepAwayFromObstacles::onRun(const std::shared_ptr<const robot_msgs::action::KeepAwayFromObstacles::Goal> command)
    {
        speed_ = command->speed;
        distance_ = command->distance;
        if (point_cloud_.data.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("keep_away_from_obstacles"), "点云为空");
            return Status::FAILED;
        }
        RCLCPP_INFO(rclcpp::get_logger("keep_away_from_obstacles"), "开始执行脱困任务");
        return Status::SUCCEEDED;
    }

    Status KeepAwayFromObstacles::onCycleUpdate()
    {
        // 转为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(point_cloud_, *cloud);
        // 找出距离最近的点
        double min_distance = std::numeric_limits<double>::max();
        size_t min_index = 0;
        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            // 过滤掉距离机器人中心点超过2m的点，因为这些点不会对机器人造成威胁
            if (abs(cloud->points[i].x) > 2.0f || abs(cloud->points[i].y) > 2.0f)
            {
                continue;
            }
            double distance = sqrt(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y);
            if (distance < min_distance)
            {
                min_distance = distance;
                min_index = i;
            }
        }
        if (min_distance > robot_radius_ + distance_)
        {
            if (is_first_cycle_)
            {
                stopRobot();
                //RCLCPP_INFO(rclcpp::get_logger("keep_away_from_obstacles"), "机器人没有被卡住，min_distance: %f, robot_radius: %f", min_distance, robot_radius_);
                is_first_cycle_ = false;
                return Status::SUCCEEDED;
                // return Status::FAILED;
            }
            else
            {
                stopRobot();
                RCLCPP_INFO(rclcpp::get_logger("keep_away_from_obstacles"), "机器人脱困，min_distance: %f, robot_radius: %f, distance: %f", min_distance, robot_radius_, distance_);
                return Status::SUCCEEDED;
            }
        }
        else
        {
            is_first_cycle_ = false;
            RCLCPP_INFO(rclcpp::get_logger("keep_away_from_obstacles"), "机器人被卡住，min_distance: %f, robot_radius: %f", min_distance, robot_radius_);
        }
        float min_index_x = cloud->points[min_index].x;
        float min_index_y = cloud->points[min_index].y;
        //RCLCPP_INFO(rclcpp::get_logger("keep_away_from_obstacles"), "min_distance: %f, x: %f, y: %f", min_distance, min_index_x, min_index_y);
        geometry_msgs::msg::Twist cmd_vel_;
        // 向远离障碍物方向运动
        float head_index_x = -min_index_x;
        float head_index_y = -min_index_y;
        float length = sqrt(head_index_x * head_index_x + head_index_y * head_index_y);

        cmd_vel_.linear.x = speed_ * head_index_x / length;
        cmd_vel_.linear.y = speed_ * head_index_y / length;

        cmd_vel_.angular.z = 0;
        vel_pub_->publish(cmd_vel_);
        // return Status::RUNNING;
        return Status::SUCCEEDED;
    }

} // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::KeepAwayFromObstacles, nav2_core::Behavior)
