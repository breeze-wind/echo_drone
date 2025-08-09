#ifndef KEEP_AWAY_FROM_OBSTACLES_HPP_
#define KEEP_AWAY_FROM_OBSTACLES_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "robot_behaviors/timed_behavior.hpp"
#include "robot_msgs/action/keep_away_from_obstacles.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace nav2_behaviors
{
    class KeepAwayFromObstacles : public TimedBehavior<robot_msgs::action::KeepAwayFromObstacles>
    {
    public:
        KeepAwayFromObstacles();
        ~KeepAwayFromObstacles();
        // 收到请求后立刻执行，即收到任务的初始化
        Status onRun(const std::shared_ptr<const robot_msgs::action::KeepAwayFromObstacles::Goal> command) override;

        // 每个周期执行一次，直到任务完成
        Status onCycleUpdate() override;

        // 用于获取参数、定义发布者接受者等
        void onConfigure() override;

        void point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    protected:
        robot_msgs::action::KeepAwayFromObstacles::Feedback::SharedPtr feedback_;
        std::string point_topic_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;
        sensor_msgs::msg::PointCloud2 point_cloud_;
        double speed_;
        double distance_;
        double robot_radius_;
        bool is_first_cycle_;
    };

} // namespace nav2_behaviors

#endif // NAV2_BEHAVIORS__PLUGINS__WAIT_HPP_