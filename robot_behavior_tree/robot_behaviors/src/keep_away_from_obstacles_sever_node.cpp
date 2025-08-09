// 接收代价地图和footprint，将机器人朝着代价地图中的最低代价区域移动，直到机器人离障碍物足够远。

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// 创建一个ActionServer类
class KeepAwayFromObstacles : public rclcpp::Node
{
public:
    explicit KeepAwayFromObstacles(std::string name) : Node(name)
    {
        RCLCPP_INFO(node_->get_logger(), "节点已启动：%s.", name.c_str());
        this->declare_parameter("point_topic", "/cloud_obstacle");
        this->get_parameter("point_topic", point_topic_);
        declare_parameter("robot_radius", 0.5);
        get_parameter("robot_radius", robot_radius_);
        point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(point_topic_, 10, std::bind(&KeepAwayFromObstacles::point_callback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    std::string point_topic_;
    double robot_radius_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    void point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::cout << "收到点云" << std::endl;
        RCLCPP_INFO(rclcpp::get_logger("keep_away_from_obstacles"), "周期更新");
        // 转为PCL点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
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
        if (min_distance > robot_radius_ + 0.05)
        {
            geometry_msgs::msg::Twist cmd_vel_;
            cmd_vel_.linear.x = 0;
            cmd_vel_.linear.y = 0;
            cmd_vel_.angular.z = 0;
            cmd_vel_pub_->publish(cmd_vel_);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("keep_away_from_obstacles"), "机器人被卡住，min_distance: %f, robot_radius: %f", min_distance, robot_radius_);
        }
        float min_index_x = cloud->points[min_index].x;
        float min_index_y = cloud->points[min_index].y;
        RCLCPP_INFO(rclcpp::get_logger("keep_away_from_obstacles"), "min_distance: %f, x: %f, y: %f", min_distance, min_index_x, min_index_y);
        geometry_msgs::msg::Twist cmd_vel_;
        // 向远离障碍物方向运动
        float head_index_x = -min_index_x;
        float head_index_y = -min_index_y;
        float length = sqrt(head_index_x * head_index_x + head_index_y * head_index_y);

        cmd_vel_.linear.x = 1.0 * head_index_x / length;
        cmd_vel_.linear.y = 1.0 * head_index_y / length;
        cmd_vel_.angular.z = 0;
        cmd_vel_pub_->publish(cmd_vel_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<KeepAwayFromObstacles>("keep_away_from_obstacles");
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}