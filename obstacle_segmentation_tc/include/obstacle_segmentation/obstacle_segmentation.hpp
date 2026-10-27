//
// Created by elsa on 25-2-18.
//

#ifndef OBSTACLE_SEGMENTATION_NODE_HPP
#define OBSTACLE_SEGMENTATION_NODE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

class ObstacleSegmentationNode : public rclcpp::Node
{
public:
    explicit ObstacleSegmentationNode(std::string name, const rclcpp::NodeOptions &options);

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /// 接收从point-lio节点传来的当前位姿
    void CurrentPoseCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

    void ClearStateCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void ObstacleHeightCallback(const std_msgs::msg::Float64::SharedPtr msg);

    double odom_array[7];

private:
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;

    bool if_need_clear;
    double current_z_;
    double obstacle_height_;

    int pcl_cnt;

    sensor_msgs::msg::PointCloud2::SharedPtr output_cloud;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr segement_cloud;

    /// 接收当前位姿
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr obstacle_height_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_cloud_pub_;
};

#endif //OBSTACLE_SEGMENTATION_NODE_HPP
