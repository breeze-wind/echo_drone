//
// Created by elsa on 25-2-18.
//

#ifndef OBSTACLE_SEGMENTATION_NODE_HPP
#define OBSTACLE_SEGMENTATION_NODE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>      //半径滤波器头文件

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ObstacleSegmentationNode : public rclcpp::Node
{
public:
    explicit ObstacleSegmentationNode(std::string name, const rclcpp::NodeOptions &options);

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstPtr msg);
private:
    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_x_;
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter_y_;
    //pcl::PassThrough<pcl::PointXYZ> pass_through_filter_z_;
    pcl::VoxelGrid<pcl::PointXYZ> voxfilter;
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    float leaf_size_;          // 体素滤波器的体素大小
    int point_num_for_normal_; // 用于计算法向量的点数
    float angle_threshold_;    // 法向量与地面的夹角阈值
    float obstacle_x_min_;     // 障碍物点云范围(livox坐标系)
    float obstacle_x_max_;
    float obstacle_y_min_;
    float obstacle_y_max_;
    float obstacle_z_min_;
    float obstacle_z_max_;
    float obstacle_range_min_; // 障碍物点云范围(livox坐标系)
    float obstacle_range_max_;
    float body_min_x_; // 车体范围x
    float body_max_x_;
    float body_min_y_; // 车体范围y
    float body_max_y_;
    bool use_downsample_;
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    int cout;
    std::string base_frame_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map;

    std::unique_ptr<tf2_ros::Buffer> tfbuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr had_been_deleted_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_cloud_1_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_cloud_2_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_cloud_3_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_cloud_4_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_cloud_5_pub_;

};

#endif //OBSTACLE_SEGMENTATION_NODE_HPP
