#include "obstacle_segmentation/obstacle_segmentation.hpp"

ObstacleSegmentationNode::ObstacleSegmentationNode(std::string name, const rclcpp::NodeOptions& options)
    : Node(name, options)
{
    RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
    // 声明参数
    this->declare_parameter("input_cloud_topic", "input_cloud");
    this->declare_parameter("output_cloud_topic", "output_cloud");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("leaf_size", 0.1);
    this->declare_parameter("point_num_for_normal", 50);
    this->declare_parameter("angle_threshold", 0.1);
    this->declare_parameter("obstacle_x_min", -10.0);
    this->declare_parameter("obstacle_x_max", 10.0);
    this->declare_parameter("obstacle_y_min", -10.0);
    this->declare_parameter("obstacle_y_max", 10.0);
    this->declare_parameter("obstacle_z_min", 0.0);
    this->declare_parameter("obstacle_z_max", 2.0);
    this->declare_parameter("obstacle_range_min", 0.5);
    this->declare_parameter("obstacle_range_max", 2.0);
    this->declare_parameter("body_min_x", -0.3);
    this->declare_parameter("body_max_x", 0.3);
    this->declare_parameter("body_min_y", -0.2);
    this->declare_parameter("body_max_y", 0.2);
    this->declare_parameter("use_downsample", true);

    RCLCPP_INFO(this->get_logger(), "点云分割节点初始化");
    this->get_parameter("input_cloud_topic", input_cloud_topic_);
    this->get_parameter("output_cloud_topic", output_cloud_topic_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("leaf_size", leaf_size_);
    this->get_parameter("point_num_for_normal", point_num_for_normal_);
    this->get_parameter("angle_threshold", angle_threshold_);
    this->get_parameter("obstacle_x_min", obstacle_x_min_);
    this->get_parameter("obstacle_x_max", obstacle_x_max_);
    this->get_parameter("obstacle_y_min", obstacle_y_min_);
    this->get_parameter("obstacle_y_max", obstacle_y_max_);
    this->get_parameter("obstacle_z_min", obstacle_z_min_);
    this->get_parameter("obstacle_z_max", obstacle_z_max_);
    this->get_parameter("obstacle_range_min", obstacle_range_min_);
    this->get_parameter("obstacle_range_max", obstacle_range_max_);
    this->get_parameter("use_downsample", use_downsample_);

    // 设置滤波器的体素大小
    pass_through_filter_x_.setFilterFieldName("x");
    pass_through_filter_x_.setFilterLimits(obstacle_x_min_, obstacle_x_max_);
    pass_through_filter_x_.setFilterLimitsNegative(false);
    pass_through_filter_y_.setFilterFieldName("y");
    pass_through_filter_y_.setFilterLimits(obstacle_y_min_, obstacle_y_max_);
    pass_through_filter_y_.setFilterLimitsNegative(false);
    pass_through_filter_z_.setFilterFieldName("z");
    pass_through_filter_z_.setFilterLimits(obstacle_z_min_, obstacle_z_max_);
    pass_through_filter_z_.setFilterLimitsNegative(false);
    voxfilter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

    current_z_ = 0.0;

    tfbuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_);
    // 初始化pub和sub
    output_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
    input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, 10, std::bind(&ObstacleSegmentationNode::cloudCallback, this, std::placeholders::_1));
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/robot/current_pose",
            10, std::bind(&ObstacleSegmentationNode::CurrentPoseCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "点云分割节点初始化完成");
}

void ObstacleSegmentationNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "障碍物点云数据回调");
    if (msg->data.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "接收到的点云数据为空.");
        return;
    }

    // 将点云转换为pcl格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    // 直通滤波
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector3f translation(odom_array[0], odom_array[1], odom_array[2]);
    Eigen::Quaternionf rotation(odom_array[3], odom_array[4], odom_array[5], odom_array[6]);
    transform.translation() = translation;
    transform.linear() = rotation.toRotationMatrix();
    //std::cout << odom_array[0] << std::endl;
    pcl::transformPointCloud(*cloud, *cloud, transform);
	/*
    pass_through_filter_x_.setInputCloud(cloud);
    pass_through_filter_x_.filter(*cloud);
    pass_through_filter_y_.setInputCloud(cloud);
    pass_through_filter_y_.filter(*cloud);
	*/

    // 创建体素滤波器主要作用是对点云进行降采样，可以在保证点云原有几何结构基本不变的前提下减少点的数量
/*
    if (use_downsample_)
    {
        voxfilter.setInputCloud(cloud);
        voxfilter.filter(*cloud);
    }
*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr segement_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (long i = 0; i < cloud->points.size(); i++)
    {
        if (cloud->points[i].z < 0.1)
        {
            continue;
        }
        if(cloud->points[i].z - current_z_ < 0.1)
        {
            if (cloud->points[i].z - current_z_ > -0.2)
            {
                segement_cloud->points.push_back(cloud->points[i]);
            }
        }
    }
    for(auto& point : segement_cloud->points){
        point.z = 0.0;
}
    segement_cloud->width = segement_cloud->points.size();
    segement_cloud->height = 1;
    segement_cloud->is_dense = true;
    sensor_msgs::msg::PointCloud2::SharedPtr output_cloud(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*segement_cloud, *output_cloud);
    output_cloud->header.frame_id = "map"; // msg->header.frame_id;
    output_cloud->header.stamp = msg->header.stamp;
    output_cloud_pub_->publish(*output_cloud);
    // RCLCPP_INFO(this->get_logger(), "障碍物点云数据正在发布");
}

void ObstacleSegmentationNode::CurrentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_z_ = msg->pose.position.z;
    odom_array[0] = msg->pose.position.x;
    odom_array[1] = msg->pose.position.y;
    odom_array[2] = msg->pose.position.z;
    odom_array[3] = msg->pose.orientation.w;
    odom_array[4] = msg->pose.orientation.x;
    odom_array[5] = msg->pose.orientation.y;
    odom_array[6] = msg->pose.orientation.z;
}