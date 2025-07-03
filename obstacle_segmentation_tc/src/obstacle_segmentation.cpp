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
    this->declare_parameter("cluster_tolerance", 0.1);
    this->declare_parameter("min_cluster_size", 10);
    this->declare_parameter("max_cluster_size", 25000);

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
    this->get_parameter("body_min_x", body_min_x_);
    this->get_parameter("body_max_x", body_max_x_);
    this->get_parameter("body_min_y", body_min_y_);
    this->get_parameter("body_max_y", body_max_y_);
    this->get_parameter("use_downsample", use_downsample_);
    this->get_parameter("cluster_tolerance", cluster_tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);

    // 设置滤波器的体素大小
    pass_through_filter_x_.setFilterFieldName("x");
    pass_through_filter_x_.setFilterLimits(obstacle_x_min_, obstacle_x_max_);
    pass_through_filter_x_.setFilterLimitsNegative(false);
    pass_through_filter_y_.setFilterFieldName("y");
    pass_through_filter_y_.setFilterLimits(obstacle_y_min_, obstacle_y_max_);
    pass_through_filter_y_.setFilterLimitsNegative(false);
    pass_through_filter_y_.setFilterFieldName("z");
    pass_through_filter_y_.setFilterLimits(obstacle_z_min_, obstacle_z_max_);
    pass_through_filter_y_.setFilterLimitsNegative(false);
    voxfilter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    ec.setClusterTolerance(cluster_tolerance_); // 聚类距离阈值
    ec.setMinClusterSize(min_cluster_size_); // 最小聚类点数
    ec.setMaxClusterSize(max_cluster_size_); // 最大聚类点数

    map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->header.frame_id = "map";
    map->header.stamp = this->get_clock()->now();
    map->info.resolution = 0.1;         // float32
    map->info.width      = 256;           // uint32
    map->info.height     = 128;           // uint32
    map->info.origin.position.x = 0.0;
    map->info.origin.position.y = 0.0;
    map->info.origin.position.z = 0.0;
    map->info.origin.orientation.x = 0.0;
    map->info.origin.orientation.y = 0.0;
    map->info.origin.orientation.z = 0.0;
    map->info.origin.orientation.w = 1.0;
    map->data.resize(map->info.width * map->info.height);

    tfbuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfbuffer_);
    // 初始化pub和sub
    output_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
    had_been_deleted_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/had_been_deleted_cloud", 10);
    // cluster_cloud_1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_cloud_1", 10);
    // cluster_cloud_2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_cloud_2", 10);
    // cluster_cloud_3_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_cloud_3", 10);
    // cluster_cloud_4_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_cloud_4", 10);
    // cluster_cloud_5_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_cloud_5", 10);
    input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic_, 10, std::bind(&ObstacleSegmentationNode::cloudCallback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&ObstacleSegmentationNode::mapCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "点云分割节点初始化完成");
}
void ObstacleSegmentationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::ConstPtr msg){
    map->header.frame_id = "map";
    map->header.stamp = msg->header.stamp;
    map->info.resolution = msg->info.resolution;
    map->info.width = msg->info.width;
    map->info.height = msg->info.height;
    map->info.map_load_time = msg->info.map_load_time;
    map->info.origin.position.x = msg->info.origin.position.x;
    map->info.origin.position.y = msg->info.origin.position.y;
    map->info.origin.position.z = msg->info.origin.position.z;
    map->info.origin.orientation.x = msg->info.origin.orientation.x;
    map->info.origin.orientation.y = msg->info.origin.orientation.y;
    map->info.origin.orientation.z = msg->info.origin.orientation.z;
    map->info.origin.orientation.w = msg->info.origin.orientation.w;
    map->data.resize(map->info.width * map->info.height);
    map->data = msg->data;
    RCLCPP_INFO(this->get_logger(), "map: width: %d, height: %d", map->info.width, map->info.height);
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
    pass_through_filter_x_.setInputCloud(cloud);
    pass_through_filter_x_.filter(*cloud);
    pass_through_filter_y_.setInputCloud(cloud);
    pass_through_filter_y_.filter(*cloud);
    //pass_through_filter_z_.setInputCloud(cloud);
    //pass_through_filter_z_.filter(*cloud);
    // 创建体素滤波器主要作用是对点云进行降采样，可以在保证点云原有几何结构基本不变的前提下减少点的数量
    if (use_downsample_)
    {
        voxfilter.setInputCloud(cloud);
        voxfilter.filter(*cloud);
    }
//    RCLCPP_INFO(this->get_logger(), "obstacle_segmentation: 滤波后点云数量： %lu", cloud->points.size());

    // 创建法向量估计对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    // 创建一个空的kdtree对象，并把它传递给法向量估计对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    // 输出数据集
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(point_num_for_normal_); // 使用最近的point_num_for_normal_个点计算法向量
    ne.compute(*cloud_normals); // 计算法向量

    pcl::PointCloud<pcl::PointXYZ>::Ptr segement_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (long i = 0; i < cloud->points.size(); i++)
    {
        float gradient = acos(
            sqrt(pow(cloud_normals->points[i].normal_x, 2) + pow(cloud_normals->points[i].normal_y, 2)) /
            sqrt(pow(cloud_normals->points[i].normal_x, 2) + pow(cloud_normals->points[i].normal_y, 2) +
                pow(cloud_normals->points[i].normal_z, 2)));
        // 如果法向量与地面的夹角小于角度阈值则认为是障碍物
        if (gradient < angle_threshold_)
        {
            segement_cloud->points.push_back(cloud->points[i]);
        }
    }
    // 再过滤一次离群点
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
    // 设置输入点云
    radiusoutlier.setInputCloud(segement_cloud);
    // 设置半径,在该范围内找临近点
    radiusoutlier.setRadiusSearch(0.15);
    // 设置查询点的邻域点集数，小于该阈值的删除
    radiusoutlier.setMinNeighborsInRadius(3);
    radiusoutlier.filter(*segement_cloud);
//    RCLCPP_INFO(this->get_logger(), "obstacle_segmentation: 点云分割剩余点云数： %lu", segement_cloud->points.size());

    segement_cloud->width = segement_cloud->points.size();
    segement_cloud->height = 1;
    segement_cloud->is_dense = true;
    sensor_msgs::msg::PointCloud2::SharedPtr output_cloud(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*segement_cloud, *output_cloud);
    output_cloud->header.frame_id = msg->header.frame_id; // msg->header.frame_id;
    output_cloud->header.stamp = msg->header.stamp;
    output_cloud_pub_->publish(*output_cloud);
    // RCLCPP_INFO(this->get_logger(), "障碍物点云数据正在发布");

//    //聚类分离
//
//    geometry_msgs::msg::TransformStamped origin_to_base;
//    geometry_msgs::msg::TransformStamped base_to_origin;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr had_deleted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    sensor_msgs::msg::PointCloud2::SharedPtr had_been_deleted_cloud(new sensor_msgs::msg::PointCloud2);
//
//    //将点云从当前雷达坐标系转到地图原点坐标系
//    try {
//        origin_to_base = tfbuffer_->lookupTransform("origin", msg->header.frame_id, rclcpp::Time(),
//                                           rclcpp::Duration::from_seconds(0.5));
//        tf2::doTransform(*output_cloud, *output_cloud, origin_to_base);
//    } catch (tf2::TransformException ex) {
//        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
//        return;
//    }
//    pcl::fromROSMsg(*output_cloud, *transformed_cloud); //转格式
//    int out=0,in=0;
//    //去除在地图障碍物中的点
//    for (int i = 0; i < transformed_cloud->points.size(); i++)
//    {
////        RCLCPP_INFO(this->get_logger(), "transformed_cloud: x: %f, y: %f", transformed_cloud->points[i].x, transformed_cloud->points[i].y);
////        RCLCPP_INFO(this->get_logger(), "map index: %d", int(transformed_cloud->points[i].x / map->info.resolution)
////            + int(transformed_cloud->points[i].y / map->info.resolution * map->info.width));
////        RCLCPP_INFO(this->get_logger(), "value: %d", map->data[int(transformed_cloud->points[i].x / map->info.resolution)
////            + int(transformed_cloud->points[i].y / map->info.resolution * map->info.width)]);
//
//        if (map->data[int(transformed_cloud->points[i].x / map->info.resolution)
//            + int(transformed_cloud->points[i].y / map->info.resolution * map->info.width)] < 80)
//        {
//          in++;
//            had_deleted_cloud->push_back(transformed_cloud->points[i]);
//            // RCLCPP_INFO(this->get_logger(), "有用");
//        }
//        else{
//          out++;
//          }
//    }
//    RCLCPP_INFO(this->get_logger(), "in:%d out:%d", in,out);
//
//
//    pcl::toROSMsg(*had_deleted_cloud, *had_been_deleted_cloud);
//    had_been_deleted_cloud->header.frame_id = msg->header.frame_id;
//    had_been_deleted_cloud->header.stamp = msg->header.stamp;
//    had_been_deleted_cloud_pub_->publish(*had_been_deleted_cloud);
//    // RCLCPP_INFO(this->get_logger(), "发布非先验地图中的点云");
//
//    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cluster (new pcl::search::KdTree<pcl::PointXYZ>);
//    ec.setInputCloud(had_deleted_cloud);
//    ec.setSearchMethod(tree_cluster);
//    ec.extract(cluster_indices); //获得聚类
//
//    // 发布聚类后的障碍物的点云
//    for (int i = 0; i < cluster_indices.size() && i < 5; i++)
//    {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//        for (int j = 0; j < cluster_indices[i].indices.size(); j++)
//        {
//            obstacle_cloud->points.push_back(had_deleted_cloud->points[cluster_indices[i].indices[j]]);
//        }
//
//        obstacle_cloud->width = obstacle_cloud->points.size();
//        obstacle_cloud->height = 1;
//        obstacle_cloud->is_dense = true;
//
//        sensor_msgs::msg::PointCloud2::SharedPtr obstacle_output_cloud(new sensor_msgs::msg::PointCloud2);
//        pcl::toROSMsg(*obstacle_cloud, *obstacle_output_cloud);
//        obstacle_output_cloud->header.frame_id = msg->header.frame_id;
//        obstacle_output_cloud->header.stamp = msg->header.stamp;
//
//        //debug
//        switch (i)
//        {
//        case 0:
//            cluster_cloud_1_pub_->publish(*obstacle_output_cloud);
//            break;
//        case 1:
//            cluster_cloud_2_pub_->publish(*obstacle_output_cloud);
//            break;
//        case 2:
//            cluster_cloud_3_pub_->publish(*obstacle_output_cloud);
//            break;
//        case 3:
//            cluster_cloud_4_pub_->publish(*obstacle_output_cloud);
//            break;
//        case 4:
//            cluster_cloud_5_pub_->publish(*obstacle_output_cloud);
//            break;
//        default:
//            break;
//        }
//        RCLCPP_INFO(this->get_logger(), "障碍物%d的点云数据正在发布", i);
//    }
}
