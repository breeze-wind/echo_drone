# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

yaml_path = os.path.join(
        get_package_share_directory('robot_bring_up'),
        'config',
        'robot_sim.yaml'
    )

params_file = LaunchConfiguration(
        "params_file",
        default=yaml_path,
    )

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=yaml_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    node_01 = Node(
        package="robot_behaviors",
        executable="behavior_server",
        output="screen",
        name="behavior_server",
        parameters=[params_file]
        
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [node_01, declare_params_file_cmd]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description