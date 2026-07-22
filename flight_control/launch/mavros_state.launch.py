from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        respawn=False,
        parameters=[
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'target_system_id': ParameterValue(
                    LaunchConfiguration('tgt_system'),
                    value_type=int,
                ),
                'target_component_id': ParameterValue(
                    LaunchConfiguration('tgt_component'),
                    value_type=int,
                ),
                'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:230400'),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        mavros_node,
    ])
