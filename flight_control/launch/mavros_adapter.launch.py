"""Launch only the MAVROS compatibility adapter.

Use this entry when MAVROS is already running separately or when you want a
ROS-only adapter dry-run.  The default YAML keeps `dry_run: true`, so service
calls are logged instead of sent unless the caller overrides the parameter.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Keep the adapter config replaceable from the command line so bench tests
    # can switch between dry-run and real FCU settings without editing files.
    config_file = LaunchConfiguration('config_file')

    adapter_node = Node(
        package='flight_control',
        executable='mavros_adapter_node',
        name='mavros_adapter',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('flight_control'),
                'config',
                'mavros_adapter.yaml',
            ]),
            description='MAVROS adapter parameter file.',
        ),
        adapter_node,
    ])
