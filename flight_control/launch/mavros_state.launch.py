"""Start a minimal MAVROS node for FCU state and topic inspection.

Foxy installations in this workspace did not reliably accept the upstream
MAVROS XML launch file because it still uses old launch substitution syntax.
This Python wrapper launches `mavros_node` directly with only the serial URL,
target IDs, and protocol arguments needed for PX4 heartbeat checks.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # This wrapper intentionally does not pass a large plugin config yet.  Keep
    # state-only bench checks simple until the exact plugin filter is verified
    # on the target Foxy/ARM image.
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
        # Default to the udev-managed FCU link, but allow /dev/ttyACM0 while
        # testing under WSL/USBIP where udev rules may not be active.
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyACM0:230400'),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        mavros_node,
    ])
