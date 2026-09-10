import os

from setuptools import find_packages, setup

package_name = 'sls_circle_controller'


def collect_data_files(source_dir):
    """递归安装 PX4/Gazebo 资源目录，确保 launch 后能通过 share 路径找到模型。"""
    data_files = []
    for root, _, files in os.walk(source_dir):
        if not files:
            continue
        data_files.append((
            os.path.join('share', package_name, root),
            [os.path.join(root, file_name) for file_name in files],
        ))
    return data_files

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/sls_circle.yaml',
            'config/mavros_px4_sitl.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/sls_circle.launch.py',
            'launch/sls_circle_sim.launch.py',
            'launch/sls_circle_gazebo.launch.py',
            'launch/px4_sitl_sls.launch.py',
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/sls_circle_force.world',
        ]),
    ] + collect_data_files('px4_sitl'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sfx',
    maintainer_email='sfx@example.com',
    description='MAVROS based SLS circle controller for Echo Drone.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sls_circle_controller_node = '
            'sls_circle_controller.sls_circle_controller_node:main',
            'fake_mavros_sim_node = '
            'sls_circle_controller.fake_mavros_sim_node:main',
            'gazebo_mavros_bridge_node = '
            'sls_circle_controller.gazebo_mavros_bridge_node:main',
            'gazebo_stability_validator = '
            'sls_circle_controller.gazebo_stability_validator:main',
            'wind_control_gui = '
            'sls_circle_controller.wind_control_gui:main',
        ],
    },
)
