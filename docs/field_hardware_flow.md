# 现场实机发布流程

## 目标

这个流程只要求先打通两件事：

- MAVROS 能真实连到飞控并发布 `/mavros/state`。
- Livox 和 Point-LIO 能发布 `/livox/lidar`、`/livox/imu`、`/Odometry` 和点云话题。

本流程不解锁、不切 OFFBOARD、不发布真实飞控控制 setpoint。

## 当前离线结论

当前这台环境没有接入雷达和飞控时，流程可以检查软件入口，但不能判定硬件正常。

离线检查时看到的关键点：

- WSL 里没有 `/dev/ttyACM*`、`/dev/ttyUSB*` 或 `/dev/px4_fcu` 时，MAVROS 会报 `serial:open: No such file or directory`。
- Livox 配置文件当前写的是主机 IP `192.168.1.5`、雷达 IP `192.168.1.12`。
- 当前 WSL NAT 地址如果是 `172.*`，则 Livox 不能直接按 `192.168.1.5` 绑定。

## 到现场要改哪里

飞控串口默认由 `run_echo_drone.sh` 自动探测。

```bash
./run_echo_drone.sh mavros-check
```

如果现场设备号固定，可以把 `FCU_URL=/dev/ttyACM0:230400` 写入工作区根目录 `.echo_drone.env`。

飞控串口底层配置在这里。

```text
robot_bring_up/config/hardware/ports.yaml
flight_control/launch/mavros_state.launch.py
robot_bring_up/config/hardware/mavros.yaml
```

Livox 网络配置在这里。

```text
third_party/livox_ros_driver2/config/MID360_config.json
```

需要重点改这些字段。

```text
MID360.host_net_info.cmd_data_ip
MID360.host_net_info.push_msg_ip
MID360.host_net_info.point_data_ip
MID360.host_net_info.imu_data_ip
MID360.lidar_configs[0].ip
```

Point-LIO 输入输出配置在这里。

```text
robot_bring_up/config/drone.yaml
```

重点字段。

```text
laser_mapping.ros__parameters.common.lid_topic
laser_mapping.ros__parameters.common.imu_topic
laser_mapping.ros__parameters.common.map_frame
laser_mapping.ros__parameters.common.odom_frame
laser_mapping.ros__parameters.publish.scan_publish_en
laser_mapping.ros__parameters.publish.scan_bodyframe_pub_en
```

感知静态 TF 在这里集中管理。

```text
robot_bring_up/launch/sensing.launch.py
```

## 现场启动顺序

第一步，检查软件环境和配置入口。

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh field-precheck
```

第二步，确认飞控 USB 已进入 WSL。

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/px4_fcu
groups
```

如果 `groups` 没有 `dialout`，但 `/etc/group` 已经有 `dialout:x:20:sfx`，重开 WSL 或执行下面命令。

```bash
newgrp dialout
```

第三步，只读检查 MAVROS。

```bash
./run_echo_drone.sh mavros-check
```

通过标准：

- 日志里不再出现 `serial:open: No such file or directory`。
- 日志里应出现 heartbeat 或连接相关信息。
- `/mavros/state` 能采到消息，且 `connected: true`。

第四步，确认 Livox 网络。

```bash
ip -br addr
ip route
```

把 `third_party/livox_ros_driver2/config/MID360_config.json` 里的 host IP 改成 Linux 实际接雷达的网卡 IP。

如果是 WSL，优先使用 mirrored 网络或让 Windows/WSL 能直接绑定到雷达同网段地址。

第五步，启动感知最小链路。

```bash
./run_echo_drone.sh sensing
```

另开终端采样。

```bash
source /opt/ros/foxy/setup.bash
source /home/sfx/echo_drone/install/setup.bash
ros2 topic list -t
ros2 topic echo /livox/imu sensor_msgs/msg/Imu
ros2 topic echo /Odometry nav_msgs/msg/Odometry
```

也可以直接跑限时检查。

```bash
CHECK_SECONDS=30 ./run_echo_drone.sh sensing-check
```

通过标准：

- `/livox/lidar` 有 `livox_ros_driver2/msg/CustomMsg`。
- `/livox/imu` 有 `sensor_msgs/msg/Imu`。
- Point-LIO 日志出现 `Node init finished`。
- `/Odometry` 有 `nav_msgs/msg/Odometry`。
- `/cloud_registered_body` 有 `sensor_msgs/msg/PointCloud2`。
- `/tf_static` 里能看到 `map -> odom`、`livox_raw -> livox`、`livox -> mavlink_body`。

第六步，MAVROS 和 Point-LIO 都正常后再接 adapter。

```bash
./run_echo_drone.sh mavros-real
```

这一步会启动 MAVROS adapter，但仍按配置控制是否自动切模式或解锁。

## 不要直接跳过的事项

不要在 `/mavros/state connected` 还不是 `true` 时跑真实圆周控制。

不要在 `/Odometry` 没有稳定发布时把外部视觉位姿送给飞控。

不要在 WSL NAT 地址还是 `172.*` 且 Livox 配置是 `192.168.1.5` 时排查 Point-LIO。

先让 `/livox/lidar` 和 `/livox/imu` 出来，再看 Point-LIO。
