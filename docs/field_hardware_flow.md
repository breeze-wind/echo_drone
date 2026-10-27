# 现场实机发布流程

## 目标

这个流程只要求先打通两件事：

- MAVROS 能真实连到飞控并发布 `/mavros/state`。
- Livox 和 Point-LIO 能发布 `/livox/lidar`、`/livox/imu`、`/Odometry` 和点云话题。

本流程不解锁、不切 OFFBOARD、不发布真实飞控控制 setpoint。

## 离线检查注意

没有接入雷达和飞控时，流程只能检查软件入口，不能判定硬件正常。

离线检查时看到的关键点：

- 没有 `/dev/ttyACM*`、`/dev/ttyUSB*` 或 `/dev/px4_fcu` 时，MAVROS 会报 `serial:open: No such file or directory`。
- `sensing`/`livox` 入口当前使用 `MID360s_config.json`，其仓库默认值是主机 IP `192.168.1.50`、雷达 IP `192.168.1.154`；现场必须按实际网卡修改或使用 VMware 生成配置。
- 如果使用 WSL NAT 且宿主机/虚拟机地址不在雷达网段，Livox 不能按配置的主机 IP 绑定；优先使用桥接或 mirrored 网络。
- 2026-10-27 的当前机器是 Ubuntu 20.04/aarch64，本次检查未发现串口设备，未做硬件验收。

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
third_party/livox_ros_driver2/config/MID360s_config.json
```

需要重点改这些字段。

```text
Mid360s.host_net_info[0].host_ip
Mid360s.lidar_net_info.cmd_data_port
Mid360s.lidar_net_info.push_msg_port
Mid360s.lidar_net_info.point_data_port
Mid360s.lidar_net_info.imu_data_port
lidar_configs[0].ip
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
cd /path/to/echo_drone
./run_echo_drone.sh field-precheck
```

第二步，确认系统能看到飞控串口。

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/px4_fcu
groups
```

如果 `groups` 没有 `dialout`，但 `/etc/group` 已经有当前用户对应的 `dialout` 组，重新登录或执行下面命令。

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

把 `third_party/livox_ros_driver2/config/MID360s_config.json` 里的 host IP 改成 Linux 实际接雷达的网卡 IP。

如果使用 WSL，优先使用 mirrored 网络或让 Windows/WSL 能直接绑定到雷达同网段地址；VMware 使用桥接网卡。

第五步，启动感知最小链路。

```bash
./run_echo_drone.sh sensing
```

另开终端采样。

```bash
source /opt/ros/foxy/setup.bash
source /path/to/echo_drone/install/setup.bash
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
- `/tf_static` 里能看到 `map -> odom`、`livox -> mavlink_body`、`livox -> camera_link`。
- Point-LIO 的动态 TF 是 `odom -> livox`；当前配置下不存在 `livox_raw -> livox`。

第六步，MAVROS 和 Point-LIO 都正常后再接 adapter。

```bash
./run_echo_drone.sh mavros-real
```

这一步会启动 MAVROS adapter，但仍按配置控制是否自动切模式或解锁。

## 不要直接跳过的事项

不要在 `/mavros/state connected` 还不是 `true` 时跑真实圆周控制。

不要在 `/Odometry` 没有稳定发布时把外部视觉位姿送给飞控。

不要在主机地址与 Livox 配置不在同一网段时排查 Point-LIO。

先让 `/livox/lidar` 和 `/livox/imu` 出来，再看 Point-LIO。
