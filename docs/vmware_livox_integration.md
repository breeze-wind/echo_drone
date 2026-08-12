# VMware Ubuntu 对接 Livox MID360

## 目的与边界

本流程将 Ubuntu VMware 虚拟机作为 MID360 的直接接入端，替代 WSL2 mirrored
networking。它解决的是 Windows 已稳定收到 UDP、但 WSL 内的 Livox 点云仍出现
数秒断档的问题。Point-LIO 的 `lose lidar` 和 IMU/LiDAR 时间错误应在点云链路
稳定后再判断；本流程不通过时间补偿掩盖 UDP 断流。

数据路径：

```text
MID360 -- physical Ethernet -- VMware bridged adapter -- Ubuntu guest
                                             |-- Livox driver
                                             |-- Point-LIO / RViz
```

虚拟机需要安装与本仓库兼容的 Ubuntu 20.04 + ROS 2 Foxy 环境，并把本仓库克隆在
虚拟机 Linux 文件系统中（例如 `~/echo_drone`），不要从 `/mnt/hgfs` 运行构建或
传感器程序。

## VMware 网络设置

1. 在 VMware 的虚拟机设置中，把雷达对应的物理网卡设为 **Bridged**，并在
   `Configure Adapters` 中只勾选该物理以太网卡；不要选 `VMnet8 (NAT)`。
2. 禁用 VMware 的 `Replicate physical network connection state`，除非你的网卡
   会频繁断开/重连；它可能在宿主网络切换时重置 guest 链路。
3. 在 Ubuntu guest 的桥接网卡设置静态 IPv4，与 MID360 同网段。例如雷达为
   `192.168.1.154/24` 时，guest 可使用 `192.168.1.5/24`。该地址必须未被
   Windows 或其他设备占用。
4. 雷达网段通常不需要默认网关。保留另一张 NAT/Wi-Fi 网卡用于联网即可；不要让
   雷达网卡抢占默认路由。

在 Ubuntu 中确认真实雷达网卡名称和地址：

```bash
ip -br addr
ip route
ping -c 3 192.168.1.154
```

`ping` 成功只证明基本 IP 可达；仍必须用下文的点云频率验证 UDP 流。

## 仓库和本地配置

```bash
git clone git@github.com:breeze-wind/echo_drone.git ~/echo_drone
cd ~/echo_drone
cp .echo_drone.env.example .echo_drone.env
```

在 `.echo_drone.env` 设置虚拟机雷达网卡 IP 和雷达 IP：

```bash
LIVOX_HOST_IP=192.168.1.5
LIVOX_LIDAR_IP=192.168.1.154
```

这些地址仅存于忽略的 `.echo_drone.env`。`tools/run_livox_vmware.sh` 会生成
`${XDG_RUNTIME_DIR:-/tmp}/echo_drone_livox/MID360s_vmware_config.json`，不会修改或
提交原有的 `MID360s_config.json`。MID360 将点云与 IMU 分别发往 guest 的 UDP
`56301`、`56401`；控制端口也按 Livox 默认端口写入该生成配置。

首次使用或拉取带 C++ 改动的版本后构建：

```bash
./run_echo_drone.sh build
```

## 启动和验收顺序

先只启动驱动：

```bash
./tools/run_livox_vmware.sh
```

另开终端：

```bash
cd ~/echo_drone
source tools/echo_drone_env.bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

连续观察至少 3–5 分钟。MID360 配置为 10 Hz 时，`/livox/lidar` 应接近 10 Hz，
且 `max` 通常约为 `0.08–0.15s`；任意数秒级 `max` 都是不合格的 UDP 断流，即使
平均频率后来回升。确认稳定后，关闭该驱动，再运行：

```bash
./run_echo_drone.sh sensing-nav
```

若只验证 LIO，使用：

```bash
./run_echo_drone.sh sensing
```

此仓库的标准 `sensing` 和 `sensing-nav` 会自行启动 Livox 驱动；不要与
`tools/run_livox_vmware.sh` 并行运行，否则会争用 UDP 端口。全栈入口仍使用仓库
默认的 `MID360s_config.json`，因此先将该 JSON 的 `host_ip` 改为 VMware guest 的
雷达网卡地址（只在虚拟机工作副本中改），再运行标准入口。

## 失败定位

- `bind failed`：guest IP 不在雷达网段、地址被占用，或 VMware 未桥接到正确的物理
  网卡。
- `Init lds lidar fail`：先检查 IP、网线、供电和 Ubuntu 是否能 ping 雷达；不应先
  修改 Point-LIO。
- `/livox/lidar` 有 `max` 达数秒：问题仍在 Ethernet/VMware/guest 接收路径，不是
  Point-LIO 的 `time_lag_imu_to_lidar`。
- 驱动稳定但 Point-LIO 报 `lidar_end_time`：再采集 `/livox/lidar` 和 `/livox/imu`
  的 `header.stamp`，判断固定偏移或 IMU 时间问题。
