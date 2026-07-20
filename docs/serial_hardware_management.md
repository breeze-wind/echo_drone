# 串口统一管理说明

## 当前目标

串口统一管理不把所有串口协议揉进一个节点，而是统一设备配置、启动入口、dry-run 策略和状态观测。

飞控串口由 MAVROS 独占，`robot_serial_manager` 只检查设备是否存在，不打开飞控串口，避免和 MAVROS 抢设备。

## 当前硬件角色

| 逻辑设备 | 默认端口 | owner | 当前处理 |
|---|---|---|---|
| `px4_fcu` | `/dev/px4_fcu` | MAVROS | `dry_run=false` 且 `use_mavros=true` 时由 MAVROS 打开 |
| `stm32_servo` | `/dev/stm32_servo` | `servo_node` | 舵机驱动打开，支持 dry-run 和状态输出 |
| `openmv` | `/dev/openmv_serial` | `openmv_serial_driver` | 暂未默认实现，保留配置位 |

## 新增入口

```bash
ros2 launch robot_bring_up hardware.launch.py
```

默认参数：

```text
dry_run:=true
use_serial_manager:=true
use_servo:=true
use_mavros:=true
use_legacy_mavlink:=false
use_openmv:=false
```

默认 `dry_run=true` 时，MAVROS 不会启动，舵机不会写真实串口。

试验机上准备连接飞控和舵机时使用：

```bash
ros2 launch robot_bring_up hardware.launch.py dry_run:=false use_mavros:=true use_servo:=true
```

旧 `pymavlink` 直连只作为 fallback：

```bash
ros2 launch robot_bring_up hardware.launch.py dry_run:=false use_mavros:=false use_legacy_mavlink:=true
```

## 新增话题和服务

| 接口 | 类型 | 说明 |
|---|---|---|
| `/hardware/serial_status` | `std_msgs/msg/String` JSON | 串口设备扫描状态 |
| `/hardware/rescan_serials` | `std_srvs/srv/Trigger` | 手动重新扫描串口 |
| `/servo/command` | `std_msgs/msg/Int32` | 设置舵机命令 |
| `/servo/drop` | `std_srvs/srv/Trigger` | 发送投放命令 |
| `/servo/status` | `std_msgs/msg/String` JSON | 舵机驱动状态 |
| `/servo_node/set_parameters` | `rcl_interfaces/srv/SetParameters` | 兼容旧 `behavior_control` 的 `/servo/servo` 参数调用 |

## dry-run 含义

`dry-run` 是模拟运行模式。

在 `dry_run=true` 下，节点会加载参数、发布状态、接收命令、打印将要发送的串口 payload，但不会写真实串口。

这个模式用于非试验机环境调 launch、调接口、跑决策逻辑，避免误动飞控或舵机。

## 已验证

```bash
python3 -m py_compile servo_node/servo_node/servo_node.py \
  robot_serial_manager/robot_serial_manager/robot_serial_manager_node.py \
  robot_bring_up/launch/hardware.launch.py
```

```bash
colcon build
```

```bash
ros2 launch robot_bring_up hardware.launch.py --show-args
ros2 launch robot_bring_up servo.launch.py --show-args
ros2 launch robot_bring_up serial.launch.py --show-args
```

非沙箱环境下短启动通过：

```bash
timeout 6 ros2 launch robot_bring_up hardware.launch.py \
  dry_run:=true use_mavros:=true use_servo:=true use_serial_manager:=true
```
