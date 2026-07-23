# 串口统一管理说明

## 当前目标

串口统一管理不把所有串口协议揉进一个节点，而是统一设备配置、启动入口、dry-run 策略和状态观测。

飞控串口由 MAVROS 独占，`robot_serial_manager` 只检查设备是否存在，不打开飞控串口，避免和 MAVROS 抢设备。

`flight_control/mavros_adapter_node` 只桥接旧 `/robot/*` 话题与 MAVROS topic/service，本身不打开串口。

## `robot_serial_manager` 当前完成度

这个包已经完成“统一清单、统一状态发布、统一 dry-run 语义”这部分，不是串口数据收发总线。

已完成：

- 从 `robot_bring_up/config/hardware/ports.yaml` 读取逻辑设备列表。
- 周期扫描系统串口，并发布 `/hardware/serial_status`。
- 提供 `/hardware/rescan_serials` 服务，支持热插拔后手动重扫。
- 支持 `dry_run=true`，非试验机或设备未接入时不会把缺失设备当成硬错误。
- 记录每个逻辑设备的 `owner`，让飞控、舵机、OpenMV 的归属关系在一个地方可查。
- 支持按配置端口优先匹配，配置端口不存在时可按 `manufacturer` 做 fallback 匹配。

未完成或刻意不做：

- 不打开任何业务串口，不读写 MAVLink、舵机协议或 OpenMV 数据。
- 不替 MAVROS、`servo_node` 或后续 OpenMV 驱动转发数据。
- 不创建 udev 规则，只检查 `/dev/px4_fcu`、`/dev/stm32_servo` 等别名是否存在。
- 不做串口抢占保护，只通过状态和 owner 提示谁应该使用哪个设备。

因此当前效果是：启动硬件层时能快速看出飞控、舵机、OpenMV 串口是否存在，缺失是 dry-run 缺失还是实机必需设备缺失，但真正收发仍由各业务节点负责。

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

默认 `dry_run=true` 时，MAVROS 不会启动，`mavros_adapter` 以 ROS-only 模式运行，舵机不会写真实串口。

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
| `/flight_control/status` | `std_msgs/msg/String` JSON | MAVROS 适配层状态 |
| `/servo_node/set_parameters` | `rcl_interfaces/srv/SetParameters` | 兼容旧 `behavior_control` 的 `/servo/servo` 参数调用 |

## `/hardware/serial_status` 字段

状态消息是 `std_msgs/msg/String`，内容为 JSON。

顶层字段：

| 字段 | 含义 |
|---|---|
| `stamp` | 扫描时间戳，来自系统时间 |
| `dry_run` | 当前是否处于 dry-run |
| `devices` | 每个逻辑设备的状态数组 |

单个设备字段：

| 字段 | 含义 |
|---|---|
| `name` | 逻辑设备名，例如 `px4_fcu` |
| `owner` | 预期占用该设备的节点，例如 `mavros` 或 `servo_node` |
| `required` | 真实硬件模式下是否必须存在 |
| `configured_port` | 配置中的端口路径 |
| `configured_exists` | 配置端口路径是否存在 |
| `resolved_port` | 最终解析到的端口；为空表示未找到 |
| `fallback_used` | 是否使用 manufacturer fallback 匹配 |
| `baudrate` | 配置波特率，仅用于说明，不会打开串口 |
| `manufacturer` | 配置的 USB manufacturer fallback 条件 |
| `state` | `available`、`dry_run_missing`、`missing_required` 或 `missing_optional` |
| `port_info` | pyserial 枚举出的设备描述、manufacturer 和 hwid |

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
