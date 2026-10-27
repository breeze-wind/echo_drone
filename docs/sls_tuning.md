# SLS 圆周控制器调参手册（ROS2 Foxy）

> 配套文档：`docs/sls_sim_spec.md`（外环权威规格、仿真/实机边界、验收清单）
> 本文所有命令按 **ROS2 Foxy**（`/opt/ros/foxy`）书写。节点名：`/sls_circle_controller`。

## 0. 三条硬约束（先读，否则会白调）

### 0.1 改参数必须重启节点 —— `ros2 param set` 在这里是「静默无效」的

C++ 节点**没有** `add_on_set_parameters_callback`：参数只在构造时 `read_parameters()` 读一次并缓存进成员变量。

因此下面这条命令会返回成功、`ros2 param get` 也会显示新值，**但控制律用的仍是启动时的值**：

```bash
ros2 param set /sls_circle_controller qsf_kv_x 60.0   # 看起来成功，实际不影响控制
```

- ROS1 原版（`drown`）有 `cfg/MrotorController.cfg` + `dynamic_reconfigure`，其 README 明确让人用
  `rqt_reconfigure` 实时拖增益；**移植到 Foxy 时丢掉了这个能力**。
- 目前唯一有效的改参方式：改 `config/sls_circle.yaml`，或用 launch 追加 `-p name:=value`，然后
  **重启节点**。
- 若希望恢复实时调参，需要给节点加 `add_on_set_parameters_callback` 并把新值同步到成员变量（尚未实现）。

### 0.2 launch 参数会覆盖 YAML，不同入口生效值不同

见 §2.2。**每次改完参数务必按 §2.3 确认生效值**，否则很可能调了半天没生效。

### 0.3 权限级参数不要随手动

`max_acc_xy` / `max_acc_z` / `max_total_acc` / `max_tilt_deg` / `max_thrust` / `hover_thrust`
直接决定可用控制权限与姿态上限。其中 `hover_thrust` 必须由台架标定得出
（节点内置 `controller_mode=thrust_calibration` 专门做这件事）。详见 §6.5。

## 1. 调参顺序（必须按层，跳层会白干）

```text
第 0 层 数据链健康度  ->  第 1 层 PD 兜底  ->  第 2 层 QSF 外环  ->  第 3 层 抗风扰
```

理由：

- QSF 的加速度/加加速度增益（`qsf_ka_*` / `qsf_kj_*`）对状态质量极其敏感。数据链不稳时调 QSF，
  只会把噪声放大成抖动。
- 抗风扰的残差估计依赖「命令 - 观测」的一致性。外环没调好时，残差里装的全是模型误差而不是风，
  这时调 `wind_estimate_filter_tau` 或 `wind_compensation_gain` 是在给模型误差编参数。

## 2. 参数来源与生效值

### 2.1 优先级（低 -> 高）

1. C++ `declare_parameter(...)` 的默认值
2. `config/sls_circle.yaml`（经 launch 的 `--params-file` 传入）
3. launch 声明并转发的 `-p name:=value`
4. 命令行追加的 `-p name:=value`（`./run_echo_drone.sh <mode> foo:=bar`）

### 2.2 各入口的实际覆盖情况

| 入口 | QSF 增益来源 | 限幅/倾角来源 | 说明 |
|---|---|---|---|
| `sls-circle-sim` | **YAML** | **YAML** | 不覆盖外环 |
| `sls-px4-sitl` | **YAML** | **YAML** | 只覆盖 `controller_mode`、`controller_hover_thrust`；**内环等价验证走这条** |
| `sls-circle-dry` / `real-*` | **YAML** | **YAML** | 转发 53 项，但**不含** `qsf_*` 与 `max_acc_*`/`max_tilt_deg` |
| `sls-circle-gazebo` | **launch 参数** | **launch 参数** | `controller_override_names` 里**包含** 10 个 `qsf_*` 与 `max_acc_xy/max_acc_z/max_total_acc/max_tilt_deg`，改 YAML 不生效 |

> `sls-circle-gazebo` 的加速度限幅（1.2/1.2/2.0）与倾角（12 度）是**仿真稳定性钳位**，
> 刻意小于规格值（该路径没有真实姿态环，放大会发散）。Gazebo 力闭环只验证外环与话题，不代表内环。

### 2.3 确认生效值（每次调参后必做）

```bash
ros2 param get /sls_circle_controller qsf_kv_x
ros2 param get /sls_circle_controller qsf_ka_x
ros2 param get /sls_circle_controller mav_mass
ros2 param dump /sls_circle_controller          # 一次性看全部
```

`ros2 param get` 返回的是**启动时载入的值**，正好等于控制律实际使用的值，因此这是可靠判据。

## 3. 启动与安全门

### 3.1 构建

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select sls_qsf_core sls_circle_controller_cpp sls_circle_controller --symlink-install
source install/setup.bash
```

### 3.2 由低风险到高风险

```bash
./run_echo_drone.sh sls-circle-sim                        # 无 Gazebo、fake MAVROS 点质量闭环
./run_echo_drone.sh sls-circle-gazebo gui:=false           # Gazebo 力闭环烟测（只验外环）
PX4_DIR=/path/to/PX4-Autopilot ./run_echo_drone.sh sls-px4-sitl   # 内环等价平台
./run_echo_drone.sh sls-takeoff-hold-dry                   # dry-run 只看状态机
./run_echo_drone.sh sls-circle-dry                         # dry-run 绕圈
./run_echo_drone.sh real-hold                              # 实机：起飞后只定点悬停
./run_echo_drone.sh real-swing                             # 实机：QSF 绕圈（不含抗风扰）
./run_echo_drone.sh real-wind                              # 实机：QSF 绕圈 + 抗风扰
```

### 3.3 安全门（调参期间不要绕过）

- `dry_run: true` + `enable_real_setpoint: false` 是**双门**，两者同时满足才写
  `/mavros/setpoint_raw/attitude`。
- `real_setpoint_gate` 还要求 `flight_stage == circle`：起飞与悬停阶段只发位置 setpoint，
  让 PX4 自己管高度，避免尚未稳定就接管姿态环。
- 自动 OFFBOARD/解锁带**锁存保护**：一旦检测到飞控自行退出 OFFBOARD 或解除解锁，就锁存
  「人工接管」直到控制器重启。调参时若看到 `manual_mode_override_latched` 或
  `manual_disarm_latched` 为 true，说明已放弃自动请求，需重启节点。
- 实机入口需要 `CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND`。

## 4. 观察手段

```bash
ros2 topic echo /sls_circle/status --once
ros2 topic echo /sls_circle/status | grep -o '"flight_stage":"[^"]*"'
ros2 topic hz     /sls_circle/status
ros2 topic echo /sls_circle/reference_pose --once
ros2 topic echo /sls_circle/debug/attitude_target --once
ros2 topic echo /sls_circle/wind_estimate --once     # 估计力（N）
ros2 topic echo /sls_circle/wind_actual   --once     # 仅 Gazebo 有真值
ros2 topic hz /mavros/local_position/pose
ros2 topic echo /mavros/state --once
```

## 5. status 关键字段（分组）

| 组 | 字段 | 判据/用途 |
|---|---|---|
| 状态机 | `flight_stage`、`mission_mode`、`reason` | `reason` 不是 `real_setpoint_active` 就说明真实输出没放行 |
| 控制源 | `controller_mode`、`controller_source`、`qsf_core_loaded`、`qsf_error` | `qsf_error` 非空表示 QSF 报错并**已回退 PD** |
| 频率/新鲜度 | `target_control_hz`、`actual_control_hz`、`command_publish_hz`、`pose_input_hz`、`velocity_input_hz`、`load_pose_input_hz`、`actual_wind_input_hz`、`pose_age`、`velocity_age`、`actual_wind_age` | 频率应接近 `control_rate`；`pose_age` 必须小于 `pose_stale_timeout` |
| 位姿 | `position`、`velocity`、`reference`、`reference_velocity` | 直接对比参考与实际 |
| 控制量 | `accel_cmd`、`attitude_thrust`、`qsf_force_ned`、`qsf_thrust_accel_enu`、`qsf_net_accel_enu`、`qsf_xi` | 看是否长期贴限幅 |
| QSF 中间量 | `qsf_state_ned_load_semantics`、`qsf_ref_x_north`、`qsf_ref_y_east`、`qsf_ref_z_down` | 核对 NED/ENU 轴交换 |
| 风扰估计 | `wind_estimator_mode`、`disturbance_estimate`、`wind_estimate_force_n`、`wind_estimate_rel_error_xy`、`wind_estimate_rel_error_xy_max` | `disturbance_estimate` 是加速度，`wind_estimate_force_n` = 前者 x `mav_mass` |
| 风扰补偿 | `wind_compensation_gain_target`、`wind_compensation_gain_effective`、`wind_compensation_accel_mps2`、`wind_compensation_force_n`、`wind_compensation_magnitude_n`、`wind_integral` | `effective` 为 0 说明还在 warmup 或 gain 配成 0 |
| 真值（仅 Gazebo） | `actual_wind_force_n`、`actual_wind_magnitude_n`、`actual_wind_direction` | 与估计对比 |
| 飞控/安全 | `mavros_connected`、`mavros_armed`、`mavros_mode`、`offboard_seen`、`armed_seen`、`manual_mode_override_latched`、`manual_disarm_latched`、`dry_run`、`enable_real_setpoint`、`real_setpoint_active` | |

## 6. 分层调参方法

### 6.1 第 0 层：数据链健康度（最高性价比，先做）

先只启动控制器（dry-run 即可），确认：

- `pose_received` = true，`pose_age` 稳定小于 `pose_stale_timeout`（默认 0.5 s）
- `pose_input_hz` 不小于 `control_rate` 的 80%
- `velocity_input_hz`：若为 0 且 `use_velocity_topic` = true，说明速度话题没接上
- `actual_control_hz` 接近 `target_control_hz`

**这一层不过关就不要往下调。**

### 6.2 第 1 层：PD 兜底（`controller_mode=pd`）

QSF 报错时会回退到这套 PD，所以必须先让它可用。

```bash
./run_echo_drone.sh sls-circle-gazebo gui:=true \
  controller_mode:=pd enable_anti_wind:=false enable_wind:=false \
  radius:=0.0 angular_velocity:=0.0
```

调 `kp_xy` / `kd_xy` / `kp_z` / `kd_z`（默认 1.8 / 1.4 / 2.5 / 1.6）：

| 现象 | 动作 |
|---|---|
| 定点有稳态偏差、收敛慢 | `kp_*` 上调 20~30% |
| 超调、来回振荡 | `kd_*` 上调，或 `kp_*` 下调 |
| 高频抖动 | `kd_*` 下调（微分放大了噪声） |

### 6.3 第 2 层：QSF 外环（`controller_mode=qsf`）

**起点必须用 `docs/sls_sim_spec.md` 的权威值**（取自 `drown` 的 `gazebo_sls_force_control_plugin.cc`）：

| 参数 | 参考值 | 含义 |
|---|---|---|
| `qsf_kp_x` / `qsf_kp_y` | 24.0 | 负载位置误差增益 |
| `qsf_kv_x` / `qsf_kv_y` | 50.0 | 负载速度误差增益（摆阻尼主力） |
| `qsf_ka_x` / `qsf_ka_y` | 35.0 | 负载加速度误差增益 |
| `qsf_kj_x` / `qsf_kj_y` | 10.0 | 负载加加速度误差增益 |
| `qsf_kp_z` / `qsf_kv_z` | 2.0 / 3.0 | 垂直通道（参考插件值） |
| `qsf_ki_*` | 12/12/1 | 仅 `controller_mode=qsf_integral` 生效 |
| `qsf_integral_limit` | 10.0 | 上面那组积分的限幅 |

> 本工程**没有** `qsf_geometric` 模式。参考插件的 `QSFGeometricController` 与现有的
> `QSFController` 控制律逐行相同、只差参考生成方式，因此对圆周参考二者数值等价
> （证明与映射见 `docs/sls_sim_spec.md` 第 5 节）。

调参纪律：

1. **一次只改一个参数，幅度 20~30%**，改完按 §2.3 确认生效值并**重启节点**。
2. 每次记录：改前值 / 改后值 / `reference` 与 `position` 的稳态误差 / `accel_cmd` 是否贴限幅 /
   有无高频抖动。
3. `qsf_error` 非空 = QSF 已回退 PD。常见触发是吊摆摆角接近水平时生成代码里的除法奇点
   （`/z[5]`、`/(g - nu_3)`）。若频繁出现，说明摆角超出模型适用范围，**应先降低
   `angular_velocity` 或 `radius`**，而不是继续加增益。

症状 -> 动作对照（在参考值附近微调时使用）：

| 现象 | 优先动作 |
|---|---|
| 圆周跟随有稳态滞后（半径偏小、相位落后） | `qsf_kv_*` 上调 |
| 绕圈时高度波动 | 先查 `qsf_kp_z` / `qsf_kv_z`，再查 `max_thrust` 是否贴顶 |
| 摆动衰减慢、到达目标拖尾 | `qsf_ka_*` 上调（参考值 35，不要一次加太多） |
| 高频抖动 / 电机噪声明显 | `qsf_ka_*`、`qsf_kj_*` 下调（这两个对状态噪声最敏感） |
| 整体发散或低频振荡 | 全部按比例下调，先降 `qsf_ka_*` / `qsf_kj_*` |
| 长期贴限幅 | 权限不够或增益过大，先看 `accel_cmd` 与 `attitude_thrust` |

> 强烈提醒：只有在**存在负载位姿反馈**时，`pend` 与 `pendRate` 才是真的；否则它们恒为「负载挂在
> 正下方」（见 `docs/sls_sim_spec.md` 第 6 节）。此时 QSF 的高阶项实际作用在机体通道上，增益含义
> 与带负载反馈时不同，**不要把 ROS1 带动捕时的调参结论直接搬过来**。

### 6.4 第 3 层：抗风扰

三段递进，每段都要有数据支撑才进下一段。

**(a) 只开估计、不开补偿**（先验证估计器本身）

```bash
./run_echo_drone.sh sls-circle-gazebo gui:=false \
  enable_anti_wind:=true wind_estimator_mode:=residual \
  wind_compensation_gain:=0.0 enable_wind:=true
```

对比 `/sls_circle/wind_estimate` 与 `/sls_circle/wind_actual`，并看 status 的
`wind_estimate_rel_error_xy_max`。

**(b) 开补偿，从半量起**

`wind_compensation_gain` 从 `0.5` 起逐步加到 `1.0`，同时必须设置渐入：

- `wind_compensation_warmup_time`（实机入口默认 5.0 s）：前 N 秒补偿恒为 0
- `wind_compensation_ramp_time`（默认 5.0 s）：之后线性升到目标增益
- 用 `wind_compensation_gain_effective` 确认当前实际用了多少（为 0 就是还在 warmup）

**(c) 只有确认 (b) 稳定后才考虑积分**

`wind_integral_gain` 默认 `0.0`，**当前不要开**。原因：参考插件的 `Ki_res = 1.5` 是与
**10 秒参考渐入**配套的，而那段渐入尚未移植；直接开积分会在起步阶段积分饱和。

| 参数 | 参考/默认 | 含义与调法 |
|---|---|---|
| `enable_anti_wind` | true | 总开关 |
| `wind_estimator_mode` | `residual` | `residual`=运动残差（实机唯一可实现）；`leso`=三阶观测器（参考观测**负载**位置，需负载反馈才有意义）；`actual_feedback`=直接跟随 `/sls_circle/wind_actual`，**仅 Gazebo 对照用** |
| `wind_observer_bandwidth` | 35.0 | 仅 `leso` 模式生效；参考值 35 rad/s，beta3 与带宽立方成正比，改动影响很大 |
| `wind_estimate_filter_tau` | 0.2 | 估计低通时间常数。调大更平滑但更滞后；调小响应快但把噪声灌进补偿 |
| `wind_estimate_force_limit` | 5.0 | 原始残差钳位（单位 N），换算成加速度要除以 `mav_mass` |
| `wind_compensation_gain` | 0.0 -> 1.0 | 补偿强度；1.0 表示按估计值全额反向补偿 |
| `wind_integral_gain` | 0.0 | 保持 0，见上 |
| `wind_integral_limit` | 5.0 | 积分限幅 |

**验收判据**：稳态 `wind_estimate_rel_error_xy_max < 0.3`。注意其中含一个**约 17% 的系统性偏差**：
估计器用 `mav_mass` 做加速度到力的换算，而 Gazebo 桥接把风作用在
`mass = mav_mass + load_mass + 绳`（配置里是 1.82）上，因此估计力系统性偏低
`1 - 1.51/1.82 ~ 17%`。**补偿本身不受影响**（补偿是加速度量），但判据要把这 17% 扣掉再看。

### 6.5 推力标定（`hover_thrust`）

`hover_thrust` 是归一化推力在 1 g 时的取值，必须按实际机体标定：

```bash
./run_echo_drone.sh sls-px4-sitl controller_mode:=thrust_calibration
```

该模式把姿态保持水平、推力固定为 `hover_thrust`，用于反推真实值（需要 launch 显式授权）。
标定后再用于 `real-*` 入口（`real-wind` 当前写的是 `hover_thrust:=0.35`）。

## 7. 常见坑

1. **`ros2 param set` 静默无效**（§0.1）—— 最容易浪费半天的一条。
2. **launch 覆盖 YAML**（§2.2）—— `sls-circle-gazebo` 的外环参数全部来自 launch，改 YAML 无效。
3. **位姿话题在不同入口不一样**：`real-*` 入口把 `pose_topic` 覆盖成 `/mavros/mavros/pose`，
   而单独跑 `sls_circle.launch.py` 用的是 `/mavros/local_position/pose`。`reason` 停在
   `waiting_for_pose` 时，先按实际入口核对话题名。
4. **`use_velocity_topic:=false` 会让残差估计退化**：`real-wind` 入口当前设了 `false`，
   于是观测加速度来自**位置的二阶差分**，噪声放大约 `1/dt^2`。此时先开速度话题
   （`/mavros/mavros/velocity_local` 在实机 bag 里是存在的）再谈估计精度。
5. **`/sls_circle/wind_actual` 只有 Gazebo 有**：实机 `actual_wind_input_hz` 应为 0；
   没有真值就不能用 `wind_estimate_rel_error_xy_max` 当判据。
6. **`disturbance_estimate` 与 `wind_estimate_force_n` 差一个 `mav_mass`**，别把两者当同一量。
7. **`mav_mass` 目前是参考仿真的 1.51**，实机必须重新称重后覆盖。
8. **权限级参数**：改 `max_total_acc` / `max_tilt_deg` / `max_thrust` 会显著改变飞行特性，
   改动前必须先在 `sls-px4-sitl` 复现。
9. **Gazebo 力闭环不等于实机**：该路径没有真实姿态环与速率环，抗风扰与大摆角这类内环敏感行为
   只认 `sls-px4-sitl` 的结果（见 `docs/sls_sim_spec.md` 第 1 节）。

## 8. 调参记录表（建议每次实飞都填）

| 日期 | 入口 | 参数 | 改前 | 改后 | 圆周稳态误差 | accel_cmd 贴限幅? | qsf_error | 摆动表现 | 备注 |
|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | |

## 9. 回滚

对齐到参考规格的改动集中在以下提交，可单独回滚：

| 提交 | 内容 |
|---|---|
| `81c0dc1` | 外环增益 / `mav_mass` / 加速度限幅 |
| `9d381ed` | 各入口 launch 默认值 |
| `524c481` | 仿真风模型（阻力模型） |
| `faf7039` | LESO 带宽 4.0 -> 35.0 |

```bash
git revert 81c0dc1      # 只回滚增益与限幅
git log --oneline       # 确认回滚顺序
```

参考值与本工程旧值的对照见 `docs/sls_sim_spec.md` 的规格表。
