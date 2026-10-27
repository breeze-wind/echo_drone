# SLS 外环规格：以 Gazebo 力控制插件为基准

> **基准文件**：`drown/controller_sitl_gazebo/src/gazebo_sls_force_control_plugin.cc`（822 行）
> **策略**：方案 A —— 外环对齐 + PX4-SITL 作内环桥梁
> **目的**：让 echo_drone 的 SLS 实机链路与参考仿真在**外环**上逐位一致，并把内环差异显式化，
> 避免后人误以为仿真结果可以直接外推到实机。

## 1. 仿真与实机的边界

参考插件不是"被控对象仿真"，而是**把整条控制栈放进 Gazebo**：

```
QSF 外环(插件内) -> 力限幅 -> acc2quaternion 姿态 -> 角速率 PID -> AddRelativeForce / AddRelativeTorque
```

实机链路：

```
QSF 外环(SLS 节点) -> /mavros/setpoint_raw/attitude -> PX4 姿态环 -> PX4 速率环 -> 电机
```

因此：

- **可以做到"一样"**：QSF 外环、参考轨迹、LESO 抗风扰、限幅 —— 同一套代码、同一组参数。这是决定吊运摆抑制与抗风扰性能的部分。
- **做不到"一样"**：姿态环与速率环的执行者不同（插件内置 PID vs PX4）。
- **内环等价的唯一验证平台**：`sls-px4-sitl`（PX4 SITL + 同一个外部控制器走 MAVROS）。风扰抑制、大摆角这类内环敏感行为，只认这条链路的结果。

## 2. 外环权威规格（全部取自插件）

| 项 | 值 | 插件位置 |
|---|---|---|
| QSF 变体 | `QSFGeometricController`（不是 `QSFController`） | `:362` |
| 增益 K[10] | `{24, 50, 35, 10, 24, 50, 35, 10, 2, 3}` | `:53` |
| param[4] | `{load_mass=0.25, mav_mass=1.51, cable_length=0.85, g=9.80665}` | `:54` |
| 控制力限幅 | `3 * mav_mass * g` = 44.4 N（约 2.5 g 总推力，基本不约束） | `:252`, `:495` |
| 姿态约定 | `acc2quaternion(总推力加速度, yaw)`，内部**不再加 g** | `:503`, `:668` |
| 推力 | `thrust_N = (总推力加速度 · z_b) * mav_mass` | `:510`, `:621` |
| 姿态带宽 | `attctrl_tau_ = 0.05` | `:66` |
| LESO | `leso_wo_ = 35.0` rad/s、`Ki_res = 1.5`、`integral_limit = 10.0` | `:178`, `:459`, `:469` |
| LESO 观测通道 | **负载**位置 `sls_state[0/1/2]` | `:436-443` |
| 风模型 | 阻力 `F = drag_coeff ⊙ (wind_speed + noise - mavVel)`；`drag_coeff={0.7,0.7,0}` N·s/m、`wind_speed={2.0,1.5,0}` m/s、湍流 0.5（噪声限幅 ±1.5） | `:795-803` |
| 参考轨迹 | `center_enu={0,0,1}`、`radium={1,1,0}`、`frequency={1,1,0}` **Hz**、`phase={1.57,0,0}` | `:104-107` |
| 任务点 | `{0,0,1}` -> `{1.5,0,1}` -> `{0,1.5,1}` | `:110-112` |
| 状态序 | `[load.y, load.x, -load.z, pend.y, pend.x, -pend.z, load.vy, load.vx, -load.vz, pendRate.y, pendRate.x, -pendRate.z]` | `:342-353` |
| 参考轴交换 | `ref_x=ref_y_, ref_y=ref_x_, ref_z=-ref_z_` | `:114-118` |
| 控制力 NED->ENU | `(F_y, F_x, -F_z)` | `:492` |

## 3. 内环契约（仿真侧的"等效执行器"）

插件用自带速率 PID 代替 PX4：

| 项 | 值 | 位置 |
|---|---|---|
| 积分限幅 | `lim_int_ = 1.5` | `:256` |
| 生效增益 | `p={1.5,1.5,3.0}`、`i={0.1,0.1,0.1}`、`d={0.004,0.004,0}` | `:263-266` |
| 速率 LPF | `fcRate_ = 100 Hz`、`fcRateD_ = 30 Hz`、`fcAtt_ = 30 Hz` | `:87`, `:92`, `:97` |
| 力矩限幅 | `maxTorque_ = 100` | `:76` |

**用法**：这组参数代表"实机 PX4 应有的内环带宽"。实机验收时以 `fcRate = 100 Hz` 量级的速率环响应为参照；若 PX4 侧明显更慢，外环调参结果不可直接外推。

## 4. 参考工程自身的问题（不要照抄）

1. **风扰施加在错误的坐标系**：`:526 baseLink->AddRelativeForce(actual_wind)`。`computeWindForce()` 返回的是**世界系**阻力（`:797 rel_vel = current_wind - mavVel_` 全为世界系量），而 Gazebo 的 `AddRelativeForce` 是**机体系**。机体一倾斜，风的方向就被多旋转一次，水平风会漏到竖直轴。应改为 `AddForce`（世界系），与 `:566` 的外环分支一致。echo_drone 的 Python 桥接已把风放在世界系 Wrench 里，**这一点移植版反而是对的**，不要把它改回参考的写法。
2. **z 增益与质量不自洽**：插件 `K_` 用 `Kp_z=2, Kv_z=3`、`mav_mass=1.51`；而 `drown/mrotor_controller/launch/drone1_sls_controller.launch` 的 `:92,96,81` 生效值是 `Kp_z=10`、`Kv_z=10.9545`、`mav_mass=1.56`。本文暂以**插件值**为权威（因为插件是被指定的仿真基准），但需要机主最终确认；`mav_mass` 无论如何应在实机上重新称重。
3. **几何轨迹多数分支传 `t=0`**（`:362-403`），只有 `:408` 传 elapsed。这会让轨迹在多数任务阶段被冻结在 t=0，需确认是否本意。
4. **死代码**：`:258-261` 的第一组速率 PID 增益立即被 `:263-266` 覆盖；`sgn()` / `sat()`（`:806-816`）在基础版里未被调用（滑模补偿只存在于 `_my_change.cc`）。

## 5. 对齐进度

- [x] 建立权威规格（本文）
- [ ] 外环参数对齐：QSF 增益、质量、力限幅
- [ ] QSF 变体对齐：新增 `qsf_geometric` 模式（12 维轨迹参考接口）
- [ ] LESO 对齐：负载观测通道 + `Ki_res=1.5` + 参考渐入
- [ ] 仿真风模型对齐：常量力改为阻力模型
- [ ] 内环等价验证：跑通 `sls-px4-sitl` 并记录速率环响应
