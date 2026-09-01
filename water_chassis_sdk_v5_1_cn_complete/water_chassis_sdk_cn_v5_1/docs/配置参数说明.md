# WATER SDK `config.json` 配置参数说明

配置文件位置：

```text
water_chassis_sdk/config.json
```

原则：**正常调用者只改 config，不改底层源码。**

## `robot`

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `host` | `192.168.10.10` | WATER 底盘 IP |
| `port` | `31001` | WATER 厂家 TCP API 端口 |
| `connect_timeout_s` | `3.0` | 单次 TCP 连接超时 |
| `request_timeout_s` | `3.0` | 厂家 API 请求等待时间 |
| `reconnect_delay_s` | `1.0` | TCP 断线后的重连间隔 |

## `gateway`

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `host` | `127.0.0.1` | 本地 Gateway 地址 |
| `port` | `8080` | 本地 Gateway 端口 |
| `auto_start` | `true` | `WaterChassis()` 是否自动启动 Gateway |
| `startup_timeout_s` | `8.0` | 等待 Gateway 启动的最大时间 |
| `chassis_wait_timeout_s` | `12.0` | 等待真实底盘上线的最大时间 |

一般不要改 Gateway 地址；如果 8080 和其他程序冲突，可以改端口。

## `state`

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `status_frequency_hz` | `5.0` | robot_status 订阅频率 |
| `velocity_frequency_hz` | `10.0` | robot_velocity 订阅频率 |
| `power_poll_period_s` | `5.0` | 主动刷新电源信息周期 |
| `pose_stale_after_s` | `1.0` | 位姿多久没有更新后认为过期 |
| `stream_frequency_hz` | `5.0` | HTTP/WS 状态流默认频率 |

## `feedback`

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enabled` | `true` | 是否启用动作过程反馈 |
| `interval_s` | `0.25` | 控制台/回调反馈周期 |
| `print_to_console` | `true` | 是否打印到终端 |
| `show_pose` | `true` | 是否显示位姿 |
| `show_velocity` | `true` | 是否显示速度 |
| `show_power` | `false` | 是否显示电量 |
| `show_progress` | `true` | 是否显示百分比进度 |

觉得日志太密，可以：

```json
"interval_s": 0.5
```

## `direct` 顶层限制

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `max_linear_mps` | `0.35` | DIRECT 最大绝对线速度 |
| `max_angular_rps` | `0.70` | DIRECT 最大绝对角速度 |
| `watchdog_s` | `0.35` | 实时速度指令超时停车时间 |
| `command_rate_hz` | `10.0` | 推荐 DIRECT 命令刷新频率 |

这些是 SDK 级安全上限。调用者即使传更大值，也会受到 Gateway 限制。

## `direct.rotation`

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `max_angular_rps` | `0.30` | 闭环转向默认最大角速度 |
| `tolerance_deg` | `2.0` | 角度误差进入多少度以内算完成 |
| `kp` | `1.8` | 转向比例增益 |
| `min_angular_rps` | `0.08` | 为克服静摩擦保留的最小角速度 |
| `timeout_factor` | `2.5` | 根据理论时间估算超时的倍率 |
| `timeout_extra_s` | `2.0` | 超时额外余量 |

如果真车接近目标角度时来回摆动，优先考虑降低 `kp` 或 `min_angular_rps`；如果差一点总转不到，可以适当放宽 `tolerance_deg`。

## `direct.distance`

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `speed_mps` | `0.10` | 默认行驶速度 |
| `tolerance_m` | `0.015` | 距离误差容差，默认 1.5 cm |
| `heading_hold` | `true` | 直线行驶时是否保持初始航向 |
| `speed_kp` | `0.85` | 接近目标时的距离比例控制增益 |
| `min_speed_mps` | `0.035` | 闭环末段最小速度 |
| `heading_kp` | `1.6` | 航向保持增益 |
| `max_heading_correction_rps` | `0.20` | 航向保持最大修正角速度 |
| `timeout_factor` | `2.8` | 距离动作超时倍率 |
| `timeout_extra_s` | `3.0` | 距离动作超时额外余量 |

## `direct.relative`

这是 `move_relative()` 默认参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `linear_speed_mps` | `0.10` | 相对移动阶段默认线速度 |
| `angular_speed_rps` | `0.30` | 相对转向阶段默认最大角速度 |
| `distance_tolerance_m` | `0.015` | 距离容差 |
| `angle_tolerance_deg` | `2.0` | 角度容差 |

## `navigation`

只影响 WATER 厂家自主导航：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `distance_tolerance_m` | `0.05` | 厂家导航目标位置容差 |
| `yaw_tolerance_deg` | `6.0` | 厂家导航目标航向容差 |
| `task_timeout_s` | `60.0` | 默认导航任务超时 |
| `max_continuous_retries` | `30` | 厂家连续重试上限 |

## 推荐调参顺序

真车调 DIRECT 时建议：

```text
1. 先调 speed / max_angular_rps
2. 再调 tolerance
3. 再调 kp
4. 最后动 min_speed / min_angular_rps
```

不要一开始同时改很多参数，否则很难判断哪个参数造成行为变化。
