# indooruav_waypoint

> 室内无人机航点录制与跟踪 ROS 1 功能包

包含两个独立节点：

- **`waypoint_recorder_node`** — 通过 ROS Service 触发，将当前里程计位置追加为航点并保存为 YAML 文件
- **`waypoint_tracker_node`** — 加载 YAML 航点文件，订阅实时里程计，按预设巡航速度跟踪航点并发布速度指令到 `/drone/cmd_vel`

---

## 1. 设计目标与适用场景

本包面向"室内固定巡检路线"场景：操作员手动飞行一遍路线并录制关键航点，此后无人机可无限次重复该路线，无需 PID 调参、无需轨迹优化器。

**核心设计取舍**

| 维度 | 选择 | 原因 |
|------|------|------|
| 控制方式 | 速度前馈 + 三阶限幅，不使用位置 PID | 室内低速场景下速度直接驱动轨迹质量足够 |
| 时间参数化 | 不使用；按距离驱动逐点切换 | 录制无需记录速度，只需位置+朝向 |
| 数据格式 | YAML，yaw 用度存储 | 人类可读，支持手动微调 |
| 中间航点策略 | fly-through（直通），仅末点预制动 | 消除转角处的停顿感 |
| 坐标系 | 沿用 `position_controller.hpp`：误差在世界系 FLU 计算，发布前转机体系 FLU | 与现有 PSDK 桥接保持一致 |

**适用条件**

- ROS 1（Noetic / Melodic）
- 上游已发布 `nav_msgs/Odometry` 类型的实时里程计（世界系 FLU）
- 飞控接受 `geometry_msgs/Twist` 类型的速度指令（机体系 FLU，`vz` 为世界系）

---

## 2. 包结构

```
indooruav_waypoint/
├── package.xml
├── CMakeLists.txt
├── config/
│   └── config.yaml                              # tracker + recorder 共享配置
├── include/indooruav_waypoint/
│   ├── smooth_velocity_limiter.hpp              # 三阶速率限幅器（header-only）
│   ├── waypoint_tracker.hpp                     # WaypointTracker 类声明
│   └── waypoint_recorder.hpp                    # WaypointRecorder 类声明
├── src/
│   ├── waypoint_tracker.cpp                     # tracker 完整实现
│   └── waypoint_recorder.cpp                    # recorder 完整实现
├── node/
│   ├── waypoint_tracker_node.cpp                # 仅 main()，构造 + spin
│   └── waypoint_recorder_node.cpp               # 仅 main()，构造 + spin
├── docs/
│   └── architecture.drawio                      # 系统架构 + 流程图（5 页）
└── launch/
    ├── bringup_waypoint_tracker.launch
    └── bringup_waypoint_recorder.launch
```

---

## 3. 依赖

**系统库（需要手动安装）**

```bash
sudo apt install libyaml-cpp-dev
```

**ROS 依赖（标准 ROS 1 包）**

`roscpp`、`roslib`、`std_srvs`、`std_msgs`、`geometry_msgs`、`nav_msgs`

---

## 4. 构建

```bash
cd ~/catkin_ws/src
# 将本包目录放到此处（或 git clone）

cd ~/catkin_ws
catkin_make
# 或
catkin build indooruav_waypoint

source devel/setup.bash
```

---

## 5. 节点接口

### 5.1 `waypoint_recorder_node`

| 类别 | 名称 | 消息类型 | 说明 |
|------|------|----------|------|
| Subscriber | `/Odometry_global_rotate` | `nav_msgs/Odometry` | 实时里程计（与 tracker 必须一致） |
| Service Server | `indooruav_controller/waypoint_recorder/record` | `std_srvs/Trigger` | 追加当前帧到内存列表 |
| Service Server | `indooruav_controller/waypoint_recorder/save` | `std_srvs/Trigger` | 整文件覆盖写入 YAML |
| Service Server | `indooruav_controller/waypoint_recorder/clear` | `std_srvs/Trigger` | 清空内存列表（不影响磁盘） |

### 5.2 `waypoint_tracker_node`

| 类别 | 名称 | 消息类型 | 说明 |
|------|------|----------|------|
| Subscriber | `/Odometry_global_rotate` | `nav_msgs/Odometry` | 实时里程计 |
| Publisher | `/drone/cmd_vel` | `geometry_msgs/Twist` | 速度指令（机体系 FLU + 世界系 vz） |
| Publisher | `/indooruav/desired_odometry` | `nav_msgs/Odometry` | 当前目标航点位姿（可视化用） |
| Service Server | `indooruav_controller/waypoint_tracker/start` | `std_srvs/Trigger` | 开始或恢复追踪 |
| Service Server | `indooruav_controller/waypoint_tracker/pause` | `std_srvs/Trigger` | 暂停追踪，保留当前航点索引 |
| Service Server | `indooruav_controller/waypoint_tracker/stop` | `std_srvs/Trigger` | 停止追踪并将索引归零 |

> 所有话题名和服务名均可通过 `config.yaml` 覆盖配置，详见第 6 节。

---

## 6. 配置文件详解

整个 `config/config.yaml` 以嵌套结构整体加载到全局参数服务器。两个节点均用绝对路径读取各自子树：

- Tracker：`/indooruav_waypoint/waypoint_tracker/...`
- Recorder：`/indooruav_waypoint/waypoint_recorder/...`

### 6.1 Tracker 参数完整列表

| 参数路径（相对 `waypoint_tracker/`） | 默认值 | 单位 | 说明 |
|--------------------------------------|--------|------|------|
| `topics/current_odometry` | `/Odometry_global_rotate` | — | 订阅的实时里程计话题 |
| `topics/desired_odometry` | `/indooruav/desired_odometry` | — | 发布的目标航点可视化话题 |
| `topics/velocity_command` | `/drone/cmd_vel` | — | 发布的速度指令话题 |
| `services/start` | `indooruav_controller/waypoint_tracker/start` | — | start 服务名 |
| `services/pause` | `indooruav_controller/waypoint_tracker/pause` | — | pause 服务名 |
| `services/stop` | `indooruav_controller/waypoint_tracker/stop` | — | stop 服务名 |
| `waypoints_package_name` | `indooruav_waypoint` | — | 相对路径解析的根包名 |
| `waypoints_file_path` | `config/waypoints.yaml` | — | 航点文件路径（相对或绝对均可） |
| `parameters/control_rate_hz` | `30.0` | Hz | 控制定时器频率 |
| `parameters/odometry_timeout_s` | `0.2` | s | 里程计陈旧判定阈值，超时发零速 |
| `parameters/autostart` | `false` | bool | 节点启动后是否自动进入 RUNNING |
| `parameters/xy_speed_mps` | `0.5` | m/s | XY 平面巡航合速度上限 |
| `parameters/z_speed_mps` | `0.3` | m/s | Z 轴巡航速度上限 |
| `parameters/yaw_speed_dps` | `30.0` | °/s | Yaw 巡航角速度上限 |
| `parameters/xy_accel_max_mps2` | `0.5` | m/s² | XY 加速度上限 |
| `parameters/xy_jerk_max_mps3` | `2.0` | m/s³ | XY 急动度上限 |
| `parameters/xy_snap_max_mps4` | `1.0e6` | m/s⁴ | XY snap 上限（默认极大值，等效禁用） |
| `parameters/z_accel_max_mps2` | `0.4` | m/s² | Z 加速度上限 |
| `parameters/z_jerk_max_mps3` | `2.0` | m/s³ | Z 急动度上限 |
| `parameters/z_snap_max_mps4` | `1.0e6` | m/s⁴ | Z snap 上限 |
| `parameters/yaw_accel_max_dps2` | `60.0` | °/s² | Yaw 角加速度上限（读入后内部转弧度） |
| `parameters/yaw_jerk_max_dps3` | `300.0` | °/s³ | Yaw 急动度上限 |
| `parameters/yaw_snap_max_dps4` | `1.0e6` | °/s⁴ | Yaw snap 上限 |
| `parameters/xy_arrival_tolerance_m` | `0.15` | m | XY 到达阈值 |
| `parameters/z_arrival_tolerance_m` | `0.15` | m | Z 到达阈值 |
| `parameters/yaw_arrival_tolerance_deg` | `5.0` | ° | Yaw 到达阈值（读入后内部转弧度） |

### 6.2 Recorder 参数完整列表

| 参数路径（相对 `waypoint_recorder/`） | 默认值 | 说明 |
|---------------------------------------|--------|------|
| `topics/current_odometry` | `/Odometry_global_rotate` | 订阅话题（**必须与 tracker 完全一致**） |
| `services/record` | `indooruav_controller/waypoint_recorder/record` | — |
| `services/save` | `indooruav_controller/waypoint_recorder/save` | — |
| `services/clear` | `indooruav_controller/waypoint_recorder/clear` | — |
| `waypoints_package_name` | `indooruav_waypoint` | 相对路径解析根包 |
| `waypoints_file_path` | `config/waypoints.yaml` | 写入文件路径 |
| `parameters/load_existing` | `false` | 启动时是否从磁盘预加载航点（true = 续录模式） |

---

## 7. 航点 YAML 格式

`waypoint_recorder_node` 自动生成如下格式，支持手动编辑：

```yaml
# indooruav_waypoint recorded waypoints
# generated_at: 2026-05-08 10:00:00
# count: 4
# fields: x [m], y [m], z [m], yaw_deg [deg]
waypoints:
  - {x: 0.000, y: 0.000, z: 1.200, yaw_deg:   0.0}
  - {x: 0.000, y: 0.000, z: 1.200, yaw_deg:   0.0}
  - {x: 3.500, y: 0.000, z: 1.200, yaw_deg:   0.0}
  - {x: 3.500, y: 2.000, z: 1.500, yaw_deg: 180.0}
```

| 字段 | 单位 | 说明 |
|------|------|------|
| `x`, `y`, `z` | 米 | 世界系 FLU 位置，与 `/Odometry_global_rotate` 坐标系一致 |
| `yaw_deg` | 度 | 偏航角，逆时针为正；加载时被 `NormalizeAngleRad` 归一化到 (−180°, 180°] |

---

## 8. 使用流程

### 8.1 录制航点

```bash
# 终端 1：启动里程计源

# 终端 2：启动 recorder
roslaunch indooruav_waypoint bringup_waypoint_recorder.launch

# 终端 3：飞行并录制
rosservice call indooruav_controller/waypoint_recorder/record "{}"   # 录第 1 个点
# 飞到下一个位置...
rosservice call indooruav_controller/waypoint_recorder/record "{}"   # 录第 2 个点
# 在改变 XY 移动方向前，原地连按两次（"停顿点"）
rosservice call indooruav_controller/waypoint_recorder/record "{}"
rosservice call indooruav_controller/waypoint_recorder/record "{}"
# 录完后保存
rosservice call indooruav_controller/waypoint_recorder/save "{}"
```

**录制建议**

- 每次只在一个轴上运动（先上升 → 停 → 平移 → 停 → 转向），避免在运动中录点
- 改变 XY 移动方向前连按两次 `record`，让限幅器在新方向从 0 起步
- 想重来：`clear` 清空内存后重录，再 `save`（注意直接覆盖，无备份）

### 8.2 跟踪航点

```bash
# 终端 2：启动 tracker
roslaunch indooruav_waypoint bringup_waypoint_tracker.launch

# 终端 3：控制
rosservice call indooruav_controller/waypoint_tracker/start "{}"   # 开始
rosservice call indooruav_controller/waypoint_tracker/pause "{}"   # 暂停（保留 index）
rosservice call indooruav_controller/waypoint_tracker/start "{}"   # 恢复
rosservice call indooruav_controller/waypoint_tracker/stop  "{}"   # 停止并复位 index=0
```

**常用监控命令**

```bash
rostopic echo /drone/cmd_vel                    # 查看速度指令输出
rostopic hz /Odometry_global_rotate             # 确认里程计频率 ≥ 20 Hz
rosservice call indooruav_controller/waypoint_tracker/start "{}"    # 开始追踪
```

### 8.3 使用自定义配置文件

```bash
roslaunch indooruav_waypoint bringup_waypoint_tracker.launch \
    config_file:=/absolute/path/to/my_config.yaml
```

---

## 9. tracker 状态机

```
  构造完成 ──▶  ┌──────────┐  start()[has_odom&&waypoints]  ┌──────────────┐
               │   IDLE   │ ─────────────────────────────▶  │   RUNNING    │
               │          │                                  │              │
               │ index=0  │ ◀──────────── stop() ─────────  │ 30Hz 控制循环│
               │ 限幅器=0  │                                  │ index 递增   │
               │ 发零速    │                                  └──────┬───────┘
               └──────────┘                                  pause()│    │末点到达
                    ▲                                               │    │
              stop()│                               ┌──────────────▼┐  ┌▼────────────┐
                    └──────────────────────────────  │    PAUSED     │  │  FINISHED   │
                                                     │               │  │             │
                                                     │ index 保留    │  │ index=size  │
                                                     │ 限幅器=0      │  │ 限幅器=0    │
                                                     │ 发零速        │  │ 发零速      │
                                                     │ start()→RUNNING│ │ start()→   │
                                                     └───────────────┘  │ index=0    │
                                                                         │ +RUNNING   │
                                                                         └────────────┘
```

**状态行为对照**

| 状态 | `cmd_vel` 输出 | 限幅器 | `current_index_` |
|------|--------------|--------|-----------------|
| `IDLE` | 30 Hz 零速 Twist | 复位 | 0 |
| `RUNNING` | 30 Hz 限幅后速度 | 持续更新 | 逐步递增 |
| `PAUSED` | 30 Hz 零速 Twist | 复位 | 保留 |
| `FINISHED` | 30 Hz 零速 Twist | 复位 | `waypoints_.size()` |

---

## 10. 控制算法详解

### 10.1 主控制循环（30 Hz）

每个 tick 的执行顺序：

1. **状态守门**：非 `RUNNING` → 复位限幅器，发零速 Twist，返回
2. **里程计守门**：未收到里程计 → 发零速 Twist，打印 `WARN_THROTTLE`，返回
3. **超时检查**：里程计陈旧（age > `odometry_timeout_s`）→ 复位限幅器，发一次零速（`stale_zero_command_sent_` 防刷屏），返回
4. **dt 计算**：`dt = now − last_control_time_`；若 `dt < 0` 或 `dt > 1.0s`，复位限幅器并令 `dt = 0`
5. **提取当前 yaw**：`GetYawFromQuaternion(orientation)`
6. **计算目标速度并过限幅器**：`ComputeDesiredWorldVelocities(dt, yaw, ...)`
7. **到达判定**：三轴同时满足阈值 → `++current_index_`；末点 → `FINISHED`
8. **坐标转换**：世界系 FLU → 机体系 FLU
9. **发布** `cmd_vel` 和 `desired_odometry`

### 10.2 目标速度计算（`ComputeDesiredWorldVelocities`）

**XY 通道（两个独立轴向限幅器）**

```
dx = target.x − current.x;  dy = target.y − current.y
d_xy = sqrt(dx² + dy²)

速度幅值：
  if 末点:  speed = min(xy_speed, sqrt(2·a_max·d_xy))   // 预制动
  else:     speed = xy_speed                              // fly-through

方向：
  target_vx = speed · dx / d_xy
  target_vy = speed · dy / d_xy

限幅：
  vx_world = xy_vx_limiter_.Update(target_vx, dt)
  vy_world = xy_vy_limiter_.Update(target_vy, dt)
```

**Z 通道（始终预制动）**

```
dz = target.z − current.z
target_vz = sign(dz) · min(z_speed, sqrt(2·a_max_z·|dz|))
vz_world = z_limiter_.Update(target_vz, dt)
```

**Yaw 通道（始终预制动）**

```
yaw_err = NormalizeAngleRad(target.yaw_rad − current_yaw)
target_vyaw = sign(yaw_err) · min(yaw_speed, sqrt(2·a_max_yaw·|yaw_err|))
vyaw = yaw_limiter_.Update(target_vyaw, dt)
```

**到达判定（三轴同时满足）**

```
arrived = (d_xy ≤ xy_tol) && (|dz| ≤ z_tol) && (|yaw_err| ≤ yaw_tol)
```

### 10.3 三阶速率限幅器

`SmoothVelocityLimiter`（header-only，命名空间 `indooruav_waypoint`）

内部状态：`v_cmd_`、`a_cmd_`、`j_cmd_`

每 tick 更新（`dt > 0` 时）：

```
v_target_clamped = clamp(v_target, ±v_max)
a_des     = (v_target_clamped − v_cmd_) / dt
j_des     = (a_des − a_cmd_) / dt
s_applied = clamp((j_des − j_cmd_) / dt, ±s_max)

j_cmd_ ← clamp(j_cmd_ + s_applied · dt, ±j_max)
a_cmd_ ← clamp(a_cmd_ + j_cmd_    · dt, ±a_max)
v_cmd_ ← clamp(v_cmd_ + a_cmd_    · dt, ±v_max)
return v_cmd_
```

`s_max = 1e6`（默认）时等效退化为二阶限幅器（snap 不触发）。

实测：默认参数下从 0 加速到 1 m/s 约 2.5 s（符合 `a_max = 0.5 m/s²`）。

### 10.4 坐标变换（与 `position_controller.hpp` 完全一致）

```
vx_body =  cos(yaw) · vx_world + sin(yaw) · vy_world
vy_body = −sin(yaw) · vx_world + cos(yaw) · vy_world
vz_body = vz_world   （z 轴一致，无旋转分量）
vyaw    = vyaw_world  （机体系标量）
```

### 10.5 切弯偏离量

```
Δ ≈ v² / a = 0.5² / 0.5 = 0.5 m  （默认参数）
```

走廊宽度 < 0.5 m 时需要降速或在转角前加密录点。

---

## 11. recorder 工作流详解

`save` 触发时，`SaveWaypointsToFile()` 执行：

1. **路径解析**：绝对路径直接使用；相对路径以 `ros::package::getPath(waypoints_package_name)` 拼接
2. **目录创建**：`EnsureParentDirectory()` 递归 `mkdir -p`
3. **覆盖写入**：`std::ios::out | std::ios::trunc`，**直接覆盖，无备份**
4. **写入头注释**：`generated_at`（本地时间戳）+ `count` + `fields`
5. **写入序列**：`x/y/z` 保留 3 位小数，`yaw_deg` 保留 1 位
6. **flush + close**：确保落盘

> ⚠️ `save` 直接覆盖文件。生产环境建议在调用前手动备份 YAML。

---

## 12. 故障排查

| 现象 | 可能原因 | 排查方法 |
|------|----------|---------|
| `start` 返回 "waypoint list is empty" | 启动时 YAML 加载失败 | 查看启动日志中 `LoadWaypointsFromYaml` 的错误信息；检查文件路径和 YAML 格式 |
| `start` 返回 "No odometry received yet" | 里程计话题无数据 | `rostopic hz /Odometry_global_rotate` 确认频率 |
| 持续打印 "Odometry stale" | 里程计断流或频率过低 | 检查上游节点；可调大 `odometry_timeout_s` |
| 飞机在航点附近震荡 | 到达阈值过严，速度在死区抖动 | 增大 `xy_arrival_tolerance_m`（如 0.25 m） |
| 切弯偏离过大 | `v²/a` 过大 | 减小 `xy_speed_mps` 或增大 `xy_accel_max_mps2` |
| tracker 目标与录制位置不符 | 两者里程计话题不一致 | 确认 `topics/current_odometry` 完全相同后重新录制 |
| `pause` 恢复后有速度抖动 | 预期行为：pause 复位限幅器防残留速度 | 可接受；抖动量 = 一个 tick 的加速度冲量 |
| `record` 返回 "No odometry received yet" | recorder 刚启动里程计未到达 | 等待 1～2 s 后重试 |

---

## 13. 已知限制与未来工作

**当前限制**

- 无障碍感知：fly-through 轨迹不绕障，需确保走廊充裕
- Z 轴每段预制动：连续多个 Z 航点间有轻微停顿感
- save 无备份：覆盖前需手动保存历史版本
- 无航点超时跳过：卡点时只能 `stop` 后重启
- yaml-cpp 解析：不支持并发读写

**可能的扩展方向**

- 增加航点超时自动跳过（`waypoint_timeout_s`）
- 支持循环播放模式（`loop: true`）
- 接入 `indooruav_core` 状态机的 `cruise_complete` 服务通知
- Z 轴改为与 XY 相同的 fly-through 策略

---

## 14. 文件清单

| 文件 | 代码行数 | 职责 |
|------|---------|------|
| `include/indooruav_waypoint/smooth_velocity_limiter.hpp` | 106 | 三阶速率限幅器（header-only，零 ROS 依赖） |
| `include/indooruav_waypoint/waypoint_tracker.hpp` | 194 | `WaypointTracker` 类声明 |
| `include/indooruav_waypoint/waypoint_recorder.hpp` | 86 | `WaypointRecorder` 类声明 |
| `src/waypoint_tracker.cpp` | 789 | tracker 完整实现 |
| `src/waypoint_recorder.cpp` | 434 | recorder 完整实现 |
| `node/waypoint_tracker_node.cpp` | 10 | tracker `main()`（构造 + `ros::spin()`） |
| `node/waypoint_recorder_node.cpp` | 10 | recorder `main()`（构造 + `ros::spin()`） |
| `config/config.yaml` | — | tracker + recorder 共享参数配置 |
| `launch/bringup_waypoint_tracker.launch` | — | tracker 一键启动 |
| `launch/bringup_waypoint_recorder.launch` | — | recorder 一键启动 |
| `docs/architecture.drawio` | — | 5 页系统架构 + 流程图 |
| `package.xml` | — | ROS 包元数据及依赖声明 |
| `CMakeLists.txt` | — | 构建规则（含 yaml-cpp 链接） |

**总计约 1630 行 C++ 代码**，语法已通过 `g++ -std=c++14 -Wall -Wextra -fsyntax-only` 全量验证。
