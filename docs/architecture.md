# 系统架构与接口契约

> 本文件定义包与包之间"必须对齐"的东西：TF 链、话题、坐标系命名、参数归属。
> 实现阶段只要照这张表填，就不会出现"某一层名字对不上"的静默故障。
> 状态：**骨架已定义，数值待阶段 1-2 确认后填入**。

## 1. TF 树

```text
map ──(amcl)──> odom ──(gz DiffDrive)──> base_footprint ──(static)──> base_link
                                                                         ├── left_wheel_link   (continuous, axis y)
                                                                         ├── right_wheel_link  (continuous, axis y)
                                                                         ├── caster_link       (revolute)
                                                                         └── laser_link        (fixed)
```

| 变换 | 发布者 | 频率 | 说明 |
|---|---|---|---|
| `map → odom` | `amcl` | 随更新 | 修正累积误差；建图时由 slam_toolbox 发布 |
| `odom → base_footprint` | gz `DiffDrive` 插件 | 50 Hz | 保证连续，绝不跳变 |
| `base_footprint → base_link` | `robot_state_publisher` | static | 车体高度 |
| 轮子 / 雷达 | `robot_state_publisher` | 随 `/joint_states` | 由 URDF 决定 |

> 铁律：`odom → base_footprint` 只能有**一个**发布者。gz 插件与 `diff_drive_controller` 同时开会打架。

## 2. 话题契约

| 话题 | 类型 | 方向 | 发布者 | 订阅者 |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | GZ → ROS | gz sim | 所有节点（`use_sim_time`） |
| `/scan` | `sensor_msgs/msg/LaserScan` | GZ → ROS | gz `gpu_lidar` | slam_toolbox、AMCL、costmap |
| `/odom` | `nav_msgs/msg/Odometry` | GZ → ROS | gz `DiffDrive` | AMCL、controller |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | GZ → ROS | 各发布者 | 全链路 |
| `/joint_states` | `sensor_msgs/msg/JointState` | GZ → ROS | gz 插件 | robot_state_publisher |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → GZ | controller_server | gz `DiffDrive` |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 内部 | map_server | planner、costmap |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 内部 | planner_server | RViz |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 内部 | controller_server | RViz |
| `/plan` | `nav_msgs/msg/Path` | 内部 | planner_server | controller、RViz |

QoS 约定：`/scan` 用 `best_effort`；`/map`、`/tf`、costmap 用 `reliable`。

## 3. 坐标系与命名约定

| 项 | 取值 | 定在哪 |
|---|---|---|
| 全局坐标系 | `map` | nav2_params |
| 里程计坐标系 | `odom` | gz 插件 + nav2_params |
| 机器人基座 | `base_footprint`（地面投影）/ `base_link`（车体中心） | URDF |
| 雷达坐标系 | `laser_link` | URDF + 传感器 `<gz_frame_id>` |
| 轮距 | 待定（m） | `properties.xacro`，插件抄同一值 |
| 轮半径 | 待定（m） | 同上 |
| 车体尺寸 | 待定（m） | 同上 |

> `wheel_separation` / `wheel_radius` / `gz_frame_id` 这三处必须与 URDF **逐字一致**，
> 是里程计尺度错误与点云挂错坐标系的高发点。

## 4. 包依赖图

```text
nav_bot_bringup ──┬──> nav_bot_gazebo ──> nav_bot_description
                  └──> nav_bot_navigation ──> nav_bot_description
nav_bot_tools ────────> （运行时通过 action / topic 交互，不依赖上游包）
```

## 5. 参数归属（改参数该去哪个文件）

| 参数组 | 文件 |
|---|---|
| 几何 / 质量 / 惯量 | `nav_bot_description/urdf/common/properties.xacro` |
| gz 插件（轮距、限速、频率） | `nav_bot_description/urdf/gazebo.xacro` |
| 传感器量程 / 采样 | `nav_bot_description/urdf/sensors.xacro` |
| 桥接映射 | `nav_bot_gazebo/config/gz_bridge.yaml` |
| 物理步长 / 场景 | `nav_bot_gazebo/worlds/*.sdf` |
| SLAM | `nav_bot_navigation/config/slam_toolbox_params.yaml` |
| Nav2 全栈 | `nav_bot_navigation/config/nav2_params.yaml` |
| 评测目标点 | `nav_bot_tools/config/goals.yaml` |
