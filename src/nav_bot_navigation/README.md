# nav_bot_navigation — 导航层（阶段 3–5）

SLAM 建图、AMCL 定位、Nav2 参数、行为树。所有参数外置 yaml，launch 里只做引用。

## 目录职责

| 路径                                | 放什么                                                            | 状态                                           |
| ----------------------------------- | ----------------------------------------------------------------- | ---------------------------------------------- |
| `config/nav2_params.yaml`         | map_server / amcl / bt_navigator / planner / controller / costmap | 待建                                           |
| `config/slam_toolbox_params.yaml` | 建图参数（`mode: mapping` / `localization`）                  | ✅ 已建                                        |
| `config/slam.rviz`                | 建图用 RViz（Fixed Frame = map，含 Map/Scan/RobotModel）          | ✅ 已建                                        |
| `maps/`                           | `map.pgm` + `map.yaml`（建图产物，需纳入版本管理）            | 有参考图 `small_house.*`（AWS 自带，非自建） |
| `behavior_trees/`                 | 自定义 BT XML（默认用 nav2 自带的，先跑通再改）                   | 空                                             |
| `launch/slam.launch.py`           | slam_toolbox 在线建图                                             | ✅ 已建，实测可跑                              |
| `launch/localization.launch.py`   | map_server + amcl                                                 | 待建                                           |
| `launch/navigation.launch.py`     | nav2 全栈 + lifecycle_manager                                     | 待建                                           |

## 阶段 3 建图操作（两个终端）

```bash
# 终端 1：仿真
ros2 launch nav_bot_gazebo sim.launch.py x:=0 y:=2.0

# 终端 2：SLAM + EKF + RViz（use_ekf 默认开启）
ros2 launch nav_bot_navigation slam.launch.py

# 终端 3：遥控（线速度 ≤ 0.3 m/s，角速度 ≤ 0.3 rad/s —— 转太快雷达畸变 + 4WD 滑移加剧）
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true

# 地图满意后保存（先备份已有的 AWS 参考图）
cp src/nav_bot_navigation/maps/small_house.pgm src/nav_bot_navigation/maps/small_house_ref.pgm
cp src/nav_bot_navigation/maps/small_house.yaml src/nav_bot_navigation/maps/small_house_ref.yaml
ros2 run nav2_map_server map_saver_cli -f src/nav_bot_navigation/maps/small_house
```

**★ 必须带 EKF**：4WD 滑移转向下轮式里程计旋转误差实测 1.31×（转 34.6° 报 45.4°），
直接建图必然出放射状重影。`slam.launch.py` 默认拉起 robot_localization EKF
（里程计 vx + IMU 陀螺 vyaw 融合，实测误差 1.00×），不要用 `use_ekf:=false` 关掉。

建图要点：走闭合回路（让 slam_toolbox 做 loop closure）、每个房间进一圈、
别原地大甩头（雷达运动畸变会把墙画成弧线）。

**已知坑**：

1. `/scan` 的 `frame_id` 必须是 `lidar_link`（由 `gazebo.xacro` 里的
   `<gz_frame_id>` 决定）。若看到 `nav_bot/base_link/lidar`，说明 URDF 漏了该标签，
   slam_toolbox 会疯狂刷 `Message Filter dropping message ... queue is full` 且
   `/map` 永远不出。
2. `odom→base_link` 的 TF 只能有一个发布者：EKF 发布，DiffDrive 的 TF 已改道
   `/tf_gz_odom`（不桥接）。两处同发会 TF 打架、odom 抽搐。
3. 车实际转角只有指令的 ~20%（4WD 侧滑打滑），遥控会觉得"转向很肉"，属正常；
   但说明阶段 4 的控制器调参会很关键。

## 启动顺序（不要跳步）

1. `nav_bot_gazebo sim.launch.py`
2. `nav_bot_navigation localization.launch.py` → RViz 给 **2D Pose Estimate**
3. `nav_bot_navigation navigation.launch.py`
4. 检查 global/local costmap 是否正常
5. 点 **2D Goal Pose**

## 参数起点

| 参数                              | 建议初值    | 位置              |
| --------------------------------- | ----------- | ----------------- |
| `resolution`                    | 0.05        | costmap / map     |
| `inflation_radius`              | 0.35        | 全局/局部 costmap |
| `cost_scaling_factor`           | 3.0         | costmap           |
| `controller_frequency`          | 15~20 Hz    | controller_server |
| `xy_goal_tolerance`             | 0.15~0.25 m | goal_checker      |
| `max_vel_x` / `max_vel_theta` | 0.5 / 1.2   | controller        |

## 验收

10 个随机目标点成功率 ≥ 90%，平均耗时 < 40 s，无碰撞；卡住时能看到 recovery 后继续。
