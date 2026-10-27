# nav_bot_navigation — 导航层（阶段 3–5）

SLAM 建图、AMCL 定位、Nav2 参数、行为树。所有参数外置 yaml，launch 里只做引用。

## 目录职责

| 路径 | 放什么 | 状态 |
|---|---|---|
| `config/nav2_params.yaml` | map_server / amcl / bt_navigator / planner / controller / costmap | 待建 |
| `config/slam_toolbox_params.yaml` | 建图参数（`mode: mapping` / `localization`） | 待建 |
| `maps/` | `map.pgm` + `map.yaml`（建图产物，需纳入版本管理） | 空 |
| `behavior_trees/` | 自定义 BT XML（默认用 nav2 自带的，先跑通再改） | 空 |
| `launch/slam.launch.py` | slam_toolbox 在线建图 | 待建 |
| `launch/localization.launch.py` | map_server + amcl | 待建 |
| `launch/navigation.launch.py` | nav2 全栈 + lifecycle_manager | 待建 |

## 启动顺序（不要跳步）

1. `nav_bot_gazebo sim.launch.py`
2. `nav_bot_navigation localization.launch.py` → RViz 给 **2D Pose Estimate**
3. `nav_bot_navigation navigation.launch.py`
4. 检查 global/local costmap 是否正常
5. 点 **2D Goal Pose**

## 参数起点

| 参数 | 建议初值 | 位置 |
|---|---|---|
| `resolution` | 0.05 | costmap / map |
| `inflation_radius` | 0.35 | 全局/局部 costmap |
| `cost_scaling_factor` | 3.0 | costmap |
| `controller_frequency` | 15~20 Hz | controller_server |
| `xy_goal_tolerance` | 0.15~0.25 m | goal_checker |
| `max_vel_x` / `max_vel_theta` | 0.5 / 1.2 | controller |

## 验收

10 个随机目标点成功率 ≥ 90%，平均耗时 < 40 s，无碰撞；卡住时能看到 recovery 后继续。
