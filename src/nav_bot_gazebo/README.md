# nav_bot_gazebo — 仿真层（阶段 2）

把 URDF 放进 Gazebo Harmonic 跑起来，并把 gz 话题桥到 ROS 2。

> Jazzy 环境**没有** Gazebo Classic。`gazebo_ros` + `libgazebo_ros_diff_drive.so` 那一套跑不通，
> 必须用 `ros_gz` + `gz::sim::systems::DiffDrive`。

## 目录职责

| 路径 | 放什么 | 状态 |
|---|---|---|
| `worlds/room_10x10.sdf` | 自建场景：10×10 m、走廊 1.2 m、一窄一宽两扇门、若干障碍 | 待建 |
| `models/` | 自定义模型（家具、动态行人） | 空 |
| `config/gz_bridge.yaml` | ros_gz_bridge 话题映射 | 待建 |
| `launch/sim.launch.py` | 起 gz sim + 生成机器人 + robot_state_publisher + bridge | 待建 |

## 桥接清单（缺一个就断链）

| ROS 话题 | 方向 | 类型 |
|---|---|---|
| `/clock` | GZ → ROS | `rosgraph_msgs/msg/Clock` |
| `/scan` | GZ → ROS | `sensor_msgs/msg/LaserScan` |
| `/odom` | GZ → ROS | `nav_msgs/msg/Odometry` |
| `/tf` | GZ → ROS | `tf2_msgs/msg/TFMessage` |
| `/joint_states` | GZ → ROS | `sensor_msgs/msg/JointState` |
| `/cmd_vel` | ROS → GZ | `geometry_msgs/msg/Twist` |

## 验收

```bash
ros2 launch nav_bot_gazebo sim.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
ros2 topic hz /scan /odom /tf
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 常见故障

- 只桥 `/scan` 忘桥 `/clock` → 全链路时间戳为 0，RViz 报 timestamp too old。
- 同时开 `ros2_control` 的 `diff_drive_controller` 和 gz `DiffDrive` → 两套 TF 打架、odom 抽搐。
- 插件 `wheel_separation` 与 URDF 不一致 → 走 1 m 只报 0.8 m，AMCL 永远收敛不好。
