# nav_bot_description — 模型层（阶段 1）

机器人的"物理定义"，只描述几何、惯量、关节、传感器位姿，不含任何控制逻辑。
当前版本：**四轮差速版（4WD）**，依据《轮式机器人建模规范》v1.5 + `config/robot_params.yaml` v2.0。

## 目录职责

| 路径 | 放什么 | 状态 |
|---|---|---|
| `urdf/nav_bot.urdf.xacro` | 顶层装配文件，聚合所有子 xacro（11 links · 10 joints） | 已建 |
| `urdf/base.xacro` | `base_link` 车体 + `platform_link` 顶板（含角柱）+ `arm_base_link` | 已建 |
| `urdf/wheels.xacro` | 四驱动轮 lf/rf/lb/rb（`continuous`，同侧同速） | 已建 |
| `urdf/sensors.xacro` | `lidar_link` / `imu_link` / `depth_camera_link` / `camera_link` | 已建 |
| `urdf/gazebo.xacro` | `<gazebo>` 扩展：gz-sim DiffDrive（4 关节）、摩擦、`gpu_lidar` 等 | 已建 |
| `urdf/common/properties.xacro` | 所有尺寸/质量/轮距参数的 `<xacro:property>` | 已建 |
| `urdf/common/inertia.xacro` | 长方体/圆柱/球惯性张量宏 | 已建 |
| `urdf/common/materials.xacro` | 颜色材质定义 | 已建 |
| `meshes/` | `.stl` / `.dae` 外观模型（无则用基本几何体） | 空 |
| `config/` | `display.rviz` + `robot_params.yaml`（参数源副本） | 已建 |
| `launch/` | `display.launch.py`（RViz + robot_state_publisher + jsp_gui） | 已建 |

## 关键约束（踩过的坑）

- 每个 link 必须有 `inertial`，质量为 0 会让模型在 Gazebo 里乱飞。
- `collision` 必须用简化几何体，禁止直接复用高精度 mesh。
- 左右轮 `axis` 统一为 +y（REP-103），差速插件按关节名区分左右、由控制器处理转向符号。
- base_link 原点位于地面（轮心 z = 轮半径 R），充当 base_footprint，否则 costmap 中心偏移。
- `wheel_separation`(0.48) / `wheel_radius`(0.10) 在这里定，阶段 2 的插件必须抄同一组数值。
- 四轮差速：驱动轮 mu ≥ 0.9，否则前后轮互顶会让里程计失真（yaml v2.0 contact）。
- 外形修正约定：底盘宽 400 mm（轮子外露）、顶板带角柱、摄像头短支架（离地 0.33 m）。
  参数表 yaml 的 420×500 底盘与 0.30 m 摄像头支架按外形审查结论覆盖，改回前先三视图确认。

## 验收

```bash
check_urdf <(xacro src/nav_bot_description/urdf/nav_bot.urdf.xacro)
ros2 launch nav_bot_description display.launch.py
ros2 run tf2_tools view_frames        # 期望 base_link → {4 轮, platform, lidar, imu, depth_camera}
```
