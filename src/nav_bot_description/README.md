# nav_bot_description — 模型层（阶段 1）

机器人的"物理定义"，只描述几何、惯量、关节、传感器位姿，不含任何控制逻辑。

## 目录职责

| 路径 | 放什么 | 状态 |
|---|---|---|
| `urdf/nav_bot.urdf.xacro` | 顶层装配文件，聚合所有子 xacro | 待建 |
| `urdf/base.xacro` | `base_footprint` / `base_link` 车体 | 待建 |
| `urdf/wheels.xacro` | 左右驱动轮（`continuous`）+ 万向轮（`revolute`） | 待建 |
| `urdf/sensors.xacro` | `laser_link`（可选 IMU / 相机） | 待建 |
| `urdf/gazebo.xacro` | `<gazebo>` 扩展：`DiffDrive` 插件、摩擦、`gpu_lidar` | 待建 |
| `urdf/common/properties.xacro` | 所有尺寸/质量/轮距参数的 `<xacro:property>` | 待建 |
| `urdf/common/inertia.xacro` | 长方体/圆柱惯性张量宏 | 待建 |
| `urdf/common/materials.xacro` | 颜色材质定义 | 待建 |
| `meshes/` | `.stl` / `.dae` 外观模型（无则用基本几何体） | 空 |
| `config/` | `display.rviz` 等 RViz 配置 | 待建 |
| `launch/` | `display.launch.py`（RViz + robot_state_publisher + jsp_gui） | 待建 |

## 关键约束（踩过的坑）

- 每个 link 必须有 `inertial`，质量为 0 会让模型在 Gazebo 里乱飞。
- `collision` 必须用简化几何体，禁止直接复用高精度 mesh。
- 左右轮 `axis` 方向相反，写成一致会原地打转。
- `base_footprint` 高度 = 轮半径，否则车陷地、costmap 中心偏移。
- `wheel_separation` / `wheel_radius` 在这里定，阶段 2 的插件必须抄同一组数值。

## 验收

```bash
check_urdf <(xacro src/nav_bot_description/urdf/nav_bot.urdf.xacro)
ros2 launch nav_bot_description display.launch.py
ros2 run tf2_tools view_frames        # 期望 base_footprint → base_link → {轮, caster, laser_link}
```
