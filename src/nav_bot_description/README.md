
# nav_bot_description — 模型层

机器人的"物理定义"：只描述几何、惯量、关节、传感器位姿与外观网格，不含任何控制逻辑。

当前版本：**四轮差速版（4WD）· 外观 v2.5**
依据《轮式机器人建模规范》v1.5 + `config/robot_params.yaml` v2.0，并叠加外形审查结论（见下"关键约束"）。

## 整机概要

| 项目          | 数值                                                                    |
| ------------- | ----------------------------------------------------------------------- |
| 驱动方式      | 4WD 差速（lf/rf/lb/rb 四轮独立驱动，同侧同速）                          |
| 轮距 / 轴距   | 0.48 m（y=±0.24）/ 0.52 m（x=±0.26）                                  |
| 轮半径 / 轮宽 | 0.10 m（Ø200）/ 0.06 m                                                 |
| 底盘          | 520×400×120 mm，离地间隙 0.06 m                                       |
| 整机质量      | ≈16.3 kg（不含机械臂；电池并入 base_link）                             |
| 传感器        | 激光雷达（扫描面离地 0.28 m）、IMU、深度相机（俯仰 −15°）、RGB 摄像头 |
| 拓扑          | 11 links · 10 joints（4 continuous + 6 fixed）                         |
| 仿真目标      | Gazebo Harmonic（gz-sim 8）+ ros_gz，本机无 Gazebo Classic              |

## 目录职责

| 路径                             | 放什么                                                                                              | 状态 |
| -------------------------------- | --------------------------------------------------------------------------------------------------- | ---- |
| `urdf/nav_bot.urdf.xacro`      | 顶层装配文件，聚合所有子 xacro                                                                      | 已建 |
| `urdf/base.xacro`              | `base_link` 车体 + `platform_link` 顶板 + `arm_base_link` 机械臂基座                          | 已建 |
| `urdf/wheels.xacro`            | 四驱动轮 lf/rf/lb/rb（`continuous`，绕 +y，共用 wheel 宏）                                        | 已建 |
| `urdf/sensors.xacro`           | `lidar_link` / `imu_link` / `depth_camera_link`（隐藏式） / `camera_link`                   | 已建 |
| `urdf/gazebo.xacro`            | gz-sim 系统插件：DiffDrive（4 关节）、JointStatePublisher；传感器：`gpu_lidar` / IMU / RGBD / RGB | 已建 |
| `urdf/common/properties.xacro` | 全部尺寸/质量/位姿参数（`<xacro:property>`），单一样源                                            | 已建 |
| `urdf/common/inertia.xacro`    | 盒/圆柱/球/驱动轮惯性张量宏                                                                         | 已建 |
| `urdf/common/materials.xacro`  | 颜色材质（机身白/底盘深灰/琥珀/轮胎黑/传感器蓝/青灯带/玻璃窗）                                      | 已建 |
| `meshes/`                      | 外观网格：每 link 一组 `.dae`（含材质色）+ `.stl`（备份），四轮共用 `wheel.*`                 | 已建 |
| `config/display.rviz`          | RViz 预览配置                                                                                       | 已建 |
| `config/robot_params.yaml`     | 参数表 v2.0 副本（资料库原始文件的机读源）                                                          | 已建 |
| `launch/display.launch.py`     | RViz 预览（robot_state_publisher + jsp_gui + rviz2）                                                | 已建 |

## mesh 外观说明

- visual 引用 `package://nav_bot_description/meshes/*.dae`；**collision 一律保持简化几何体**（规范约束，禁止复用外观 mesh）。
- 网格由脚本按 `properties.xacro` 参数程序化生成（盒/圆柱组合，含细分），改参数后需重新生成并重新 build。
- 网格生成约定（避免渲染伪影）：
  - 部件间禁止共面接触：要么完全嵌入 1~2 mm，要么完全坐在外面（灯带、玻璃窗、法兰、镜头片均已按此处理）；
  - 大三角形需细分（max_edge 6 cm），否则 painter 算法排序会遮挡相邻部件。

## 关键约束（踩过的坑）

- 每个 link 必须有非零 `inertial`，质量为 0 会让模型在 Gazebo 里乱飞。
  例外：`base_footprint` 是零质量根 link，靠固定关节合并进 `base_link`，只提供 TF。
- `collision` 必须用简化几何体，禁止直接复用高精度 mesh。
- 四轮 `axis` 统一为 +y（REP-103），差速插件按关节名区分左右、由控制器处理转向符号。
- `base_link` 原点位于地面（轮心 z = 轮半径 R），其父帧 `base_footprint` 与它重合，否则 costmap 中心偏移。
- `wheel_separation`(0.48) / `wheel_radius`(0.10) 以 `properties.xacro` 为准，阶段 2 插件必须抄同一组数值。
- 驱动轮摩擦 mu ≥ 0.9，否则四轮互顶会让里程计失真（yaml v2.0 contact）。
- 深度相机机身藏于车头内，Gazebo 传感器 `<pose>` 前移到面板外并带 −15° 俯仰，否则虚拟相机被自家前壁遮挡。
- 本机为 **Gazebo Harmonic / gz-sim 8**（无 gazebo_ros_pkgs），`<gazebo>` 扩展只能用 `gz-sim-*-system` 插件与 SDF 传感器标签。
- 外形审查结论（覆盖 yaml 的部分，改回前先出三视图确认）：底盘 520×400（轮子外露的 rover 造型）；摄像头平嵌顶板前缘、无外伸杆；深度相验收

```bash
# 语法与拓扑
xacro src/nav_bot_description/urdf/nav_bot.urdf.xacro > /tmp/nav_bot.urdf
check_urdf /tmp/nav_bot.urdf                      # 期望：root base_link，8 个直接子节点

# RViz 外观预览（应与 mesh 渲染一致）
ros2 launch nav_bot_description display.launch.py

# TF 树（期望 base_link → 4 轮 / platform / lidar / imu / depth_camera）
ros2 run tf2_tools view_frames

# 质量核对：sum(mass) ≈ 16.3
```
