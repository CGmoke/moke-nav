"""阶段 3 建图启动：slam_toolbox 在线异步 SLAM + RViz 可视化。

前置：另一个终端已经在跑仿真
    ros2 launch nav_bot_gazebo sim.launch.py

然后：
    ros2 launch nav_bot_navigation slam.launch.py

再开第三个终端遥控建图（速度压到 0.2~0.3 m/s）：
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true

地图满意后保存（第四个终端）：
    ros2 run nav2_map_server map_saver_cli -f src/nav_bot_navigation/maps/small_house

本文件只做三件事，不重复启动仿真与 robot_state_publisher：
  1. 引入 slam_toolbox 官方 online_async_launch.py，但把参数文件换成我们的 yaml
  2. 起 RViz（固定坐标系 map，显示 Map / LaserScan / RobotModel）
  3. 保证 use_sim_time 一路传下去（缺一个节点就会 TF 时间戳错乱）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """在运行期解析路径（get_package_share_directory 不能用 LaunchConfiguration 拼接）。"""
    nav_share = get_package_share_directory('nav_bot_navigation')
    slam_share = get_package_share_directory('slam_toolbox')

    params_file = LaunchConfiguration('slam_params_file').perform(context)
    rviz_config = LaunchConfiguration('rvizconfig').perform(context)

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': params_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    # ── robot_localization EKF：融合轮式里程计 + IMU ──
    # 4WD 滑移转向下轮式里程计旋转虚报（IMU 真值 34.6° vs 里程计 45.4°，1.31×，
    # 详见 docs/阶段三任务总结.md 与 experiments.md EXP-001），
    # 旋转量改由陀螺仪提供（见 config/ekf.yaml 头部注释）。
    # EKF 是 odom->base_footprint 的唯一发布者（DiffDrive 的 TF 已改道 /tf_gz_odom）。
    # 代码抽到了 launch/ekf.launch.py，这里只做 Include，use_ekf:=false 时整段跳过。
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'ekf.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    actions = [slam_launch, ekf_launch]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    actions.append(rviz)

    return actions


def generate_launch_description():
    nav_share = get_package_share_directory('nav_bot_navigation')

    default_params = os.path.join(nav_share, 'config', 'slam_toolbox_params.yaml')
    default_rviz = os.path.join(nav_share, 'config', 'slam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_params,
            description='slam_toolbox 参数文件（阶段 4 定位可换成 localization 版）'),
        DeclareLaunchArgument(
            'rvizconfig', default_value=default_rviz,
            description='RViz 配置路径'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='是否启动 robot_localization EKF（融合 IMU 修正 4WD 里程计漂移）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟（Gazebo 必须为 true）'),

        OpaqueFunction(function=launch_setup),
    ])
