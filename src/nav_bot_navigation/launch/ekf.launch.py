"""robot_localization EKF 独立启动文件。

职责：只做一件事 —— 起一个 ekf_node，融合轮式里程计(/odom) + IMU(/imu/data)，
       并成为 odom -> base_footprint 的**唯一**发布者。

单独运行：
    ros2 launch nav_bot_navigation ekf.launch.py

被 slam.launch.py / navigation.launch.py 用 IncludeLaunchDescription 引入（use_ekf 开关控制）。

★ TF 单一来源约束（阶段三踩过的坑，务必保持）：
   gazebo.xacro 里 DiffDrive 插件的 <tf_topic> 已改道 /tf_gz_odom（且未被 gz bridge 桥接），
   所以 Gazebo 侧不发布 odom->base_footprint；只有本文件的 ekf_node 发。
   若再多一个发布者（例如 AMCL 同时 publish odom tf、或另一个 ekf_node 实例），
   TF 打架 → odom 抽搐、建图糊掉。
   验证：ros2 run tf2_tools view_frames 只应看到 ekf_filter_node 连着 odom -> base_footprint。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # 参数文件路径：默认 config/ekf.yaml，允许外部覆盖
    # ★ 参数名必须是 ekf_params_file，不能叫 params_file：
    #   DeclareLaunchArgument 是全局作用域的，父 launch（如 localization.launch.py）
    #   里的 params_file=nav2_params.yaml 会沿用到 Include 进来的子 launch，
    #   覆盖掉这里的默认值 —— 症状就是 EKF 加载了 nav2 的 yaml、odom0/imu0 全空、
    #   /odometry/filtered 一条都不发。
    params_file = LaunchConfiguration('ekf_params_file').perform(context)

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        # EKF 挂掉不要静默：崩溃会直接导致 odom->base_footprint 断链
        respawn=True,
        respawn_delay=2.0,
    )

    return [ekf]


def generate_launch_description():
    nav_share = get_package_share_directory('nav_bot_navigation')
    default_params = os.path.join(nav_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_params_file', default_value=default_params,
            description='robot_localization 参数文件（默认 config/ekf.yaml）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟（Gazebo 必须为 true）'),

        OpaqueFunction(function=launch_setup),
    ])
