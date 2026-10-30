"""阶段 4 定位启动：map_server + AMCL + EKF（不含 Nav2 导航节点）。

前置：另一个终端已经在跑仿真
    ros2 launch nav_bot_gazebo sim.launch.py

然后：
    ros2 launch nav_bot_navigation localization.launch.py

预期：RViz 里粒子云（PoseArray）3 s 内收敛成一小团，RobotModel 贴在 /map 上。

TF 链（本文件只产生中间一段）：
    map --amcl--> odom --ekf--> base_footprint --rsp--> base_link --> lidar_link
★ 绝不在此链路启动 slam_toolbox：它和 AMCL 都会发 map->odom，TF 打架、车瞬移。
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
    """运行期解析路径（get_package_share_directory 不能直接和 LaunchConfiguration 拼）。"""
    nav_share = get_package_share_directory('nav_bot_navigation')

    params_file = LaunchConfiguration('params_file').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    rviz_config = LaunchConfiguration('rvizconfig').perform(context)
    autostart = LaunchConfiguration('autostart')

    # nav2 节点习惯上把 /tf 写成相对名，namespace 非空时才能被正确重映射
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'yaml_filename': map_yaml}],
        remappings=remappings,
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file],
        remappings=remappings,
    )

    # map_server 必须先 configure 完（地图进 /map）AMCL 才有图可匹配；
    # lifecycle_manager 按 node_names 顺序逐个 configure+activate。
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']},
        ],
    )

    # EKF：odom->base_footprint 的唯一发布者，和定位必须同生共死
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'ekf.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_localization',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return [map_server, amcl, lifecycle_manager, ekf_launch, rviz]


def generate_launch_description():
    nav_share = get_package_share_directory('nav_bot_navigation')

    default_params = os.path.join(nav_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(nav_share, 'maps', 'small_house.yaml')
    default_rviz = os.path.join(nav_share, 'config', 'localization.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 参数文件（含 map_server / amcl 小节）'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='地图 yaml 绝对路径'),
        DeclareLaunchArgument(
            'rvizconfig', default_value=default_rviz,
            description='RViz 配置路径'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description='是否启动 robot_localization EKF（odom->base_footprint 发布者）'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='lifecycle_manager 自动 configure+activate'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟（Gazebo 必须为 true）'),

        OpaqueFunction(function=launch_setup),
    ])
