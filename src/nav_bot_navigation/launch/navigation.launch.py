"""阶段 5 导航启动：定位（map_server + AMCL + EKF） + Nav2 导航节点组。

前置：另一个终端已经在跑仿真
    ros2 launch nav_bot_gazebo sim.launch.py

然后：
    ros2 launch nav_bot_navigation navigation.launch.py

TF 链（两套 lifecycle_manager 各自负责一段，谁也不抢谁的发布权）：
    map --amcl--> odom --ekf--> base_footprint --rsp--> base_link --> lidar_link
★ 本文件不启动 slam_toolbox：它和 AMCL 都会发 map->odom，TF 打架、车瞬移。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context, *args, **kwargs):
    """运行期解析路径：get_package_share_directory 不能与 LaunchConfiguration 拼。"""
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    pkg_share = get_package_share_directory('nav_bot_navigation')

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    loc_launch = os.path.join(pkg_share, 'launch', 'localization.launch.py')
    rviz_config = os.path.join(pkg_share, 'config', 'navigation.rviz')

    if not os.path.exists(params_file):
        raise FileNotFoundError(f"params_file {params_file} not found")
    if not os.path.exists(loc_launch):
        raise FileNotFoundError(f"loc_launch {loc_launch} not found")
    if not os.path.exists(rviz_config):
        raise FileNotFoundError(f"rviz_config {rviz_config} not found")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(loc_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'use_rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_localization')),
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── controller_server：出 /cmd_vel_nav（原始指令），交给 smoother 平滑 ──
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    # ── velocity_smoother：订阅 cmd_vel_nav，发布到 /cmd_vel 供 Gazebo 消费 ──
    #    /cmd_vel 因此只有一个发布者，避免原始指令与平滑指令叠加造成抖动
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings + [
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    # ── smoother_server：路径平滑（NavigateThroughPoses 的 BT 会调 SmoothPath）──
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    # ── waypoint_follower：多航点巡逻（也是 lifecycle 节点，挂进 node_names）──
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    # ── 第二套 lifecycle_manager：只管导航节点，与定位那套互不影响 ──
    #    node_names 必须与上面各 Node 的 name 逐字一致，顺序即 activate 顺序
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'waypoint_follower',
                'bt_navigator',
                'velocity_smoother',
            ]},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return [
        LogInfo(msg=f"[INFO] params_file = {params_file}"),
        localization,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        waypoint_follower,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager_navigation,
        rviz,
        LogInfo(
            msg='[SKIP] localization 已关闭 (use_localization := false)',
            condition=UnlessCondition(LaunchConfiguration('use_localization')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时钟（Gazebo 必须为 true）'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='lifecycle_manager 自动 configure+activate'),
        DeclareLaunchArgument(
            'use_localization', default_value='true',
            description='是否同时启动 map_server + AMCL + EKF'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz（加载 navigation.rviz）'),
        OpaqueFunction(function=setup),
    ])
