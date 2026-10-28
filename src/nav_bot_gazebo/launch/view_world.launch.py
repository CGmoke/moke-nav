"""单独预览 world（不生成机器人），用于确认场景能否正常加载。

用法：
  ros2 launch nav_bot_gazebo view_world.launch.py
  ros2 launch nav_bot_gazebo view_world.launch.py world:=<绝对路径.sdf>
  ros2 launch nav_bot_gazebo view_world.launch.py gui:=false   # 只看服务端日志

注意：此处只起 gz sim，不涉及 ros_gz_bridge，因此没有 /clock、/scan 等话题。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_bot_gazebo')
    models_dir = os.path.join(pkg_share, 'models')
    worlds_dir = os.path.join(pkg_share, 'worlds')

    default_world = os.path.join(worlds_dir, 'small_house.world')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')

    # model:// 解析依赖此变量：包含 models 与 worlds，并保留调用者原有的搜索路径
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = models_dir + os.pathsep + worlds_dir
    if existing:
        resource_path += os.pathsep + existing

    gz_args = ['-r', '-v', verbose, world]

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),

        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='SDF world 文件绝对路径'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否启动 Gazebo GUI'),
        DeclareLaunchArgument(
            'verbose', default_value='3',
            description='gz sim 日志级别 1-4'),

        ExecuteProcess(
            cmd=['gz', 'sim'] + ['-s'] + gz_args,
            output='screen',
            condition=UnlessCondition(gui),
        ),
        ExecuteProcess(
            cmd=['gz', 'sim'] + gz_args,
            output='screen',
            condition=IfCondition(gui),
        ),
    ])
