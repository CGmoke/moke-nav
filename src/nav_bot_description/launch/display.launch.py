"""display.launch.py — RViz 预览模型（robot_state_publisher + joint_state_publisher_gui + rviz2）"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_bot_description')
    default_model = os.path.join(pkg_share, 'urdf', 'nav_bot.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'config', 'display.rviz')

    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='true',
        description='启动 joint_state_publisher_gui 滑条')
    rviz_arg = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz,
        description='RViz 配置文件路径')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', LaunchConfiguration('model')]),
            value_type=str,
        )
    }
    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model,
        description='URDF/Xacro 文件路径')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        use_gui_arg,
        rviz_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
