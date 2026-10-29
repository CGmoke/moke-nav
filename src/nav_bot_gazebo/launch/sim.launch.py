"""仿真总启动文件：加载小车到 Gazebo（Gazebo Harmonic / gz-sim）仿真世界。

流程：
  1. xacro 展开 nav_bot.urdf.xacro → robot_description
  2. 启动 robot_state_publisher（发布 TF，供 RViz / 导航使用）
  3. 启动 gz sim 并加载 small_house.world
  4. 通过 ros_gz_sim create 将小车 spawn 进世界
  5. 通过 ros_gz_bridge 建立话题桥接（/clock、/cmd_vel、/odom、/tf、
     /scan、/imu/data、相机与深度相机话题、/joint_states）

用法：
  ros2 launch nav_bot_gazebo sim.launch.py
  ros2 launch nav_bot_gazebo sim.launch.py world:=<绝对路径.sdf>
  ros2 launch nav_bot_gazebo sim.launch.py gui:=false          # 无界面仿真
  ros2 launch nav_bot_gazebo sim.launch.py x:=1.0 y:=0.5       # 自定义出生点

teleop 验证：
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
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
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_pkg_share = get_package_share_directory('nav_bot_gazebo')
    description_pkg_share = get_package_share_directory('nav_bot_description')

    models_dir = os.path.join(gazebo_pkg_share, 'models')
    worlds_dir = os.path.join(gazebo_pkg_share, 'worlds')
    default_world = os.path.join(worlds_dir, 'small_house.world')

    # model:// 解析依赖此变量：包含 models 与 worlds，并保留调用者原有的搜索路径。
    # 机器人 visual 用 package://nav_bot_description/meshes/*.dae，sdformat_urdf 会
    # 改写成 model://nav_bot_description/...，所以还必须加上该包 share 目录的上一级，
    # 否则 spawn 时报 Unable to find file。
    desc_share_parent = os.path.dirname(description_pkg_share)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = os.pathsep.join([desc_share_parent, models_dir, worlds_dir])
    if existing:
        resource_path += os.pathsep + existing

    xacro_file = os.path.join(description_pkg_share, 'urdf', 'nav_bot.urdf.xacro')

    # 参数
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # 机器人描述（xacro 展开，显式声明为字符串避免 YAML 解析报错）
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    # ── gz sim 服务端 + GUI（复用 view_world 的启动方式，自动加载 world） ──
    gz_args = ['-r', '-v', verbose, world]
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s'] + gz_args,
        output='screen',
        condition=UnlessCondition(gui),
    )
    gz_full = ExecuteProcess(
        cmd=['gz', 'sim'] + gz_args,
        output='screen',
        condition=IfCondition(gui),
    )

    # ── spawn：从 robot_description 话题读取并生成小车 ──
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'nav_bot',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
        output='screen',
    )

    # ── ROS 侧节点 ──
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ── 话题桥接（gz 话题在 URDF 的 gazebo.xacro 中定义） ──
    # 注意：gz-sim JointStatePublisher 发布在
    # /world/<world_name>/model/<model_name>/joint_state（gz.msgs.Model），
    # 本项目 world name 为 'default'、模型名为 'nav_bot'。
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # 差速驱动（DiffDrive）
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        # 关节状态
        '/world/default/model/nav_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        # 激光雷达
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        # IMU
        '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        # RGB 摄像头
        '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        # 深度相机（rgbd_camera）
        '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
    ]
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        parameters=[{'use_sim_time': use_sim_time}],
        # 关节状态重映射为 ROS 标准 /joint_states，供 robot_state_publisher 使用
        remappings=[('/world/default/model/nav_bot/joint_state', '/joint_states')],
        output='screen',
    )

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
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='ROS 节点是否使用仿真时钟'),
        DeclareLaunchArgument('x', default_value='0.0',
                              description='小车出生点 x 坐标'),
        DeclareLaunchArgument('y', default_value='0.0',
                              description='小车出生点 y 坐标'),
        DeclareLaunchArgument('z', default_value='0.0',
                              description='小车出生点 z 坐标（base_link 原点在地面）'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='小车出生朝向（弧度）'),

        gz_server,
        gz_full,
        spawn,
        robot_state_publisher,
        bridge,
    ])
