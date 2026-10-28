cmake_minimum_required(VERSION 3.8)
project(nav_bot_gazebo)

find_package(ament_cmake REQUIRED)

# 纯数据包：world / model / bridge 配置 / launch。
# photos/ 是 aws_robomaker_residential 系列模型 DAE 中 ../../../photos 的相对纹理路径目标，
# 必须与 models/ 同级安装，否则 gz GUI 会报 Unable to find texture。
install(
  DIRECTORY worlds models photos config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
