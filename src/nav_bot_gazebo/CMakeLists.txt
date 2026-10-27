cmake_minimum_required(VERSION 3.8)
project(nav_bot_gazebo)

find_package(ament_cmake REQUIRED)

# 纯数据包：world / model / bridge 配置 / launch。
# 阶段 2 填充 worlds/ models/ config/ launch/ 后无需改动本文件。
install(
  DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
