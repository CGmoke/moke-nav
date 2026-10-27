cmake_minimum_required(VERSION 3.8)
project(nav_bot_bringup)

find_package(ament_cmake REQUIRED)

# 纯数据包：全链路 launch + RViz 配置。
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
