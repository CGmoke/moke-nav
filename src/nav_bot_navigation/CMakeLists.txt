cmake_minimum_required(VERSION 3.8)
project(nav_bot_navigation)

find_package(ament_cmake REQUIRED)

# 纯数据包：params yaml / 地图 / 行为树 / launch。
# 阶段 3-5 填充 config/ maps/ behavior_trees/ launch/ 后无需改动本文件。
install(
  DIRECTORY config maps behavior_trees launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
