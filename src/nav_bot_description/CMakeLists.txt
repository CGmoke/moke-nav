cmake_minimum_required(VERSION 3.8)
project(nav_bot_description)

find_package(ament_cmake REQUIRED)

# 纯数据包：只安装资源，不编译任何节点。
# 阶段 1 填充 urdf/ meshes/ config/ launch/ 后无需改动本文件。
install(
  DIRECTORY urdf meshes config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
