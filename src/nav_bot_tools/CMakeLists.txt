cmake_minimum_required(VERSION 3.8)
project(nav_bot_tools)

find_package(ament_cmake REQUIRED)

# 阶段 5：评测脚本与自研插件填充后，把 scripts/ 的安装方式改成
#   install(PROGRAMS scripts/eval_nav.py DESTINATION lib/${PROJECT_NAME})
# 并在下方 ament_package() 前加入插件库的 add_library / pluginlib 导出。
install(
  DIRECTORY config scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
