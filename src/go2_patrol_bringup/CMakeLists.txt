cmake_minimum_required(VERSION 3.8)
project(go2_patrol_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# maps/ 与 rviz/ 由用户按需填充（建图产物、RViz 配置），
# 允许为空目录，缺失时跳过，保证全新 clone 也能构建。
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/maps)
  install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
endif()
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
