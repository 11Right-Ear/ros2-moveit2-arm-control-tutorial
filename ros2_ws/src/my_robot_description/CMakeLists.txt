cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# URDF -> SDF 转换脚本（Gazebo Classic 用，详见 scripts/urdf_to_sdf.py）
install(
  PROGRAMS scripts/urdf_to_sdf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
