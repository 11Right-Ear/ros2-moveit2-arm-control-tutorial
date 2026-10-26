cmake_minimum_required(VERSION 3.8)
project(my_robot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Declare the C++ standard explicitly so that the flag ends up in the compile
# command. clangd reads the compile command from compile_commands.json and
# otherwise falls back to its default standard, which breaks C++17 code such
# as std::variant and std::is_convertible_v.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(example_interfaces REQUIRED)

find_package(my_robot_interfaces REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)

ament_target_dependencies(
  test_moveit
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2
)

add_executable(commander_template src/commander_template.cpp)

ament_target_dependencies(
  commander_template
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
  geometry_msgs
  moveit_msgs
  tf2
  my_robot_interfaces
)

install(
  TARGETS
    test_moveit
    commander_template
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()