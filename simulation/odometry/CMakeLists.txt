cmake_minimum_required(VERSION 3.8)
project(odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)          # 【关键】必须添加
find_package(tf2_geometry_msgs REQUIRED)

# 编译 odometry_transform
add_executable(odometry_transform src/odometry_transform.cpp)
ament_target_dependencies(odometry_transform
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros             # 【关键】必须添加
  tf2_geometry_msgs
)

# 编译 odometry_listener
add_executable(odometry_listener src/odometry_listener.cpp)
ament_target_dependencies(odometry_listener
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros             # 【关键】必须添加
  tf2_geometry_msgs
)

install(TARGETS
  odometry_transform
  odometry_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()