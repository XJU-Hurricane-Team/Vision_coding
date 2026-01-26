cmake_minimum_required(VERSION 3.8)
project(odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 创建odometry_listener可执行文件
add_executable(odometry_listener src/odometry_listener.cpp)
ament_target_dependencies(odometry_listener 
  rclcpp 
  nav_msgs 
  tf2 
  tf2_geometry_msgs
)

# 创建odometry_transform可执行文件
add_executable(odometry_transform src/odometry_transform.cpp)
ament_target_dependencies(odometry_transform
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
)


# 安装可执行文件（仅保留存在的目标）
install(TARGETS
  odometry_listener
  odometry_transform
  DESTINATION lib/${PROJECT_NAME})

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
