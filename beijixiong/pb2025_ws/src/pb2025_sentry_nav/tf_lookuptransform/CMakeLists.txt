cmake_minimum_required(VERSION 3.8)
project(tf_lookuptransform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------ Standered ------------

# 编译核心库
add_library(${PROJECT_NAME}_core SHARED
  src/tf_lookuptransform.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core PUBLIC
  rclcpp
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
  tf2
)

add_executable(tf_lookuptransform_node    # 编译成可执行节点
  src/getpose.cpp  
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)


target_link_libraries(tf_lookuptransform_node
  ${PROJECT_NAME}_core      # 将库链接到对应节点
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

install(TARGETS
  ${PROJECT_NAME}_core
  tf_lookuptransform_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}_core
)

ament_package()
