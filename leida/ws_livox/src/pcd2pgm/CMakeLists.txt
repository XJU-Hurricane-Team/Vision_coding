cmake_minimum_required(VERSION 3.5)
project(pcd2pgm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
# 新增：寻找yaml-cpp库
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  # 新增：添加yaml-cpp头文件目录
  ${YAML_CPP_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 编译原有可执行文件
add_executable(pcd2pgm src/pcd2pgm.cpp)
ament_target_dependencies(pcd2pgm
  rclcpp
  nav_msgs
  pcl_ros
  sensor_msgs
  std_msgs
  geometry_msgs
)
target_link_libraries(pcd2pgm ${PCL_LIBRARIES})

# 新增：编译pgm发布节点
add_executable(pgm_publisher src/pgm_publisher.cpp)
ament_target_dependencies(pgm_publisher
  rclcpp
  nav_msgs
)
# 关键修改：链接yaml-cpp库
target_link_libraries(pgm_publisher ${YAML_CPP_LIBRARIES})

# 安装可执行文件
install(TARGETS
  pcd2pgm
  pgm_publisher  # 新增：安装pgm_publisher
  DESTINATION lib/${PROJECT_NAME})
install(
    DIRECTORY launch save rviz
    DESTINATION share/${PROJECT_NAME}
  )
ament_package()
