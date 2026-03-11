cmake_minimum_required(VERSION 3.8)
project(chassis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS2 构建依赖
find_package(ament_cmake REQUIRED)

# 安装目录，确保这些文件夹能被 ROS2 环境找到
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
