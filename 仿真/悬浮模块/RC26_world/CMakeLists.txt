cmake_minimum_required(VERSION 3.8)
project(RC26_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 world 文件到 share/RC26_world 根目录 (对应 launch 文件中的查找路径)
install(
  FILES
  my_scene.world
  DESTINATION share/${PROJECT_NAME}
)

# 安装 models 文件夹到 share/RC26_world/models
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()