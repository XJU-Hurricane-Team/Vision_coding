### 网站链接

[SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav:  Shenzhen MSU-BIT University PolarBear Team's Sentry Navigation Sim2Real  Package for RoboMaster2025. QQ group: 932119307](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav?tab=readme-ov-file)



### 功能包说明

```

├── fake_vel_transform                  # 虚拟速度参考坐标系，以应对云台扫描模式自旋，详见子仓库 README
├── ign_sim_pointcloud_tool             # 仿真器点云处理工具
├── livox_ros_driver2                   # Livox 驱动
├── loam_interface                      # point_lio 等里程计算法接口
├── pb_teleop_twist_joy                 # 手柄控制
├── pb2025_nav_bringup                  # 启动文件
├── pb2025_sentry_nav                   # 本仓库功能包描述文件
├── pb_omni_pid_pursuit_controller      # 路径跟踪控制器
├── point_lio                           # 里程计
├── pointcloud_to_laserscan             # 将 terrain_map 转换为 laserScan 类型以表示障碍物（仅 SLAM 模式启动）
├── sensor_scan_generation              # 点云相关坐标变换
├── small_gicp_relocalization           # 重定位
├── terrain_analysis                    # 距车体 4m 范围内地形分析，将障碍物离地高度写入 PointCloud intensity
└── terrain_analysis_ext                # 车体 4m 范围外地形分析，将障碍物离地高度写入 PointCloud intensity
```



### rmu_gazebo_simulator（配套的仿真包）

- colcon build

  - ```
    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

  - ```
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
    ```

- 启动仿真环境

  - ```
    ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
    ```

- 控制机器人移动（只有导航模式能使用）

  - ```
    ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3
    #根据提示进行输入，支持平移与自旋
    #速度可以自己调节
    ```

- 机器人云台

  - ```
    ros2 run rmoss_gz_base test_gimbal_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
    #根据提示进行输入，支持绝对角度控制
    ```

- 机器人射击

  - ```
    ros2 run rmoss_gz_base test_shoot_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
    #根据提示进行输入
    ```

- 切换仿真世界

  - 修改 [gz_world.yaml](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator/blob/main/rmu_gazebo_simulator/config/gz_world.yaml) 中的 `world`。当前可选: `rmul_2024`, `rmuc_2024`, `rmul_2025`, `rmuc_2025`



### pd2025_sentry_nav

- 导航

  - ```
    ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    world:=rmuc_2025 \
    slam:=False
    ```

- 建图

  - ```
    ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
    slam:=True
    ```

  - 用Nav2 Goal来跑图

  - ```
    ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1
    ```

- 两辆车

  - ```
    ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py \
    world:=rmul_2024 \
    robots:=" \
    red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
    blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
    "
    ```

- 可视化机器人模型（在ROS_WS里面）

  - ```
    ros2 launch pb2025_robot_description robot_description_launch.py
    ```

  - 查看TF树

  - ```
    ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static
    ```


- 建图指南

  - 启动建图模式

  - 保存初版地图

    - SLAM边建图边导航（**先不要 Ctrl+C 终止建图程序**，而是新开一个终端，运行以下命令保存地图）

      - ```
        ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>
        ```

    - pcd2pgm建图

      - 读取指定的.pcd文件

      - 使用 Pass Through 滤波器过滤点云

      - 使用 Radius Outlier 滤波器进一步处理点云

      - 将处理后的点云转换为占据栅格地图

      - 将转换后的地图发布到指定ROS话题上

      - ---

      - 启动pcd2pgm节点，可在RViz中预览滤波后点云和栅格地图

      - ```
        ros2 launch pcd2pgm pcd2pgm_launch.py
        ```

      - 保存栅格地图

      - ```
        ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>
        ```

        

