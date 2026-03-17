# 点云显示处理

- **fast_lio**
  - mid360.yaml
    - filter_size_surf - 体素滤波尺寸（网格越小，点云越密集，细节越清晰）（0.1，0.2）
    - filter_size_map - 点云采样倍率（可以调节点云的稠密度）（0.2）
    - dense_publish_en - 全局点云稠密度（ture）
  - navigation2.launch.py
    - global_leaf_size - GICP匹配叶子尺寸（0.5）
    - registered_leaf_size - GICP匹配叶子尺寸（0.5）



# 小车模型抖动

- **small_gicp_relocalization**
  - small_gicp_relocalization_launch.py
    - global_leaf_size（0.1，0.15）
    - registered_leaf_size（0.1，0.15）
    - num_neighbors - 协方差估计的邻居数量（临近的点越少，算出来的局部平面法向量会非常抖）（20，30）
    - max_dist_sq - 收紧最大匹配距离（强迫算法只对其足够近、置信度高的点）（0.25，0.5）



# Nav2的雷达盲区

- **fishbot_navigation2**
  - nav2_params.yaml
    - robot_radius - 机器人半径（0.20）
    - inflation_radius - 膨胀半径（0.5）
- **fast_lio**
  - mid360.yaml
    - blind - 探测距离（0.1）



# 实时物体扫描（卡帧感）

- **fishbot_navigation2**
  - nav2_params.yaml
    - update_frrequency - 计算频率（10.0（`local_costmap`）5.0（`global_costmap`））
    - publish_frequency - Rviz显示频率（10.0（`local_costmap`）5.0（`global_costmap`））
    - raytrace_max_range - 清除距离（15.0）
    - obstacle_max_range - 障碍物标记距离（14.0）



# 话题及参数

- **统一坐标系**

  - /odom_map

  - ```
    具体数据格式：
    pose:
      position:
        x: 0.25722039559935195
        y: 1.930963034355335
        z: -0.0011968423977814363
      orientation:
        x: 0.001982200830151646
        y: -0.0021401778668381426
        z: -0.0007883594037919605
        w: 0.999995434493587
    ```

  - ```
    定义 2 个【统一地图原点】
        map_origins = {
            'map_1': { # 场地一的地图原点
                'map_origin_x': -5.543, 'map_origin_y': -5.998, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0,
            },
            'map_2': { # 默认全零地图原点
                'map_origin_x': -5.548, 'map_origin_y': -0.007, 'map_origin_z': 0.0, 'map_origin_qz': 0.0, 'map_origin_qw': 1.0,
            }
        }
    ```

- **规划路径导航**

  - /nav_speed_heading_data

  - ```
    linear_velocity: 0.16421052631578947
    angular_velocity: 0.4736842105263157
    yaw: 0.012978140341457415
    ```
    
  - ```bash
    ros2 topic echo /nav_speed_heading_data
    ```
    
  - 因为是自定义话题类型，是用上面命令查看或者订阅话题得先source环境
  

### 帧率调整

- `fast_lio`
  - `start.launch.py`
    - `publish_freq`（20hz）

### 小车速度调整

- `fishbot_navigation2`
  - `nav2_params.yaml`（y的为0）
    
    - `max_vel_x`（最大支线速度）（直行）（1.0）
    
    - `max_vel_y`（横向速度）（0.0）
    
    - `max_speed_xy`（和速度）（改成x的）
    
    - `max_vel_theta`（最大角速度）（1.0）
    
    - `acc_lim_x`（直线最大加速度）（0.5）
    
    - `acc_lim_y`（横向加速度）（0.0）
    
    - `acc_lim_theta`（旋转最大加速度）（0.5）
    
    - `decel_lim_x`（直线最大减速度）（取负值）
    
    - `decel_lim_theta`（旋转最大减速度）（取负值）
    
    - `trans_stopped_velocity`（停止阈值）（0.05）
    
    - `velocity_smoother`（速度平滑器）
    - `max_velocity`（对着填下去）
      
    - `max_accel`（对着填下去）
      
    - `max_decel`（对着填下去）

### 小车的容忍度

- `nav2_params.yaml`
  - `failure_tolerance`（控制器容错）（1.0）
  - `movement_time_allowance`（进程检查器）（20.0）



### 小车位置参数(预置好了，就是挑用哪一个序号)

- `fishbot_navigation2`
  - `navigation2.launch.py`
  - 小车在`rviz`中初始化的位置
- `nav_speed_heading`
  - `tracker_node.py`
  - 小车导航给出目标点的信息（速度和方向）
  - `preset_nav_node.py`
  - 调用`Nav2_Goal`给出小车的目标点（不用进行坐标变换）
- `odometry`
  - `odometry_transform_math_launch.py`
  - 小车当前位置信息 
