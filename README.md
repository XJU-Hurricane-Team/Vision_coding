# 2026R2_Vision

### 代码运行的效果

- **fishbot_camera**（深度摄像头与机械臂）
  - 服务通信和话题通信
    - 服务通信（每隔2秒发送0，返回机械臂状态true或false），话题通信（当返回ture，就不会发送请求并且当识别物体达到要求就发送坐标，当返回true就持续等待）
  - 持续识别物体中心坐标
- **start**（ubuntu多线程运行程序）

---

### 运行代码的流程

- **fishbot_camera**
  - ros2 run fishbot_camera detector_node
  - 结合micro_ros
- **start**
  - python start.py
