import pyrealsense2 as rs
import numpy as np
import cv2

# 配置深度摄像头
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动摄像头
pipeline.start(config)

# 创建对齐对象
align_to = rs.stream.color
align = rs.align(align_to)

# 深度值映射的颜色表
colorizer = rs.colorizer()


# 鼠标点击回调函数
def mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # 判断点击位置是在RGB图像还是深度图像上
        if x < 640:  # RGB图像区域 (0-639)
            depth = depth_frame.get_distance(x, y)
            print(f"RGB图像点击位置 ({x}, {y}) 的距离为: {depth:.3f} 米")

            # 在彩色图像上标记点击位置和深度值
            cv2.circle(color_image, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(color_image, f"{depth:.3f}m", (x + 10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 在深度图像上标记点击位置
            cv2.circle(depth_color_image, (x, y), 5, (0, 0, 255), -1)
        else:  # 深度图像区域 (640-1279)
            # 将深度图像的坐标转换为对齐后的深度帧坐标
            depth_x = x - 640
            depth = depth_frame.get_distance(depth_x, y)
            print(f"深度图像点击位置 ({depth_x}, {y}) 的距离: {depth:.3f} 米")

            # 在彩色图像上标记对应位置和深度值
            cv2.circle(color_image, (depth_x, y), 5, (0, 0, 255), -1)
            cv2.putText(color_image, f"{depth:.3f}m", (depth_x + 10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 在深度图像上标记点击位置
            cv2.circle(depth_color_image, (x, y), 5, (0, 0, 255), -1)


# 创建窗口并设置鼠标回调
cv2.namedWindow('RealSense Camera')
cv2.setMouseCallback('RealSense Camera', mouse_click)

try:
    while True:
        # 等待获取一帧数据
        frames = pipeline.wait_for_frames()

        # 对齐深度帧到彩色帧
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # 将帧转换为numpy数组
        color_image = np.asanyarray(color_frame.get_data())

        # 将深度帧转换为彩色图像以便显示
        depth_color_frame = colorizer.colorize(depth_frame)
        depth_color_image = np.asanyarray(depth_color_frame.get_data())

        # 拼接彩色图像和深度图像
        combined_image = np.hstack((color_image, depth_color_image))

        # 添加标签
        cv2.putText(combined_image, 'RGB Image', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(combined_image, 'Depth Image', (650, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # 显示图像
        cv2.imshow('RealSense Camera', combined_image)

        # 按ESC退出
        key = cv2.waitKey(1)
        if key == 27:  # ESC键
            break

finally:
    # 停止管道并关闭窗口
    pipeline.stop()
    cv2.destroyAllWindows()