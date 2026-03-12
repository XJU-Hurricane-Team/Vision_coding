import pyrealsense2 as rs
import numpy as np
import cv2

# 配置深度摄像头
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动摄像头
profile = pipeline.start(config)

# 获取彩色相机的内参（用于坐标转换）
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# 创建对齐对象（将深度帧对齐到彩色帧）
align_to = rs.stream.color
align = rs.align(align_to)

# 深度值映射的颜色表
colorizer = rs.colorizer()


# 鼠标点击回调函数（输出调整后坐标系的三维坐标）
def mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # 定义坐标转换函数：将原始相机坐标转换为 (右X, 前Y, 上Z)
        def convert_coords(original_xyz):
            # 原始坐标：(右X, 下Y, 前Z)
            # 目标坐标：(右X, 前Y, 上Z) → Y和Z互换并修正Y方向
            return (
                original_xyz[0],       # X保持不变（右为正）
                original_xyz[2],       # Y改为原始Z（前为正）
                -original_xyz[1]       # Z改为原始Y的负值（上为正）
            )

        if x < 640:  # RGB图像区域 (0-639)
            depth = depth_frame.get_distance(x, y)
            # 原始三维坐标（相机坐标系：右X，下Y，前Z）
            original_xyz = rs.rs2_deproject_pixel_to_point(intr, [x, y], depth)
            # 转换为目标坐标系
            xyz = convert_coords(original_xyz)
            print(f"RGB图像点击位置 ({x}, {y}) 的三维坐标: ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) 米")
            print(f"  说明：X(右), Y(前), Z(上)")
            print("------------------------")

            # 在彩色图像上标记点击位置和三维坐标
            cv2.circle(color_image, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(color_image, f"X:{xyz[0]:.2f}", (x + 10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(color_image, f"Y:{xyz[1]:.2f}", (x + 10, y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(color_image, f"Z:{xyz[2]:.2f}", (x + 10, y + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 在深度图像上标记对应位置
            cv2.circle(depth_color_image, (x, y), 5, (0, 0, 255), -1)

        else:  # 深度图像区域 (640-1279)
            depth_x = x - 640  # 转换为深度图像的像素坐标
            depth = depth_frame.get_distance(depth_x, y)
            # 原始三维坐标（相机坐标系：右X，下Y，前Z）
            original_xyz = rs.rs2_deproject_pixel_to_point(intr, [depth_x, y], depth)
            # 转换为目标坐标系
            xyz = convert_coords(original_xyz)
            print(f"深度图像点击位置 ({depth_x}, {y}) 的三维坐标: ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) 米")
            print(f"  说明：X(右), Y(前), Z(上)")
            print("------------------------")

            # 在彩色图像上标记对应位置和三维坐标
            cv2.circle(color_image, (depth_x, y), 5, (0, 0, 255), -1)
            cv2.putText(color_image, f"X:{xyz[0]:.2f}", (depth_x + 10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(color_image, f"Y:{xyz[1]:.2f}", (depth_x + 10, y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(color_image, f"Z:{xyz[2]:.2f}", (depth_x + 10, y + 40),
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

        # 对齐深度帧到彩色帧（确保像素坐标对应）
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
