import cv2

# 打开摄像头（0代表默认摄像头）
cap = cv2.VideoCapture(0)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 设置摄像头参数（可选）
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置宽度
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置高度
cap.set(cv2.CAP_PROP_FPS, 30)  # 设置帧率

print("按 'q' 键退出，按 's' 键保存图片")

while True:
    # 读取一帧图像
    ret, frame = cap.read()

    # 如果读取成功，ret为True
    if not ret:
        print("无法获取图像")
        break

    # 显示图像
    cv2.imshow('Camera', frame)

    # 等待按键
    key = cv2.waitKey(1) & 0xFF

    # 按'q'退出
    if key == ord('q'):
        break
    # 按's'保存图片
    elif key == ord('s'):
        cv2.imwrite('captured_image.jpg', frame)
        print("图片已保存")

# 释放摄像头
cap.release()
cv2.destroyAllWindows()