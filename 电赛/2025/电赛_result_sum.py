import cv2
import numpy as np
import enum
import math

DEBUG_MODE = False  # 调试模式显示中间过程

A4_REAL_WIDTH = 21.0  # A4纸实际宽度(cm)
A4_REAL_HEIGHT = 29.7  # A4纸实际高度(cm)
A4_REAL_AREA = A4_REAL_WIDTH * A4_REAL_HEIGHT  # A4纸实际面积(cm²)
MIN_CONTOUR_AREA = 1200  # 最小轮廓面积阈值（过滤噪声）
FOCAL_LENGTH = 587.5  # 摄像头焦距（使用默认值）


class ContourType(enum.Enum):
    Nothing = 1,
    Basic_shapes = 2,  # 合并正方形和三角形检测
    Basic_circle = 3,
    Extension_Two_and_One = 5,
    Extension_Rotate = 6,
    Distance_Measure = 7  # 单独测距离模式


contour_type = ContourType.Nothing

# 创建视频捕获对象
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)


### A4边框识别和测距
def calculate_pixel_width(contour):
    """使用最小外接接矩形计算短边"""
    rect = cv2.minAreaRect(contour)
    width, height = rect[1]
    distance = min(width, height)
    return distance  # 返回短边长度


def detect_a4_contour(image):
    """检测A4纸轮廓，返回轮廓、阈值图像和A4纸像素面积"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 自适应阈值处理 - 增强黑色边框
    thresh = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 77, 10
    )
    thresh_copy = thresh.copy()
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if DEBUG_MODE:
        print(f"检测到 {len(contours)} 个轮廓")
    # 筛选A4纸轮廓（最大轮廓+四边形判断）
    if contours:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        if DEBUG_MODE:
            print(f"检测到contours  {len(contours)} 个轮廓")
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if DEBUG_MODE:
                print(f"检测到 A4 轮廓 {area:.2f} 面积")
            if area < MIN_CONTOUR_AREA:
                continue

            # 多边形逼近
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            if DEBUG_MODE:
                print(f"len(approx){len(approx)}")
            # 四边形检测
            if len(approx) == 4:
                if DEBUG_MODE:
                    print(f"approx{approx}")
                return approx, thresh_copy, area  # 返回A4纸像素面积
    return None, thresh_copy, 0


def calculate_distance(focal_length, pixel_width):
    """计算目标物距离D"""
    return (focal_length * A4_REAL_WIDTH) / pixel_width


### 透视变换
def perspective_transform(image, contour):
    """将A4纸区域透视变换为标准矩形"""
    # 将轮廓点排序为（左上、右上、右下、左下）
    pts = contour.reshape(4, 2)
    rect = np.zeros((4, 2), dtype="float32")

    # 计算轮廓点中心
    center = np.mean(pts, axis=0)

    # 根据点与中心的相对位置排序
    for point in pts:
        if point[0] < center[0] and point[1] < center[1]:
            rect[0] = point  # 左上
        elif point[0] > center[0] and point[1] < center[1]:
            rect[1] = point  # 右上
        elif point[0] > center[0] and point[1] > center[1]:
            rect[2] = point  # 右下
        else:
            rect[3] = point  # 左下

    # 计算目标标矩形尺寸（保持A4比例）
    width = max(
        np.linalg.norm(rect[0] - rect[1]),
        np.linalg.norm(rect[2] - rect[3])
    )
    height = max(
        np.linalg.norm(rect[0] - rect[3]),
        np.linalg.norm(rect[1] - rect[2])
    )

    # 创建目标点
    dst = np.array([
        [0, 0],
        [width - 1, 0],
        [width - 1, height - 1],
        [0, height - 1]
    ], dtype="float32")

    # 计算变换矩阵
    M = cv2.getPerspectiveTransform(rect, dst)

    # 应用透视变换
    warped = cv2.warpPerspective(image, M, (int(width), int(height)))

    return warped


def calculate_rotation(contour):
    """计算形状的旋转角度（内部使用，不对外输出）"""
    rect = cv2.minAreaRect(contour)
    angle = rect[2]

    # 优化角度计算，确保范围在0-90度之间
    if angle < -45:
        angle += 90
    angle = abs(angle)
    if angle > 90:
        angle = 180 - angle

    return angle


### 形状检测函数
def detect_squares_in_a4(warped, a4_pixel_area):
    """检测正方形，使用面积比例例计算边长"""
    inverted_gray = cv2.bitwise_not(warped)
    _, binary_img = cv2.threshold(inverted_gray, 160, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    squares = []
    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) == 4 and is_square(approx):
            # 计算正方形像素面积（白色部分）
            square_pixel_area = cv2.contourArea(approx)
            if square_pixel_area < 2:
                continue

            # 核心：通过面积比例计算真实面积
            if a4_pixel_area > 0:
                area_ratio = square_pixel_area / a4_pixel_area
                square_real_area = A4_REAL_AREA * area_ratio
                square_real_side = math.sqrt(square_real_area)
            else:
                # 回退方案：原有比例方法
                square_real_side = calculate_square_size_using_a4_ratio(
                    calculate_side_length(approx), warped)

            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            squares.append({
                "type": "square",
                "size": square_real_side,
                "center": (cX, cY),
                "contour": approx  # 添加轮廓信息
            })

    return squares


def is_square(contour, angle_threshold=15, aspect_threshold=0.15):
    """验证是否为正方形（基于角度和边长比例）"""
    sides = []
    for i in range(4):
        pt1 = contour[i][0]
        pt2 = contour[(i + 1) % 4][0]
        sides.append(np.linalg.norm(pt2 - pt1))
    max_side, min_side = max(sides), min(sides)
    aspect_ratio = abs(max_side - min_side) / ((max_side + min_side) / 2)
    if aspect_ratio > aspect_threshold:
        return False
    for i in range(4):
        pt1 = contour[i][0]
        pt2 = contour[(i + 1) % 4][0]
        pt3 = contour[(i + 2) % 4][0]
        angle = calculate_angle(pt1, pt2, pt3)
        if not (85 <= angle <= 95):
            return False
    return True


def calculate_angle(p1, p2, p3):
    """计算三点形成的角度"""
    v1 = p1 - p2
    v2 = p3 - p2
    cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    # 处理数值精度问题
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    return np.degrees(np.arccos(cos_theta))


def calculate_side_length(contour):
    """计算正方形边长（像素素单位）"""
    sides = []
    for i in range(4):
        pt1 = contour[i][0]
        pt2 = contour[(i + 1) % 4][0]
        sides.append(np.linalg.norm(pt2 - pt1))
    avg_side = np.mean(sides)
    rect = cv2.minAreaRect(contour)
    min_side = min(rect[1])
    return (avg_side + min_side) / 2


def calculate_square_size_using_a4_ratio(pixel_side, warped_a4):
    """基于A4纸比例计算正方形实际尺寸的方法（作为备选）"""
    a4_pixel_height, a4_pixel_width = warped_a4.shape[:2]
    a4_pixel_ratio = a4_pixel_width / a4_pixel_height
    a4_real_ratio = A4_REAL_WIDTH / A4_REAL_HEIGHT  # 标准A4比例

    if a4_pixel_ratio > a4_real_ratio:  # 横放A4纸
        pixel_to_cm = A4_REAL_HEIGHT / a4_pixel_height  # 使用高度计算比例
    else:  # 竖放A4纸
        pixel_to_cm = A4_REAL_WIDTH / a4_pixel_width  # 使用宽度计算比例

    return pixel_side * pixel_to_cm


def detect_triangles_in_a4(warped):
    """检测三角形"""
    inverted = cv2.bitwise_not(warped)
    _, binary_img = cv2.threshold(inverted, 160, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    triangles = []
    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
        if len(approx) == 3 and is_equilateral_triangle(approx):
            side_length = calculate_triangle_side(approx)
            pixel_side = side_length
            cm_side = (pixel_side / warped.shape[1]) * A4_REAL_WIDTH

            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            triangles.append({
                "type": "triangle",
                "size": cm_side,
                "center": (cX, cY),
                "contour": approx  # 添加轮廓信息

            })
    return triangles


def is_equilateral_triangle(contour, angle_threshold=15, side_threshold=0.15):
    """验证等边三角形（基于角度和边长）"""
    points = contour.reshape(3, 2)
    sides = [
        np.linalg.norm(points[1] - points[0]),
        np.linalg.norm(points[2] - points[1]),
        np.linalg.norm(points[0] - points[2])
    ]
    max_side, min_side = max(sides), min(sides)
    aspect_ratio = abs(max_side - min_side) / ((max_side + min_side) / 2)
    if aspect_ratio > side_threshold:
        return False
    for i in range(3):
        a, b, c = points[i], points[(i + 1) % 3], points[(i + 2) % 3]
        angle = calculate_angle(a, b, c)
        if not (55 <= angle <= 65):
            return False
    return True


def calculate_triangle_side(contour):
    """计算三角形边长（三种方法融合）"""
    points = contour.reshape(3, 2)
    sides_direct = [
        np.linalg.norm(points[1] - points[0]),
        np.linalg.norm(points[2] - points[1]),
        np.linalg.norm(points[0] - points[2])
    ]
    avg_side = np.mean(sides_direct)
    (cx, cy), radius = cv2.minEnclosingCircle(contour)
    circum_radius = radius
    theoretical_side = circum_radius * np.sqrt(3)
    area = cv2.contourArea(contour)
    area_side = (4 * area / np.sqrt(3)) ** 0.5
    return (avg_side * 0.6 + theoretical_side * 0.3 + area_side * 0.1)


def detect_circle_in_a4(warped):
    """检测圆形"""
    inverted_gray = cv2.bitwise_not(warped)
    _, binary_img = cv2.threshold(inverted_gray, 160, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    MIN_AREA = 30
    MAX_AREA = 6000
    MIN_CIRCULARITY = 0.7
    MAX_CIRCULARITY = 1.3

    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue
        circularity = (4 * np.pi * area) / (perimeter ** 2)
        if (MIN_AREA <= area <= MAX_AREA and
                MIN_CIRCULARITY <= circularity <= MAX_CIRCULARITY):
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            diameter = 2 * radius
            pixel_diameter = diameter * circularity
            actual_diameter = (pixel_diameter / warped.shape[1]) * A4_REAL_WIDTH
            if actual_diameter < 9:
                continue
            return {
                "type": "circle",
                "size": actual_diameter,
                "center": (int(x), int(y))
            }
    return None


def detect_and_measure_squares(warped_a4, a4_pixel_area):
    """检测測并测量A4纸上的正方形，使用面积比例例计算边长"""
    # 确保输入转为灰度图
    if len(warped_a4.shape) == 3:
        gray = cv2.cvtColor(warped_a4, cv2.COLOR_BGR2GRAY)
    else:
        gray = warped_a4.copy()

    inverted_gray = cv2.bitwise_not(gray)
    # 阈值处理
    _, binary_img = cv2.threshold(inverted_gray, 150, 255, cv2.THRESH_BINARY)

    # 形态学操作
    kernel = np.ones((2, 2), np.uint8)
    closed1 = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel, iterations=1)
    opened = cv2.morphologyEx(closed1, cv2.MORPH_OPEN, kernel, iterations=1)

    # 寻找轮廓
    contours, _ = cv2.findContours(opened, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    squares = []

    for cnt in contours:
        # 过滤过小轮廓
        area = cv2.contourArea(cnt)
        if area < 5:
            continue

        # 轮廓近似
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)

        # 检查是否为四边形
        if len(approx) == 4:
            # 计算四条边长度
            sides = []
            for i in range(4):
                x1, y1 = approx[i][0]
                x2, y2 = approx[(i + 1) % 4][0]
                length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                sides.append(length)

            # 检查边长比例（正方形判断）
            max_side = max(sides)
            min_side = min(sides)
            if max_side == 0:
                continue
            aspect_ratio = min_side / max_side
            if aspect_ratio < (1 - 0.25):  # 放宽的比例阈值
                continue

            # 检查角度是否接近直角
            angles = []
            for i in range(4):
                p_curr = approx[i][0]
                p_prev = approx[(i - 1) % 4][0]
                p_next = approx[(i + 1) % 4][0]

                v_prev = p_curr - p_prev
                v_next = p_curr - p_next

                dot = np.dot(v_prev, v_next)
                norm_prev = np.linalg.norm(v_prev)
                norm_next = np.linalg.norm(v_next)

                if norm_prev > 0 and norm_next > 0:
                    cos_theta = dot / (norm_prev * norm_next)
                    angle = np.abs(np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0))))
                    angles.append(angle)

            # 角度检查
            if not angles or not all(70 < a < 110 for a in angles):
                continue

            # 计算平均边长
            side_length = np.mean(sides)

            # 面积一致性检查
            contour_area = cv2.contourArea(approx)
            expected_area = side_length ** 2
            if not (0.5 < (contour_area / expected_area) < 1.5):
                continue

            # 计算中心坐标
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # 计算实际尺寸（使用面积比例法）
            actual_size = None
            if a4_pixel_area > 0:
                area_ratio = contour_area / a4_pixel_area
                square_real_area = A4_REAL_AREA * area_ratio
                actual_size = math.sqrt(square_real_area)
            else:
                actual_size = calculate_square_size_using_a4_ratio(side_length, warped_a4)

            # 添加到结果列表
            squares.append({
                "type": "square",
                "side_length": side_length,
                "center": (cX, cY),
                "actual_size": actual_size,
                "area": contour_area,
                "contour": approx  # 添加轮廓信息
            })

    # 按面积排序
    squares.sort(key=lambda x: x["area"])
    return squares


def find_min_area_square(squares):
    """识别最小面积的正方形"""
    if not squares:
        return None
    return min(squares, key=lambda x: x["side_length"])


def convert_to_cm(pixel_length, warped_image):
    """将像素长度转换为实际厘米"""
    a4_pixel_width = warped_image.shape[1]
    pixel_to_cm_ratio = A4_REAL_WIDTH / a4_pixel_width
    return pixel_length * pixel_to_cm_ratio


### 主循环
print("程序启动 - 按以下键选择检测模式:")
print("1: 基本形状检测（正方形和三角形）")
print("2: 圆形检测")
print("3: 检测多个正方形(最小)")
print("4: 检测旋转信息")
print("5: 单独测距离")
print("ESC: 退出程序")

while True:
    # 读取图像
    ret, img_cv2 = cap.read()
    if not ret:
        print("无法获取摄像头图像")
        break

    # 显示操作提示
    cv2.putText(img_cv2, "1:Shapes 2:Circle 3:MultiSq 4:Rotate 5:Distance", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # 检测键盘输入
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC键退出
        break
    elif key == ord('1'):
        contour_type = ContourType.Basic_shapes
        print("切换到基本形状检测模式（正方形和三角形）")
    elif key == ord('2'):
        contour_type = ContourType.Basic_circle
        print("切换到圆形检测模式")
    elif key == ord('3'):
        contour_type = ContourType.Extension_Two_and_One
        print("切换到多正方形检测模式")
    elif key == ord('4'):
        contour_type = ContourType.Extension_Rotate
        print("切换到旋转检测模式")
    elif key == ord('5'):  # 单独测距离模式
        contour_type = ContourType.Distance_Measure
        print("切换到单独测距离模式")

    # 处理图像
    text = "Nothing"

    if contour_type != ContourType.Nothing:
        # 检测A4纸轮廓，获取A4纸像素面积
        contour, thresh, a4_pixel_area = detect_a4_contour(img_cv2)
        if contour is not None:
            # 计算像素宽度和距离
            pixel_width = calculate_pixel_width(contour)
            distance = calculate_distance(FOCAL_LENGTH, pixel_width)

            # 根据模式执行不同操作
            if contour_type == ContourType.Distance_Measure:
                # 单独测距离模式：只显示距离信息
                print(f"A4纸距离: {distance:.2f} cm")
                text = f"Distance: {distance:.2f} cm"
                # 绘制A4纸轮廓（可视化）
                cv2.drawContours(img_cv2, [contour], -1, (0, 255, 0), 2)
            else:
                # 透视变换
                warped = perspective_transform(thresh, contour)
                warped_copy = warped.copy()

                if contour_type == ContourType.Basic_shapes:
                    # 同时检测正方形和三角形
                    squares = detect_squares_in_a4(warped_copy, a4_pixel_area)
                    triangles = detect_triangles_in_a4(warped_copy)

                    # 合并结果
                    all_shapes = squares + triangles

                    if all_shapes:
                        # 显示所有检测到的形状
                        shape_info = []
                        for shape in all_shapes:
                            shape_info.append(f"{shape['type']}:{shape['size']:.2f}cm")
                            # 在图像上标记形状中心
                            cv2.circle(img_cv2, shape['center'], 5, (0, 255, 0), -1)

                        print(f"检测到形状: {', '.join(shape_info)}, 距离: {distance:.2f} cm")
                        text = f"Shapes,D:{distance:.2f},{', '.join(shape_info)}"

                elif contour_type == ContourType.Basic_circle:
                    circle = detect_circle_in_a4(warped_copy)
                    if circle:
                        print(f"圆形直径: {circle['size']:.2f} cm, 距离: {distance:.2f} cm")
                        text = f"Circle,D:{distance:.2f},size:{circle['size']:.2f}"
                        # 标记圆心
                        cv2.circle(img_cv2, circle['center'], 5, (0, 0, 255), -1)

                elif contour_type == ContourType.Extension_Two_and_One:
                    squares = detect_and_measure_squares(warped, a4_pixel_area)
                    if squares and len(squares) > 0:
                        min_square = find_min_area_square(squares)
                        if min_square:
                            actual_size = min_square["actual_size"]
                            print(f"最小正方形边长: {actual_size:.2f} cm, 距离: {distance:.2f} cm")
                            text = f"MinSquare,D:{distance:.2f},size:{actual_size:.2f}"

                elif contour_type == ContourType.Extension_Rotate:
                    squares = detect_and_measure_squares(warped, a4_pixel_area)
                    if squares and len(squares) > 0:
                        min_square = find_min_area_square(squares)
                        if min_square:
                            actual_size = min_square["actual_size"]
                            print(f"最小正方形边长: {actual_size:.2f} cm")
                            text = f"ROTATE,size:{actual_size:.2f} cm"

    # 显示结果
    cv2.putText(img_cv2, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.imshow("Shape Detection", img_cv2)

# 释放资源
cap.release()
cv2.destroyAllWindows()
