import cv2
import requests
import math
import threading
import time
import os
import socket
from flask import Flask, render_template, Response, jsonify, request, send_from_directory
from ultralytics import YOLO

app = Flask(__name__)

# 定位服务配置
WEB_SERVICE_KEY = "3b56e23a509f36ab6770e5a420efa95b"
MAP_FILENAME = "map_result.png"
MAP_WIDTH = 800
MAP_HEIGHT = 600
ZOOM_LEVEL = 16

# 全局共享变量
latest_frame = None
latest_map = None
lock = threading.Lock()

# 全局 GPS 变量，用于实时保存手机传来的定位
global_lon = None
global_lat = None

# 确保保存严重裂缝的文件夹存在
SAVE_DIR = "severe_cracks"
os.makedirs(SAVE_DIR, exist_ok=True)


def get_local_ip():
    """获取本机在局域网中的IP地址，方便生成手机访问链接"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip


def lonlat_to_pixel(target_lon, target_lat, center_lon, center_lat, zoom, width, height):
    def get_mercator_px(lon, lat, zoom):
        n = 2.0 ** zoom
        x = ((lon + 180.0) / 360.0) * n * 256.0
        lat_rad = math.radians(lat)
        y = (1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n * 256.0
        return x, y

    cx, cy = get_mercator_px(center_lon, center_lat, zoom)
    tx, ty = get_mercator_px(target_lon, target_lat, zoom)
    return int(width / 2 + (tx - cx)), int(height / 2 + (ty - cy))


def get_location_and_static_map():
    global global_lon, global_lat
    print("正在等待手机端发送初始 GPS 定位 (请用手机浏览器访问控制台提示的 HTTPS 地址)...")

    # 阻塞等待手机传来第一次真实的 GPS 坐标
    while global_lon is None or global_lat is None:
        time.sleep(1)

    center_lon, center_lat = global_lon, global_lat
    print(f"[成功] 获取到手机初始 GPS 定位：{center_lon}, {center_lat}")

    print("正在加载区域定位底图...")
    static_map_url = "https://restapi.amap.com/v3/staticmap"
    map_params = {
        "key": WEB_SERVICE_KEY, "location": f"{center_lon},{center_lat}",
        "zoom": ZOOM_LEVEL, "size": f"{MAP_WIDTH}*{MAP_HEIGHT}"
    }
    try:
        map_response = requests.get(static_map_url, params=map_params, timeout=10)
        if map_response.status_code == 200:
            with open(MAP_FILENAME, "wb") as f:
                f.write(map_response.content)
            return center_lon, center_lat
    except Exception as e:
        print("地图服务加载异常:", e)
    return None, None


def process_video_and_map():
    """后台核心线程：运行视觉模型并绘制定位点"""
    global latest_frame, latest_map, global_lon, global_lat

    # 获取初始中心坐标和地图
    center_lon, center_lat = get_location_and_static_map()
    if center_lon is None:
        print("无法初始化定位服务，程序退出。")
        return

    base_map_img = cv2.imread(MAP_FILENAME)
    if base_map_img is None:
        print("地图文件读取失败，程序退出。")
        return

    model = YOLO('best.pt')
    cap = cv2.VideoCapture(0)

    # 基于空间区域划分的记录器
    zones = {}  # 记录每个区域的最高严重程度: {(cx, cy): max_count}
    saved_severe_locations = set()
    ZONE_RADIUS = 20  # 物理区域吸附半径（像素）

    print("视觉识别模块已启动，开始实时监测...")

    while True:
        success, frame = cap.read()
        if not success:
            time.sleep(0.1)
            continue

        results = model.track(frame, persist=True, verbose=False)
        annotated_frame = results[0].plot()
        boxes = results[0].boxes

        if boxes is not None:
            current_count = len(boxes)

            # 使用从手机实时获取的最新的全局 GPS 坐标
            current_lon, current_lat = global_lon, global_lat

            # 确保定位存在
            if current_lon is not None and current_lat is not None:
                px, py = lonlat_to_pixel(current_lon, current_lat, center_lon, center_lat, ZOOM_LEVEL, MAP_WIDTH,
                                         MAP_HEIGHT)

                if 0 <= px < MAP_WIDTH and 0 <= py < MAP_HEIGHT:

                    # 【核心修改】去除了不论是否检测到都画轨迹点的代码
                    # 现在只有检测到至少1个裂缝时，才开始处理位置逻辑和画点
                    if current_count > 0:
                        my_zone = None
                        min_dist = float('inf')
                        for cx, cy in zones.keys():
                            dist = math.hypot(cx - px, cy - py)
                            if dist <= ZONE_RADIUS:
                                if dist < min_dist:
                                    min_dist = dist
                                    my_zone = (cx, cy)

                        if my_zone is None:
                            my_zone = (px, py)
                            zones[my_zone] = current_count
                        else:
                            if current_count > zones[my_zone]:
                                zones[my_zone] = current_count

                        severity = zones[my_zone]

                        # 根据严重程度判定颜色和大小
                        if severity >= 6:
                            color, radius = (0, 0, 255), 12  # 红色
                            if my_zone not in saved_severe_locations:
                                timestamp = time.strftime("%Y%m%d_%H%M%S")
                                save_path = os.path.join(SAVE_DIR,
                                                         f"severe_lon{current_lon:.6f}_lat{current_lat:.6f}_{timestamp}.jpg")
                                cv2.imwrite(save_path, annotated_frame)
                                print(f"[警报] 发现严重路面损伤({severity}处)！已保存: {save_path}")
                                saved_severe_locations.add(my_zone)
                        elif severity >= 3:
                            color, radius = (0, 165, 255), 8  # 橙色
                        else:
                            color, radius = (0, 255, 255), 5  # 黄色

                        # 在地图上画点（只有发现了裂缝才会执行到这里）
                        cx, cy = my_zone
                        cv2.circle(base_map_img, (cx, cy), radius=radius, color=color, thickness=-1)
                        cv2.circle(base_map_img, (cx, cy), radius=radius, color=(0, 0, 0), thickness=1)

        with lock:
            latest_frame = annotated_frame.copy()
            latest_map = base_map_img.copy()


def generate_stream(stream_type):
    global latest_frame, latest_map
    while True:
        with lock:
            img = latest_frame if stream_type == 'video' else latest_map

        if img is None:
            time.sleep(0.1)
            continue

        ret, buffer = cv2.imencode('.jpg', img)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.05)


# --- 路由配置 ---

@app.route('/')
def home():
    return render_template('home.html')


@app.route('/detection')
def detection():
    return render_template('detection.html')


@app.route('/severe')
def severe():
    return render_template('severe.html')


# 供手机端访问发送定位数据的页面
@app.route('/gps')
def gps():
    return render_template('gps.html')


@app.route('/video_feed')
def video_feed():
    return Response(generate_stream('video'), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/map_feed')
def map_feed():
    return Response(generate_stream('map'), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/images/<filename>')
def serve_image(filename):
    return send_from_directory(SAVE_DIR, filename)


# 接收手机端 GPS 数据 API
@app.route('/api/receive_location', methods=['POST'])
def receive_location():
    global global_lon, global_lat
    data = request.get_json()
    if data:
        global_lat = data.get('latitude')
        global_lon = data.get('longitude')
        return jsonify({"status": "success", "message": "位置数据接收完毕"})
    return jsonify({"status": "error", "message": "未收到有效数据"}), 400


@app.route('/api/records')
def get_records():
    records = []
    if os.path.exists(SAVE_DIR):
        for filename in os.listdir(SAVE_DIR):
            if filename.endswith(".jpg") and filename.startswith("severe_"):
                parts = filename.replace(".jpg", "").split("_")
                try:
                    lon = parts[1].replace("lon", "")
                    lat = parts[2].replace("lat", "")
                    time_str = f"{parts[3]}_{parts[4]}"
                    records.append({"filename": filename, "lon": lon, "lat": lat, "time": time_str})
                except IndexError:
                    continue
    records.sort(key=lambda x: x['time'], reverse=True)
    return jsonify(records)


@app.route('/api/fetch_map')
def fetch_map():
    lon = request.args.get('lon')
    lat = request.args.get('lat')
    if not lon or not lat:
        return "Missing coordinates", 400
    static_map_url = "https://restapi.amap.com/v3/staticmap"
    map_params = {
        "key": WEB_SERVICE_KEY, "location": f"{lon},{lat}",
        "zoom": ZOOM_LEVEL, "size": f"{MAP_WIDTH}*{MAP_HEIGHT}",
        "markers": f"mid,,A:{lon},{lat}"
    }
    try:
        resp = requests.get(static_map_url, params=map_params, timeout=10)
        if resp.status_code == 200:
            return Response(resp.content, mimetype='image/png')
    except Exception as e:
        print("定位图加载异常:", e)
    return "Failed to fetch map data", 500


if __name__ == "__main__":
    local_ip = get_local_ip()
    print("\n" + "=" * 60)
    print("手机端采集GPS，请务必使用手机浏览器访问以下地址 (注意必须是 https):")
    print(f"👉  https://{local_ip}:5000/gps  👈")
    print("PC端监控看板，请访问：")
    print(f"👉  https://{local_ip}:5000  👈")
    print("=" * 60 + "\n")

    t = threading.Thread(target=process_video_and_map, daemon=True)
    t.start()

    # 启用自动 HTTPS，以便移动端允许获取定位权限
    app.run(host='0.0.0.0', port=5000, threaded=True, ssl_context='adhoc')