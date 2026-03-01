from flask import Flask, render_template, request, jsonify
import socket

app = Flask(__name__)


# 获取你电脑在局域网中的 IP 地址（方便你用手机访问）
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/receive_location', methods=['POST'])
def receive_location():
    data = request.get_json()
    if data:
        lat = data.get('latitude')
        lon = data.get('longitude')
        accuracy = data.get('accuracy')

        print("\n" + "🚀" * 15)
        print("🎉 成功接收到手机发来的真实 GPS 数据！")
        print(f"📍 纬度 (Latitude):  {lat}")
        print(f"📍 经度 (Longitude): {lon}")
        print(f"🎯 精度 (Accuracy):   {accuracy} 米")
        print("🚀" * 15 + "\n")

        return jsonify({"status": "success", "message": "位置数据接收完毕"})

    return jsonify({"status": "error", "message": "未收到有效数据"}), 400


if __name__ == '__main__':
    local_ip = get_local_ip()
    print("\n" + "=" * 50)
    print("手机测试请访问以下地址 (注意必须是 https):")
    print(f"👉  https://{local_ip}:5000  👈")
    print("=" * 50 + "\n")

    # ssl_context='adhoc' 会自动生成临时的 HTTPS 证书
    app.run(host='0.0.0.0', port=5000, debug=True, ssl_context='adhoc')