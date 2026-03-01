import requests


def get_public_ip_info():
    url = "http://ip-api.com/json/"
    try:
        # 发送 GET 请求
        response = requests.get(url, timeout=2)
        response.raise_for_status()  # 检查请求是否成功

        # 将返回的 JSON 数据解析为字典
        data = response.json()

        if data['status'] == 'success':
            print("=== 当前公网 IP 信息 ===")
            print(f"IP 地址:  {data.get('query')}")
            print(f"国家:     {data.get('country')}")
            print(f"省/州:    {data.get('regionName')}")
            print(f"城市:     {data.get('city')}")
            print(f"经纬度:   {data.get('lat')}, {data.get('lon')}")
            print(f"运营商:   {data.get('isp')}")
            print(f"时区:     {data.get('timezone')}")
        else:
            print("获取 IP 信息失败。")

    except requests.RequestException as e:
        print(f"网络请求发生错误: {e}")


if __name__ == "__main__":
    get_public_ip_info()
