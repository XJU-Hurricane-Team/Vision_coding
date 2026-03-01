import requests
from openai import OpenAI

# 1. 模拟网页端的行为：先通过第三方 API 获取当前设备的公网 IP 定位
try:
    # 这里使用一个免费的 IP 定位 API
    location_data = requests.get("http://ip-api.com/json/?lang=zh-CN").json()
    lat = location_data.get("lat", "未知")
    lon = location_data.get("lon", "未知")
    city = location_data.get("city", "未知")
    location_context = f"用户当前位于{city}，经度为{lon}，纬度为{lat}。"
except Exception:
    location_context = "无法获取用户的当前位置。"

# 2. 初始化客户端
client = OpenAI(
    api_key="e9e92c72-a4e5-4bdf-bc4e-4064ef2f63c7", # 建议平时将 Key 写在环境变量中防泄露
    base_url="https://ark.cn-beijing.volces.com/api/v3",
)

# 3. 发起请求：通过 System Prompt 把位置信息“偷偷”告诉模型
response = client.chat.completions.create(
    model="ep-20260227111921-ts7hk",
    messages=[
        # 这一行就是网页版比你多做的步骤
        {"role": "system", "content": f"你是一个智能助手。请参考以下背景信息回答用户问题：{location_context}"},
        {"role": "user", "content": "当前经纬度"}
    ],
)

# 4. 打印豆包的回答
print(response.choices[0].message.content)