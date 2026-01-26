import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
class FaceDetecClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client')
        self.client = self.create_client(FaceDetector, '/face_detect')
        self.bridge = CvBridge()
        self.test1_image_path = get_package_share_directory('demo_python_service')+'/resource/test1.jpg'
        self.image = cv2.imread(self.test1_image_path)
    def send_request(self):
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('Waiting for service...')
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(
            f'接收到响应 : 图像中共有{response.number}张脸，耗时{response.use_time}'
        )
        # future = self.show_face_locations(response)
        # def request_callback(result_future):
        #     response = result_future.result()
        #     self.get_logger().info(
        #         f'接收到响应 : 图像中共有{response.number}张脸，耗时{response.use_time}'
        #     )
        #     self.show_face_locations(response)
        # future.add_done_callback(request_callback)
    def show_face_locations(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 2)
        cv2.imshow('Face Detection Result', self.image)
        cv2.waitKey(0)
    def call_set_parameters(self, parameters):
        # 1. Create a client and wait for the service to come online
        client = self.create_client(SetParameters, '/face_detection_node/set_parameters')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(' 等待参数设置服务端上线 ...')
        # 2. Create a request object
        request = SetParameters.Request()
        request.parameters = parameters
        # 3. Asynchronously call, wait for and return the response result
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response
    def update_detect_model(self, model):
        # 1. Create a parameter object
        param = Parameter()
        param.name = "face_locations_model"
        # 2. Create a parameter value object and assign it a value
        new_model_value = ParameterValue()
        new_model_value.type = ParameterType.PARAMETER_STRING
        new_model_value.string_value = model
        param.value = new_model_value
        # 3. Request the update of the parameter and handle the response
        response = self.call_set_parameters([param])
        for result in response.results:
            if result.successful:
                self.get_logger().info(f' 参数 {param.name} 设置为 {model}')
            else:
                self.get_logger().info(f' 参数设置失败，原因为：{result.reason}')
            
def main(args=None):
    rclpy.init(args=args)
    face_detect_client = FaceDetecClientNode()
    face_detect_client.update_detect_model('hog')
    face_detect_client.send_request()
    face_detect_client.update_detect_model('cnn')
    face_detect_client.send_request()
    rclpy.spin(face_detect_client)
    rclpy.shutdown()