"""
人脸检测服务端
创建人:lyove
创建时间:2025.11.23
功能:接收客户端的请求，进行人脸检测，并返回响应
输入:ROS2图像消息image(sensor_msgs/msg/Image类型)
输出:检测到的人脸位置信息，检测用时，检测到的人脸数量
服务名称:face_detector
服务接口:FaceDetector
"""
import rclpy
from rclpy.node import Node
from face_detect_interfaces.srv import FaceDetector
import face_recognition
from cv_bridge import CvBridge # 用于将ROS2里的图像消息类型转换为OpenCV格式的图像

class FaceDetectNode(Node): # 继承Node类
    def __init__(self): # 构造函数
        super().__init__('face_detect_node') # 调用父类构造函数，节点名称为face_detect_node
        # 通过Node类的方法create_service创建服务端，服务接口为自定义的FaceDetector，服务名称为face_detector，回调函数为handle_face_detect
        self.srv = self.create_service(FaceDetector, 'face_detector', self.handle_face_detect)
        self.bridge = CvBridge() # 创建cv_bridge对象
        self.get_logger().info('Face Detect Service is Ready.')
        self.number_of_times_to_upsample = 1 # 设置上采样次数
        self.model = 'hog' # 设置模型类型

    def handle_face_detect(self, request, response): # 定义回调函数，接收到服务请求后，执行回调函数内容，并返回响应
        if not request.image.data:
           self.get_logger().error('No image provided.')
           return response
        else:

           self.get_logger().info('收到图像，开始进行人脸检测...')

           # 将请求里面的ROS2图像消息转换为OpenCV格式的图像
           cv_image = self.bridge.imgmsg_to_cv2(request.image)

           start_time = self.get_clock().now() # 记录检测开始时间

           # 检测人脸位置
           face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.number_of_times_to_upsample, model=self.model)
       
           end_time = self.get_clock().now() # 记录结束时间
        
           # 准备响应数据
           response.use_time = (end_time - start_time).nanoseconds / 1e9  # 检测用时，转换为秒
           response.number = len(face_locations) # 检测到的人脸数量
           response.top = []
           response.left = []
           response.right = []
           response.bottom = []

           for top, right, bottom, left in face_locations:
              # 将检测到的人脸位置信息添加到响应数据中
              response.top.append(top) 
              response.left.append(left)
              response.right.append(right)
              response.bottom.append(bottom)

           # 回调函数必须返回响应
           return response 

def main(args=None):
    rclpy.init(args=args)
    face_detect_node = FaceDetectNode()
    rclpy.spin(face_detect_node)
    rclpy.shutdown()