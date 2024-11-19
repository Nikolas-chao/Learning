import rclpy 
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
import face_recognition 
import cv2
from ament_index_python.packages import get_package_share_directory # 获取功能包绝对路径
import os
from cv_bridge import CvBridge
import time


class FaceDetectNode(Node): 
    def __init__(self):
        # step1:> 初始化父节点
        super().__init__("face_detect_node")
        # step2:> 初始化参数

        self.service_ = self.create_service(FaceDetector,"face_detect",self.detect_callback)    # 初始化服务端 
        self.bridge = CvBridge()  # 用于ros的image和opencv的image类型的转化
        self.number_of_times_to_upsample = 1    # 人脸识别函数的参数
        self.model = 'hog'  # 人脸识别函数的参数
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'),'resource/default.jpg')
        self.get_logger().info("人脸检测服务初始化成功！")
        
    def detect_callback(self,request,response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().warn("图像为空，使用默认图像")

        start_time = time.time() 
        self.get_logger().info("加载图像成功！开始检测！")
        
        # 使用人脸识别库，检测人脸
        face_locations = face_recognition.face_locations(cv_image,self.number_of_times_to_upsample,self.model)
        response.use_time = time.time() -start_time
        response.num = len(face_locations)
        
        
        for top,right,bottom,left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)

        self.get_logger().info("检测完成！")
        return response



def main():
    rclpy.init()
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()

