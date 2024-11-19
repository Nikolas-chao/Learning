import rclpy 
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
import face_recognition 
import cv2
from ament_index_python.packages import get_package_share_directory # 获取功能包绝对路径
import os
from cv_bridge import CvBridge
import time

import rclpy
from rclpy.node import Node


class FaceDetectCilent(Node):

    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'),'resource/trump.jpg')
        self.client = self.create_client(FaceDetector,'face_detect')
        self.image = cv2.imread(self.default_image_path)
    def send_request(self):
        # step1:> 等待服务上线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info("等待服务上线！")
        # step2:> 构造Requests
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        # step3:> 发送请求并等待处理完成
        future = self.client.call_async(request)    # fature没有包含响应结果，需等待服务端处理完成后才会把结果放到fature中
        # ！！！！！！！！！！！！！！！！！！！！！！
        # while not future.done():  # 这里不能使用while循环等待接受服务的返会值，sleep会休眠当前进行，会导致当前线程无法接受来自服务的返回，陷入死循环
        #     time.sleep(1.0)
        # ！！！！！！！！！！！！！！！！！！！！！！
        rclpy.spin_until_future_complete(self,future)    # 等待服务端返回响应
        response = future.result()                  # 获取响应
        self.get_logger().info(f'接收到响应,检测到{response.num}张人脸,耗时{response.use_time}s')
        self.show_response(response)

    def show_response(self,response):
        for i in range(response.num):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image,(left,top),(right,bottom),(255,0,0),4)
        cv2.imshow('Face Detecte Result',self.image)
        cv2.waitKey(0) # 也是阻塞，会导致spin无法正常运行

def main():
    rclpy.init()
    node = FaceDetectCilent()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()