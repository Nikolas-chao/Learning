import face_recognition 
import cv2
from ament_index_python.packages import get_package_share_directory # 获取功能包绝对路径
import os

def main():
    # step1:>获取图片的绝对路径 /home/cat/WorkSpace/Learning/ROS2/Code/chapt4/src/demo_python_service/resource/default.jpg
    default_image_path = os.path.join(get_package_share_directory('demo_python_service'),'resource/default.jpg')
    print(f"图片绝对路径:{default_image_path}")
    
    # step2:> 使用opencv加载图片
    image = cv2.imread(default_image_path)
    
    # step3:> 使用人脸识别库，检测人脸
    face_locations = face_recognition.face_locations(image,number_of_times_to_upsample=1,model='hog')
    
    # step4:> 绘制人脸识别框
    for top,right,bottom,left in face_locations:
        cv2.rectangle(image,(left,top),(right,bottom),(255,0,0),4)
    # step5> 结果显示
    cv2.imshow('Face Detecte Result',image)
    cv2.waitKey(0)