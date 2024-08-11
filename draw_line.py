#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from simple_pid import PID
from ultralytics import YOLO  # YOLOv8 kütüphanesi ekleniyor


class CameraAndControlNode(Node):
    def __init__(self):
        super().__init__("camera_n_control")
        
        self.model = YOLO("/home/merts/ros2_ws/src/simulasyon_2024/scripts/lane.pt")  # YOLOv8 modelinin yüklenmesi
        print(f"YOLOv8 modeli yüklendi")
        
        self.subscriber_ = self.create_subscription(Image,
                                                    "/zed_cam/camera_sensor/left/image_raw",
                                                    self.callback_camera_n_control,
                                                    10)
        
        self.msg = Twist()
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.bridge = CvBridge()
        self.get_logger().info("Control from camera has started.")
        
        # PID kontrolörünün ayarlanması
        self.pid = PID(0.1, 0.01, 0.05, setpoint=0)
        self.pid.sample_time = 0.1  # PID güncelleme süresi
        self.pid.output_limits = (-1.0, 1.0)  # Çıkış sınırları

    def callback_camera_n_control(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
        
        # YOLOv8 ile tahmin
        results = self.model(cv_image)
        boxes = results[0].boxes.xyxy  # Tahmin edilen kutuların alınması
        
        if len(boxes) > 0:
            # Kutular arasında sol ve sağ şerit kutularını tespit etme
            left_lane = None
            right_lane = None
            
            for box in boxes:
                x1, y1, x2, y2 = box
                center_x = (x1 + x2) / 2
                
                if center_x < cv_image.shape[1] / 2:
                    left_lane = center_x if left_lane is None else min(left_lane, center_x)
                else:
                    right_lane = center_x if right_lane is None else max(right_lane, center_x)
            
            if left_lane is not None and right_lane is not None:
                center_img = cv_image.shape[1] // 2
                midpoint = (left_lane + right_lane) // 2
                error = center_img - midpoint

                # PID kontrol
                control = float(self.pid(error))  # PID kontrol çıktısını float'a dönüştür

                
                # Direksiyon açısını terminale yazdırma
                print(f"Steering Angle: {control}")
                self.msg.angular.z = control
                self.msg.linear.x = 1.0
                self.publisher_.publish(self.msg)
        
        cv2.imshow("img", cv_image)
        key = cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraAndControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

