#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from simple_pid import PID
from ultralytics import YOLO
import os

class LaneFollowingNode(Node):
    def __init__(self):
        super().__init__("lane_following_pid")

        # YOLO modeli ve yollar
        self.model = YOLO("/home/merts/ros2_ws/src/simulasyon_2024/scripts/lane.pt")
        self.images_path = "./lanefotodata"
        self.output_dir = "./deneme_mark"
        
        # Görüntü işleme için ROS abone ve yayıncıları
        self.bridge = CvBridge()
        self.subscriber_ = self.create_subscription(Image, "/zed_cam/camera_sensor/left/image_raw", self.callback_image, 10)
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # PID kontrolörünün ayarlanması
        self.pid = PID(0.1, 0.01, 0.05, setpoint=0)
        self.pid.sample_time = 0.1  # PID güncelleme süresi
        self.pid.output_limits = (-1.0, 1.0)  # Çıkış sınırları

        self.get_logger().info("Lane following with PID has started.")
        
        # Çıktı dizinini oluştur
        os.makedirs(self.output_dir, exist_ok=True)

    def callback_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # YOLO modeli ile görüntüyü işle
        results = self.model(cv_image)
        masks = results[0].masks

        b_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)

        # Maskeleri birleştir
        if masks is not None:
            for i, mask in enumerate(masks):
                binary_mask = mask.data.cpu().numpy().astype(np.uint8) * 255

                if len(binary_mask.shape) == 3 and binary_mask.shape[0] == 1:
                    binary_mask = binary_mask.squeeze(0)

                if binary_mask is not None and binary_mask.size > 0:
                    if binary_mask.shape != b_mask.shape:
                        binary_mask = cv2.resize(binary_mask, (b_mask.shape[1], b_mask.shape[0]))

                    b_mask = cv2.bitwise_or(b_mask, binary_mask)

        # Hough Çizgi Dönüşümü kullanarak çizgileri tespit et
        edges = cv2.Canny(b_mask, 50, 150, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

        # Görüntüdeki merkez noktayı belirle
        img_center = b_mask.shape[1] / 2

        # Tespit edilen çizgileri depolamak için listeler
        left_lines = []
        right_lines = []

        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))

                if x0 < img_center:
                    left_lines.append((x1, y1, x2, y2))
                else:
                    right_lines.append((x1, y1, x2, y2))

        # Ortalama sapmayı hesapla (basit bir ortalama ile)
        left_avg_x = np.mean([line[0] for line in left_lines]) if left_lines else img_center
        right_avg_x = np.mean([line[0] for line in right_lines]) if right_lines else img_center

        # Yolun merkezini hesapla
        lane_center = (left_avg_x + right_avg_x) / 2

        # Sapmayı hesapla (araç merkezine göre)
        deviation = lane_center - img_center

        # PID kontrol
        steering_angle = float(self.pid(deviation))  # PID kontrol çıktısını float'a dönüştür

        # Direksiyon açısını terminale yazdırma
        self.get_logger().info(f"Steering Angle: {steering_angle}")

        # Araç kontrol mesajı oluştur ve yayınla
        twist_msg = Twist()
        twist_msg.angular.z = steering_angle
        twist_msg.linear.x = 0.5  # Sabit bir ileri hız belirleniyor
        self.publisher_.publish(twist_msg)

        # Görüntüyü göster (isteğe bağlı)
        cv2.imshow('Detected Lines', b_mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

