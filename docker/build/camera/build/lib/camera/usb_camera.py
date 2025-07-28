#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/rgb', 10)
        self.bridge = CvBridge()

        self.get_logger().info('Initializing camera...')
        self.cap = None
        self.retry_open_camera()

        # 使用定時器讀取影像
        timer_period = 1.0/30.0  # 30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def retry_open_camera(self):
        """嘗試打開攝影機，若失敗則指數延遲重試"""
        delay = 1
        while self.cap is None or not self.cap.isOpened():
            self.get_logger().warn(f'Failed to open camera, retrying in {delay} second(s)...')
            time.sleep(delay)
            self.cap = cv2.VideoCapture(0)
            delay = min(delay * 2, 30)  # 最長延遲 30 秒
        self.get_logger().info('Camera opened successfully!')

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error('Camera disconnected! Trying to reopen...')
            self.retry_open_camera()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame, retrying...')
            self.retry_open_camera()
            return

        # 轉換為 ROS2 Image 訊息
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(img_msg)
        self.get_logger().debug('Published an image frame.')


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
