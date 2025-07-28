import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
from ultralytics import YOLO
import time

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.model = YOLO('./src/yolo/yolo/epoch187.pt')  # 替換成你的模型
        self.publisher_ = self.create_publisher(Float32MultiArray, '/yolo/bbox', 10)
        self.rgb_publisher_ = self.create_publisher(Image, '/yolo/rgb', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            exit()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Cannot read image')
            return

        results = self.model(frame, imgsz=320)[0]

        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = map(float, box.xyxy[0])
            detections.append([x1, y1, x2, y2])

        # 發佈 JSON
        bbox_msg = Float32MultiArray()
        bbox_msg.data = detections
        self.publisher_.publish(bbox_msg)

        # 發佈標註影像
        annotated_frame = results.plot()
        ros_img = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.rgb_publisher_.publish(ros_img)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
