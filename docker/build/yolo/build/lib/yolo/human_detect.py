#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Import vision_msgs for structured detection messages
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)
# Import PoseWithCovariance for hypothesis pose (required field)
from geometry_msgs.msg import PoseWithCovariance

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # 如果你遇到 config 目录不可写的警告，可以在外部设置：
        #   export YOLO_CONFIG_DIR=/tmp/ultralytics
        # 或者在这里指定：
        # import os
        # os.environ['YOLO_CONFIG_DIR'] = '/tmp/ultralytics'

        # Load YOLO model (replace with your own weights path)
        self.model = YOLO('./src/yolo/yolo/epoch187.pt')

        # Publisher for Detection2DArray
        self.det_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        # Publisher for annotated image
        self.rgb_pub = self.create_publisher(Image, '/yolo/rgb', 10)

        # CV Bridge for converting OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Open default camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            exit(1)

        # Timer for periodic inference ~10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Cannot read image')
            return

        # Perform YOLO inference
        results = self.model(frame, imgsz=640)[0]

        # Prepare Detection2DArray message
        det_array = Detection2DArray()
        det_array.header.stamp = self.get_clock().now().to_msg()
        det_array.header.frame_id = 'camera'

        # Confidence threshold
        conf_threshold = 0.4

        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < conf_threshold:
                continue

            # Bounding box coordinates
            x1, y1, x2, y2 = map(float, box.xyxy[0])

            # Create Detection2D
            det = Detection2D()
            det.header = det_array.header

            # Fill bounding box
            bb = BoundingBox2D()
            bb.center.position.x = (x1 + x2) / 2.0
            bb.center.position.y = (y1 + y2) / 2.0
            bb.center.theta = 0.0
            bb.size_x = x2 - x1
            bb.size_y = y2 - y1
            det.bbox = bb

            # Fill hypothesis (class_id and score)
            hyp = ObjectHypothesisWithPose()
            # 将类别索引转换为字符串，赋给 class_id
            class_idx = int(box.cls[0])
            # 方法 1：使用数字作为 class_id
            hyp.hypothesis.class_id = str(class_idx)
            # 方法 2：使用模型自带的类别名称（若可读性更好）
            # hyp.hypothesis.class_id = self.model.names[class_idx]
            hyp.hypothesis.score = conf
            # 提供一个默认位姿（必须字段）
            hyp.pose = PoseWithCovariance()

            det.results = [hyp]
            det_array.detections.append(det)

        # Publish Detection2DArray
        self.det_pub.publish(det_array)

        # Annotated image for visualization
        annotated_frame = results.plot()
        ros_img = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        ros_img.header.stamp = det_array.header.stamp
        ros_img.header.frame_id = det_array.header.frame_id
        self.rgb_pub.publish(ros_img)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
