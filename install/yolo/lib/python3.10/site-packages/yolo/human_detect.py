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

        # CV Bridge for converting between ROS Image and OpenCV images
        self.bridge = CvBridge()

        # Subscriber for input images from /rgb topic
        self.create_subscription(
            Image,
            '/camera/rgb',
            self.image_callback,
            1
        )

        self.get_logger().info('YoloNode initialized, subscribed to /rgb')

    def image_callback(self, img_msg: Image):
        try:
            # 將 ROS Image 轉為 OpenCV BGR 格式
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return


        # Perform YOLO inference
        results = self.model.track(frame, imgsz=640, persist=True)[0]

        # Prepare Detection2DArray message
        det_array = Detection2DArray()
        det_array.header.stamp = img_msg.header.stamp
        det_array.header.frame_id = img_msg.header.frame_id

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
            track_id = int(box.id[0]) if box.id is not None else -1
            hyp.hypothesis.class_id = str(track_id)
            # 或者用可讀性更好的名稱：
            # hyp.hypothesis.class_id = self.model.names[class_idx]
            hyp.hypothesis.score = conf
            hyp.pose = PoseWithCovariance()

            det.results = [hyp]
            det_array.detections.append(det)

        # Publish Detection2DArray
        self.det_pub.publish(det_array)

        # Annotated image for visualization
        annotated_frame = results.plot()
        try:
            ros_img = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            ros_img.header.stamp = img_msg.header.stamp
            ros_img.header.frame_id = img_msg.header.frame_id
            self.rgb_pub.publish(ros_img)
        except Exception as e:
            self.get_logger().error(f'cv_bridge error (annotated): {e}')

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
