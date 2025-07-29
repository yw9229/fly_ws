import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # Create CvBridge for converting between OpenCV and ROS Image
        self.bridge = CvBridge()
        # Create a publisher with topic '/camera/rgb' and message type sensor_msgs/Image
        self.publisher_ = self.create_publisher(Image, '/camera/rgb', 1)
        # Initialize camera (default is /dev/video0, i.e., ID=0)
        self.cap = cv2.VideoCapture(3)
        if not self.cap.isOpened():
            self.get_logger().error('Unable to open camera (ID=0)')
            return

        # Set resolution to 1280x720
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Use a 30 Hz timer to read and publish images
        timer_period = 1.0 / 30.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Camera publisher node started with resolution 1280×720 at 30Hz')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return

        # Convert OpenCV BGR image to ROS Image message (bgr8)
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # Add timestamp
        image_msg.header.stamp = self.get_clock().now().to_msg()
        # Publish to the rgb topic
        self.publisher_.publish(image_msg)

    def destroy_node(self):
        # Release the camera and call parent cleanup
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupt received, shutting down node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
