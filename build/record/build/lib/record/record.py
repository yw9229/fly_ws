#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import json
from datetime import datetime
import threading
import pytz

class DetectionRecorderNode(Node):
    def __init__(self):
        super().__init__('detection_recorder')

        # Base directory for all records
        default_base = 'record_data'
        self.declare_parameter('record_data_dir', default_base)
        self.base_dir = self.get_parameter('record_data_dir').value
        os.makedirs(self.base_dir, exist_ok=True)

        # Parameters: subdirectory for images and JSON filename
        self.declare_parameter('image_save_dir', 'images')
        self.declare_parameter('json_file', None)
        image_subdir = self.get_parameter('image_save_dir').value
        self.image_save_dir = os.path.join(self.base_dir, image_subdir)

        # Create timestamped JSON filename if not provided
        taipei_tz = pytz.timezone("Asia/Taipei")
        run_ts = datetime.now(taipei_tz).isoformat()
        safe_run_ts = run_ts.replace(':', '-')
        default_json_name = f"records_{safe_run_ts}.json"
        json_param = self.get_parameter('json_file').value
        self.json_file = os.path.join(
            self.base_dir,
            json_param if json_param else default_json_name
        )

        # Ensure directories exist
        os.makedirs(self.image_save_dir, exist_ok=True)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # In-memory storage
        self.recorded_ids = set()
        self.records = []
        self.gps_data = {}

        # Lock for thread safety
        self.lock = threading.Lock()

        # Subscribe to GPS data topic
        self.create_subscription(
            Float32MultiArray,
            '/person/gps',
            self.gps_callback,
            10
        )

        # Synchronize detection and image topics
        from message_filters import Subscriber, ApproximateTimeSynchronizer
        det_sub = Subscriber(self, Detection2DArray, '/yolo/detections')
        img_sub = Subscriber(self, Image, '/yolo/rgb')
        sync = ApproximateTimeSynchronizer(
            [det_sub, img_sub],
            queue_size=10,
            slop=0.1
        )
        sync.registerCallback(self.synced_callback)

        self.get_logger().info('DetectionRecorderNode initialization complete.')

    def gps_callback(self, msg: Float32MultiArray):
        """
        Parse GPS data from /person/gps:
        Format: [id1, lat1, lon1, id2, lat2, lon2, ...]
        """
        data = msg.data
        with self.lock:
            for i in range(0, len(data), 3):
                try:
                    id_ = int(data[i])
                    lat = float(data[i+1])
                    lon = float(data[i+2])
                    self.gps_data[id_] = (lat, lon)
                except Exception as e:
                    self.get_logger().error(f'Error parsing GPS data at index {i}: {e}')

    def synced_callback(self, det_msg: Detection2DArray, img_msg: Image):
        # Current timestamp in Taipei timezone
        taipei_tz = pytz.timezone("Asia/Taipei")
        timestamp = datetime.now(taipei_tz).isoformat()

        # Convert ROS Image to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        for det in det_msg.detections:
            if hasattr(det, 'id') and det.id:
                id_str = det.id
            elif det.results and len(det.results) > 0:
                id_str = det.results[0].hypothesis.class_id
            else:
                continue

            try:
                id_ = int(id_str)
            except ValueError:
                id_ = id_str

            with self.lock:
                if id_ in self.recorded_ids:
                    continue
                self.recorded_ids.add(id_)

            # Save image with timestamped filename
            safe_ts = timestamp.replace(':', '-')
            fname = f"{safe_ts}_id_{id_}.jpg"
            img_path = os.path.join(self.image_save_dir, fname)
            cv2.imwrite(img_path, cv_image)
            self.get_logger().info(f'Saved image: {img_path}')

            with self.lock:
                lat, lon = self.gps_data.get(id_, (None, None))

            record = {
                "time": timestamp,
                "id": id_,
                "gps": {"lat": lat, "lon": lon},
                "image_path": img_path
            }

            with self.lock:
                self.records.append(record)
                self._save_records()

    def _save_records(self):
        try:
            with open(self.json_file, 'w', encoding='utf-8') as f:
                json.dump(self.records, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f'Updated JSON file: {self.json_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to write JSON file: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DetectionRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
