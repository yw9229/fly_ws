#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from bytetrack.tracker.byte_tracker import BYTETracker

from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import PoseWithCovariance


class ByteTrackNode(Node):
    def __init__(self):
        super().__init__('bytetrack_node')

        # --- OpenCV ↔ ROS bridge ---
        self.bridge = CvBridge()

        # --- ByteTrack 初始化参数 ---
        class Args:
            track_thresh = 0.5
            track_buffer = 30
            match_thresh = 0.8
            mot20 = False
        self.tracker = BYTETracker(Args())

        # 相机分辨率（请根据实际情况修改）
        self.image_size = (640, 480)

        # 存储最后一帧图像与其 header
        self.last_frame = None
        self.last_header = None

        # 发布话题：跟踪结果（Detection2DArray）和可视化图像
        self.pub_tracked = self.create_publisher(
            Detection2DArray, '/yolo/tracked', 10
        )
        self.img_pub = self.create_publisher(
            Image, '/yolo/tracked_image', 10
        )

        # 订阅话题：来自 YOLO 的图像和检测结果
        self.create_subscription(
            Image,
            '/yolo/rgb',
            self.img_callback,
            10
        )
        self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detections_callback,
            10
        )

        self.get_logger().info('ByteTrackNode 已初始化，等待图像和检测结果...')

    def img_callback(self, msg: Image):
        try:
            # 假设 YOLO 发布的是 RGB 图像；若是 BGR 则改为 'bgr8'
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.last_header = msg.header
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge in img_callback: {e}')

    def detections_callback(self, msg: Detection2DArray):
        if self.last_frame is None:
            self.get_logger().warn('尚未收到图像，跳过跟踪')
            return

        # 1) 从 Detection2DArray 构造 ByteTrack 输入格式 [x, y, w, h, score]
        dets = []
        for det in msg.detections:
            bb = det.bbox
            cx = bb.center.position.x
            cy = bb.center.position.y
            w = bb.size_x
            h = bb.size_y
            x1 = cx - w / 2.0
            y1 = cy - h / 2.0
            # 取第一个 hypothesis 里的置信度
            score = det.results[0].hypothesis.score
            dets.append([x1, y1, w, h, score])
        if not dets:
            return
        dets_np = np.array(dets, dtype=np.float32)

        # 2) 更新跟踪器
        online_targets = self.tracker.update(
            dets_np, self.image_size, self.image_size
        )

        # 3) 发布跟踪结果为 Detection2DArray
        tracked_msg = Detection2DArray()
        tracked_msg.header = msg.header

        for t in online_targets:
            x, y, w, h = t.tlwh
            det2d = Detection2D()
            det2d.header = msg.header

            # 填充 BoundingBox2D
            bb2 = BoundingBox2D()
            bb2.center.position.x = x + w / 2.0
            bb2.center.position.y = y + h / 2.0
            bb2.center.theta = 0.0
            bb2.size_x = w
            bb2.size_y = h
            det2d.bbox = bb2

            # 使用 track_id 作为 class_id，score 暂设为 1.0
            hyp2 = ObjectHypothesisWithPose()
            hyp2.hypothesis.class_id = str(t.track_id)
            hyp2.hypothesis.score = 1.0
            hyp2.pose = PoseWithCovariance()
            det2d.results = [hyp2]

            tracked_msg.detections.append(det2d)

        self.pub_tracked.publish(tracked_msg)
        self.get_logger().info(
            f'Published {len(online_targets)} tracked targets'
        )

        # 4) 在图像上绘制跟踪框并发布
        vis = self.last_frame.copy()
        for t in online_targets:
            x, y, w, h = map(int, t.tlwh)
            cv2.rectangle(
                vis, (x, y), (x + w, y + h), (0, 255, 0), 2
            )
            cv2.putText(
                vis, str(t.track_id), (x, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )

        try:
            img_out = self.bridge.cv2_to_imgmsg(vis, 'rgb8')
            img_out.header = self.last_header
            self.img_pub.publish(img_out)
            self.get_logger().info('Published tracked image')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge in detections_callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ByteTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
