import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from yolox.tracker.byte_tracker import BYTETracker

class ByteTrackNode(Node):
    def __init__(self):
        super().__init__('bytetrack_node')

        # 訂閱 YOLO Bounding Boxes
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/yolo/bbox',
            self.bbox_callback,
            10
        )

        # 發布含 ID 的追蹤框
        self.pub_tracked = self.create_publisher(
            Float32MultiArray,
            '/yolo/tracked',
            10
        )

        # 初始化 ByteTrack
        self.tracker = BYTETracker(
            track_thresh=0.5,   # 追蹤閾值
            match_thresh=0.8,   # 匹配閾值
            track_buffer=30     # 幀緩衝
        )

        self.image_size = [640, 480]  # 影像大小 (記得跟相機解析度一致)

    def bbox_callback(self, msg):
        # msg.data 格式: [x1, y1, x2, y2, conf, x1, y1, x2, y2, conf, ...]
        data = msg.data
        if len(data) < 5:
            return  # 沒偵測到物件

        detections = []
        for i in range(0, len(data), 5):
            x1, y1, x2, y2, conf = data[i:i+5]
            w = x2 - x1
            h = y2 - y1
            detections.append([x1, y1, w, h, conf])

        # 更新 ByteTrack
        online_targets = self.tracker.update(detections, self.image_size, self.image_size)

        # 整理輸出資料
        tracked_data = []
        for t in online_targets:
            x, y, w, h = t.tlwh
            x2 = x + w
            y2 = y + h
            tracked_data.extend([t.track_id, x, y, x2, y2, t.score])

        # 發布結果
        if tracked_data:
            msg_out = Float32MultiArray()
            msg_out.data = tracked_data
            self.pub_tracked.publish(msg_out)
            self.get_logger().info(f"Tracked {len(online_targets)} targets")

def main(args=None):
    rclpy.init(args=args)
    node = ByteTrackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

