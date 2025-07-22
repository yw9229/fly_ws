import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math


class CalculateNode(Node):
    def __init__(self):
        super().__init__('calculate_node')

        # 訂閱 GPS + 姿態
        self.subscription_gps = self.create_subscription(
            Float32MultiArray,
            '/drone/info',
            self.gps_callback,
            10
        )

        # 訂閱 YOLO Bounding Box
        self.subscription_yolo = self.create_subscription(
            Float32MultiArray,
            '/yolo/bbox',
            self.yolo_callback,
            10
        )

        # 發布計算後的人員經緯度
        self.person_pub = self.create_publisher(Float32MultiArray, '/person/gps', 10)

        self.drone_info = None   # [lat, lon, alt, roll, pitch, yaw]
        self.yolo_bbox = None   # [x1, y1, x2, y2, ...]

        # 相機參數 (需依實際攝影機調整)
        self.image_width = 640
        self.image_height = 480
        self.fov_x = 155.1 * math.pi / 180  # 60度水平視角
        self.fov_y = 147.2 * math.pi / 180  # 45度垂直視角

    def gps_callback(self, msg):
        self.drone_info = msg.data  # [lat, lon, alt, roll, pitch, yaw]
        self.try_calculate()

    def yolo_callback(self, msg):
        self.yolo_bbox = msg.data  # [x1, y1, x2, y2, ...]
        self.try_calculate()

    def try_calculate(self):
        if self.drone_info is None or self.yolo_bbox is None:
            return
        if len(self.yolo_bbox) < 4:
            return  # 沒偵測到任何框

        # 取第一個框框
        x1, y1, x2, y2 = self.yolo_bbox[0:4]
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        lat, lon, alt, roll, pitch, yaw = self.drone_info

        # 計算相對地面偏移
        dx, dy = self.pixel_to_ground_offset(cx, cy, alt, yaw)

        # 計算絕對 GPS 座標
        person_lat, person_lon = self.offset_to_gps(lat, lon, dx, dy)

        self.get_logger().info(f"Person GPS: Lat={person_lat:.6f}, Lon={person_lon:.6f}")

        # 發布
        msg = Float32MultiArray()
        msg.data = [person_lat, person_lon]
        self.person_pub.publish(msg)

    def pixel_to_ground_offset(self, cx, cy, altitude, yaw):
        """
        將像素座標轉換為無人機座標系下的 (dx, dy)，單位: 公尺
        """
        # 像素 -> 正規化 [-1,1]
        norm_x = (cx - self.image_width / 2) / (self.image_width / 2)
        norm_y = (cy - self.image_height / 2) / (self.image_height / 2)

        # 估算地面偏移 (簡化為平地假設)
        dx = norm_x * altitude * math.tan(self.fov_x / 2)
        dy = norm_y * altitude * math.tan(self.fov_y / 2)

        # 考慮 yaw 旋轉
        yaw_rad = math.radians(yaw)
        rotated_dx = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        rotated_dy = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        return rotated_dx, rotated_dy

    def offset_to_gps(self, lat, lon, dx, dy):
        """
        將 (dx, dy) 偏移（公尺）轉換成經緯度
        """
        R = 6378137.0  # 地球半徑 (m)
        new_lat = lat + (dy / R) * (180 / math.pi)
        new_lon = lon + (dx / (R * math.cos(math.radians(lat)))) * (180 / math.pi)
        return new_lat, new_lon


def main(args=None):
    rclpy.init(args=args)
    node = CalculateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
