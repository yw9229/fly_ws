import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
import cv2


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
        self.yolo_bbox = None    # [x1, y1, x2, y2, ...]

        # 相機內參矩陣 (需標定得到)
        self.fx = 244.61769346  
        self.fy = 325.31942852
        self.cx = 329.07635483  # assume image width=640
        self.cy = 249.59847313  # assume image height=480
        self.K = np.array([[self.fx, 0, self.cx],
                           [0, self.fy, self.cy],
                           [0, 0, 1]], dtype=np.float64)

        # 相機畸變參數 (k1, k2, p1, p2, k3)
        self.dist_coeffs = np.array([-0.24119534, 0.05743331, 0.00039577, 0.00080172, -0.00581896], dtype=np.float64)

        # 相機與機體的姿態 (假設相機朝下)
        self.R_cam2body = self.get_camera_rotation_matrix(tilt_deg=45.0)
        

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

        x1, y1, x2, y2 = self.yolo_bbox[0:4]
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2

        lat, lon, alt, roll, pitch, yaw = self.drone_info

        # 計算像素點對應的地面偏移
        dx, dy = self.pixel_to_ground_offset(cx, cy, alt, roll, pitch, yaw)

        # 計算絕對 GPS 座標
        person_lat, person_lon = self.offset_to_gps(lat, lon, dx, dy)

        self.get_logger().info(f"Person GPS: Lat={person_lat:.6f}, Lon={person_lon:.6f}")

        # 發布
        msg = Float32MultiArray()
        msg.data = [person_lat, person_lon]
        self.person_pub.publish(msg)

    def pixel_to_ground_offset(self, u, v, altitude, roll, pitch, yaw):
        """
        使用相機內參計算像素 (u,v) 在地面上的偏移量 (dx, dy) 單位: m
        """
        # Step 1: 先做畸變矯正
        pts = np.array([[[u, v]]], dtype=np.float64)
        undistorted = cv2.undistortPoints(pts, self.K, self.dist_coeffs)
        x_c, y_c = undistorted[0, 0, 0], undistorted[0, 0, 1]

        # Step 2: 相機座標射線 (歸一化)
        ray_cam = np.array([x_c, y_c, 1.0])

        # Step 3: 將相機射線轉到世界座標
        R_body2world = self.rpy_to_matrix(roll, pitch, yaw)
        ray_world = R_body2world @ (self.R_cam2body @ ray_cam)

        # Step 4: 計算與地面的交點
        t = -altitude / ray_world[2]
        dx = t * ray_world[0]
        dy = t * ray_world[1]
        return dx, dy

    def offset_to_gps(self, lat, lon, dx, dy):
        """
        將 (dx, dy) 偏移（公尺）轉換成經緯度
        """
        R = 6378137.0  # 地球半徑 (m)
        new_lat = lat + (dy / R) * (180 / math.pi)
        new_lon = lon + (dx / (R * math.cos(math.radians(lat)))) * (180 / math.pi)
        return new_lat, new_lon

    def rpy_to_matrix(self, roll, pitch, yaw):
        """
        將 roll, pitch, yaw (度) 轉換成旋轉矩陣
        """
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

        Rx = np.array([[1, 0, 0],
                       [0, math.cos(roll), -math.sin(roll)],
                       [0, math.sin(roll), math.cos(roll)]])
        Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                       [0, 1, 0],
                       [-math.sin(pitch), 0, math.cos(pitch)]])
        Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                       [math.sin(yaw), math.cos(yaw), 0],
                       [0, 0, 1]])
        return Rz @ Ry @ Rx


def main(args=None):
    rclpy.init(args=args)
    node = CalculateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
