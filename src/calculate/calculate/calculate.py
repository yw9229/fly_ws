#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2DArray
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

        # 訂閱 YOLO Detection2DArray
        self.subscription_yolo = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.yolo_callback,
            10
        )

        # 發布計算後的人員經緯度 (一次性發佈所有偵測結果)
        self.person_pub = self.create_publisher(Float32MultiArray, '/person/gps', 10)

        self.drone_info = None  # [lat, lon, relative_alt, roll, pitch, yaw]

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
        
    def get_camera_rotation_matrix(self, tilt_deg=45.0):
        angle = math.radians(tilt_deg)
        Ry = np.array([[ math.cos(angle), 0, math.sin(angle)],
                       [ 0,               1, 0              ],
                       [-math.sin(angle), 0, math.cos(angle)]])
        return Ry

    def gps_callback(self, msg):
        self.drone_info = msg.data  # [lat, lon, relative_alt, roll, pitch, yaw]

    def yolo_callback(self, msg: Detection2DArray):
        if self.drone_info is None:
            return  # 無法計算，等有 GPS 資訊

        if not msg.detections:
            return  # 沒有偵測到任何物件

        all_persons = []  # 儲存所有人的 [id, lat, lon]

        # 遍歷每個 Detection2D
        for det in msg.detections:
            track_id = int(det.results[0].hypothesis.class_id)  # 追蹤 ID
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y

            person_data = self.calculate_for_one_object(track_id, cx, cy)
            all_persons.append(person_data)

        # 將所有人的 GPS 一次發佈
        msg_out = Float32MultiArray()
        msg_out.data = [item for sublist in all_persons for item in sublist]  # 扁平化為一個 list
        self.person_pub.publish(msg_out)

    def calculate_for_one_object(self, track_id, cx, cy):
        lat, lon, relative_alt, roll, pitch, yaw = self.drone_info

        # 計算像素點對應的地面偏移
        dx, dy = self.pixel_to_ground_offset(cx, cy, relative_alt, roll, pitch, yaw)

        # 計算絕對 GPS 座標
        person_lat, person_lon = self.offset_to_gps(lat, lon, dx, dy)

        self.get_logger().info(f"Track ID={track_id} | GPS: Lat={person_lat:.6f}, Lon={person_lon:.6f}")

        return [float(track_id), person_lat, person_lon]

    def pixel_to_ground_offset(self, u, v, altitude, roll, pitch, yaw):
        pts = np.array([[[u, v]]], dtype=np.float64)
        undistorted = cv2.undistortPoints(pts, self.K, self.dist_coeffs)
        x_c, y_c = undistorted[0, 0, 0], undistorted[0, 0, 1]

        ray_cam = np.array([x_c, y_c, 1.0])

        R_body2world = self.rpy_to_matrix(roll, pitch, yaw)
        ray_world = R_body2world @ (self.R_cam2body @ ray_cam)

        t = -altitude / ray_world[2]
        dx = t * ray_world[0]
        dy = t * ray_world[1]
        return dx, dy

    def offset_to_gps(self, lat, lon, dx, dy):
        R = 6378137.0
        new_lat = lat + (dy / R) * (180 / math.pi)
        new_lon = lon + (dx / (R * math.cos(math.radians(lat)))) * (180 / math.pi)
        return new_lat, new_lon

    def rpy_to_matrix(self, roll, pitch, yaw):
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
