#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import time


class GPSSimulator(Node):
    def __init__(self):
        super().__init__('gps_simulator')
        
        # 建立發布者
        self.publisher_ = self.create_publisher(Float32MultiArray, '/drone/info', 10)
        
        # 設定定時器，0.1 秒發布一次 (10 Hz)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 模擬初始位置 (可以自行更改)
        self.base_lat = 25.033964  # 台北 101 附近
        self.base_lon = 121.564468
        self.alt = 100.0  # 無人機高度 (公尺)
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.start_time = time.time()

    def timer_callback(self):
        # 模擬無人機在原地繞圈 (yaw 旋轉)
        t = time.time() - self.start_time
        self.yaw = (t * 10) % 360  # 每秒 10 度

        # 生成訊息
        msg = Float32MultiArray()
        msg.data = [
            float(self.base_lat),
            float(self.base_lon),
            float(self.alt),
            float(self.roll),
            float(self.pitch),
            float(self.yaw)
        ]
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publish GPS Info: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = GPSSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
