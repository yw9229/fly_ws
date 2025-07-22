import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import re

class LoraReceiverNode(Node):
    def __init__(self):
        super().__init__('lora_receiver')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/drone/info', 10)
        
        # 打開 Arduino 串口
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f"Raw: {line}")
            
            # 嘗試用正則解析 Lat, Lon, Alt, Roll, Pitch, Yaw
            match = re.search(
                r'Lat = ([\-\d\.]+), Lon = ([\-\d\.]+), Alt = ([\-\d\.]+).*Roll = ([\-\d\.]+), Pitch = ([\-\d\.]+), Yaw = ([\-\d\.]+)',
                line
            )
            if match:
                lat = float(match.group(1))
                lon = float(match.group(2))
                alt = float(match.group(3))
                roll = float(match.group(4))
                pitch = float(match.group(5))
                yaw = float(match.group(6))

                # 發布 ROS2 Topic
                msg = Float32MultiArray()
                msg.data = [lat, lon, alt, roll, pitch, yaw]
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LoraReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
