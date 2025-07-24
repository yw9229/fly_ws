import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import re

class LoraReceiverNode(Node):
    def __init__(self):
        super().__init__('lora_receiver')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/drone/info', 10)
        
        # 打開 Arduino 串口（請依實際設備修改 port）
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            # 讀入多行資料直到湊齊一筆
            lines = []
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                lines.append(line)
                if 'RSSI' in line:
                    break

            full_text = ' '.join(lines)
            self.get_logger().info(f"Raw: {full_text}")

            # 若是 RSSI 資訊，就抓出來顯示，不進行資料解析
            rssi_match = re.search(r'RSSI:\s*(-?\d+)', full_text)
            if rssi_match:
                rssi = int(rssi_match.group(1))
                self.get_logger().info(f"RSSI: {rssi} dBm")
                return  # 不再繼續解析

            # 如果不是包含 relative_alt 的資料，也跳過
            if 'relative_alt' not in full_text:
                return

            # 使用正則式抓出 7 個 float 值
            match = re.search(
                r'relative_alt\s*=\s*([\-\d\.]+).*Lat\s*=\s*([\-\d\.]+),\s*Lon\s*=\s*([\-\d\.]+),\s*Alt\s*=\s*([\-\d\.]+).*Roll\s*=\s*([\-\d\.]+),\s*Pitch\s*=\s*([\-\d\.]+),\s*Yaw\s*=\s*([\-\d\.]+)',
                full_text
            )
            if match:
                try:
                    relative_alt = float(match.group(1))
                    lat = float(match.group(2))
                    lon = float(match.group(3))
                    alt = float(match.group(4))
                    roll = float(match.group(5))
                    pitch = float(match.group(6))
                    yaw = float(match.group(7))

                    msg = Float32MultiArray()
                    msg.data = [relative_alt, lat, lon, alt, roll, pitch, yaw]
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: {msg.data}")
                except ValueError:
                    self.get_logger().warn("資料轉換失敗")
            else:
                self.get_logger().warn("無法解析感測資料格式")

def main(args=None):
    rclpy.init(args=args)
    node = LoraReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()