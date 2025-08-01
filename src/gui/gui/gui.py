import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray,Float32MultiArray
import tkinter as tk
from tkinter import messagebox, simpledialog
from tkintermapview import TkinterMapView
from PIL import Image, ImageTk
import threading
import serial
import struct
import os

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 57600

class DroneMissionPlanner(Node):
    def __init__(self):
        super().__init__('drone_mission_planner_gui')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'drone/waypoints', 10)
        # new 
        self.subscription_ = self.create_subscription(Float32MultiArray,
            '/drone/info',
            self.drone_info_callback,
            10
        )

        self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

        self.root = tk.Tk()
        self.root.title("Drone Mission Planning")
        self.root.geometry("1920x1080")

        self.waypoints = []  # (lat, lon, alt, marker)
        self.selected_index = None
        self.flight_path_line = None
        self.dragging_marker = None

        script_dir = os.path.dirname(os.path.abspath(__file__))
        icon_path = os.path.join(script_dir, "drone_icon.png")
        self.drone_icon_img = Image.open(icon_path)
        self.drone_icon_img = self.drone_icon_img.resize((40, 40), Image.ANTIALIAS)
        self.drone_icon = ImageTk.PhotoImage(self.drone_icon_img)
        self.drone_marker = None  # 編輯無人機marker

        frame_left = tk.Frame(self.root)
        frame_left.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)

        tk.Label(frame_left, text="Waypoint List (Select and press Delete to remove)").pack()
        self.listbox = tk.Listbox(frame_left, width=40, height=25)
        self.listbox.pack()
        self.listbox.bind("<<ListboxSelect>>", self.on_listbox_select)
        self.listbox.bind("<Delete>", self.delete_selected_waypoint)

        tk.Button(frame_left, text="Start Mission", command=self.start_drone_mission).pack(pady=0)

        self.map_widget = TkinterMapView(self.root, width=500, height=500)
        self.map_widget.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # 若測試成功可註解初始位置
        initial_lat = 25.174944889151664
        initial_lon = 121.45149081028671
        self.map_widget.set_position(initial_lat, initial_lon)
        self.map_widget.set_zoom(25)
        self.drone_marker = self.map_widget.set_marker(
            initial_lat,
            initial_lon,
            icon=self.drone_icon,
            text="Initial Position of the Drone"
        )
        self.map_widget.add_left_click_map_command(self.on_map_left_click)

        # threading.Thread(target=self.root.mainloop, daemon=True).start()

    # new 
    def drone_info_callback(self, msg):
        if len(msg.data) < 3:
            return  # 確保資料足夠

        lat, lon,_ = msg.data[0], msg.data[1], msg.data[2]

        # 使用 Tkinter 的 thread-safe 方法執行 GUI 更新
        self.root.after(0, self.update_drone_marker_position, lat, lon)

    def on_map_left_click(self, coords):
        click_event = self.map_widget.winfo_pointerxy()
        map_x = click_event[0] - self.map_widget.winfo_rootx()
        _ = click_event[1] - self.map_widget.winfo_rooty()

        if map_x < 50:
            return

        if self.dragging_marker is None:
            lat, lon = coords
            clicked_marker = None
            clicked_index = None
            for i, (w_lat, w_lon, _, marker) in enumerate(self.waypoints):
                if abs(w_lat - lat) < 0.0001 and abs(w_lon - lon) < 0.0001:
                    clicked_marker = marker
                    clicked_index = i
                    break

            if clicked_marker is not None:
                self.edit_waypoint_altitude(clicked_marker, clicked_index)
            else:
                if not self.waypoints:
                    alt = simpledialog.askfloat("Enter Takeoff Altitude", "Please enter the takeoff altitude (m):", parent=self.root, minvalue=0)
                else:
                    alt = simpledialog.askfloat("Enter Altitude", "Please enter the waypoint altitude (m): ", parent=self.root, minvalue=0)
                if alt is None:
                    return
                self.add_waypoint(lat, lon, alt)

    def add_waypoint(self, lat, lon, alt):
        marker = self.map_widget.set_marker(lat, lon, text=f"waypoint #{len(self.waypoints)+1}\Altitude: {alt}m")
        self.waypoints.append((lat, lon, alt, marker))
        self.update_listbox()
        self.draw_flight_path()

    def edit_waypoint_altitude(self, _, index):
        new_alt = simpledialog.askfloat("Change Altitude", f"Please enter the waypoint #{index+1} new altitude(m):", parent=self.root, minvalue=0)
        if new_alt is not None:
            lat, lon, _, old_marker = self.waypoints[index]
            old_marker.delete()
            new_marker = self.map_widget.set_marker(lat, lon, text=f"waypoint #{index+1}\nAltitude: {new_alt}m")
            self.waypoints[index] = (lat, lon, new_alt, new_marker)
            self.update_listbox()
            self.draw_flight_path()

    def update_listbox(self):
        self.listbox.delete(0, tk.END)
        for i, (lat, lon, alt, _) in enumerate(self.waypoints):
            self.listbox.insert(tk.END, f"{i+1}. ({lat:.6f}, {lon:.6f}) Altitude: {alt}m")

    def on_listbox_select(self, _):
        if not self.listbox.curselection() or self.dragging_marker is not None:
            self.selected_index = None
            return
        idx = self.listbox.curselection()[0]
        self.selected_index = idx
        lat, lon, _, _ = self.waypoints[idx]
        self.map_widget.set_position(lat, lon)

    def delete_selected_waypoint(self, _=None):
        if self.selected_index is None:
            return
        lat, lon, alt, marker = self.waypoints.pop(self.selected_index)
        marker.delete()
        self.update_listbox()
        self.draw_flight_path()
        self.selected_index = None
        for i, (lat, lon, alt, marker) in enumerate(self.waypoints):
            marker.delete()
            new_marker = self.map_widget.set_marker(lat, lon, text=f"waypoint #{i+1}\nAltitude: {alt}m")
            self.waypoints[i] = (lat, lon, alt, new_marker)

    def draw_flight_path(self):
        if hasattr(self, 'flight_path_line') and self.flight_path_line:
            self.flight_path_line.delete()
        if len(self.waypoints) < 2:
            return
        path = [(lat, lon) for (lat, lon, _, _) in self.waypoints]
        self.flight_path_line = self.map_widget.set_path(path)
# new 
    def update_drone_marker_position(self, lat, lon):
        if self.drone_marker:
            self.drone_marker.set_position(lat, lon)
        else:
            self.map_widget.set_position(lat, lon)
            self.map_widget.set_zoom(25)
            self.drone_marker = self.map_widget.set_marker(
                lat,
                lon,
                icon=self.drone_icon,
                text="Drone Position"
            )

    def start_drone_mission(self):
        if not self.waypoints:
            messagebox.showerror("Wrong", "Please add at least one waypoint.")
            return

        takeoff_altitude = self.waypoints[0][2]
        messagebox.showinfo("Mission", f"Takeoff Altitude: {takeoff_altitude}m, Total of {len(self.waypoints)} waypoints\nStart Mission")
        num_waypoints = len(self.waypoints)

        msg = Float64MultiArray()
        msg.data = []
        msg.data.append(takeoff_altitude)
        msg.data.append(float(num_waypoints))

        for (lat, lon, alt, _) in self.waypoints:
            msg.data.extend([lat, lon, alt])

        self.publisher_.publish(msg)

        # Serial payload
        payload = b''
        payload += struct.pack('<4sIiff', b"\xaa\xbb\xcc\xdd", 0, int(num_waypoints), takeoff_altitude, 0.0)
        for (lat, lon, alt, _) in self.waypoints:
            payload += struct.pack('<fff', lat, lon, alt)

        self.serial_conn.write(payload)
        print(f"Payload length: {len(payload)} bytes")

        print("=== send fly data ===")
        print(f"takeoff_altitude: {takeoff_altitude} m")
        print(f"num_waypoints: {num_waypoints}")
        for i, (lat, lon, alt, _) in enumerate(self.waypoints):
            print(f"waypoint {i+1}: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt}m")

def main():
    rclpy.init()
    planner = DroneMissionPlanner()
    try:
        # 在主執行緒中跑 GUI，讓 ROS2 在後台用 executor spin
        def ros2_spin():
            rclpy.spin(planner)

        threading.Thread(target=ros2_spin, daemon=True).start()
        planner.root.mainloop()  # Tkinter must run in main thread
    finally:
        planner.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()