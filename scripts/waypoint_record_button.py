#!/usr/bin/env python3
"""
航点记录按钮 + 圆形轨迹模拟器。

- Record Waypoint: 调用 /waypoint_recorder/record 记录当前位姿
- Save / Clear:  保存/清空内存航点
- Start Circle:  在 /Odometry_global_rotate 上发布绕圆轨迹，供测试用
"""

import math
import sys
import tkinter as tk
from tkinter import messagebox

import rospy
from geometry_msgs.msg import Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest


class CircleSimulator:
    """定时器驱动，在 /Odometry_global_rotate 上发布绕圆里程计。"""

    def __init__(self, rate=50.0):
        self.rate = rate
        self.pub = rospy.Publisher("/Odometry_global", Odometry, queue_size=10)

        self.cx = rospy.get_param("~circle_center_x", 0.0)
        self.cy = rospy.get_param("~circle_center_y", 0.0)
        self.cz = rospy.get_param("~circle_altitude", 1.0)
        self.radius = rospy.get_param("~circle_radius", 2.0)
        self.period = rospy.get_param("~circle_period", 10.0)  # 一圈耗时 [s]

        self.angle = 0.0
        self.timer = None
        self.last_time = None

    @property
    def running(self):
        return self.timer is not None

    def start(self):
        if self.timer is not None:
            return
        self.last_time = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self._tick)

    def stop(self):
        if self.timer is None:
            return
        self.timer.shutdown()
        self.timer = None
        # 发一帧零速，避免 tracker 残留
        self.pub.publish(self._build_odom(0.0, 0.0, 0.0, 0.0))

    def _tick(self, event):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.rate
        self.last_time = now

        self.angle += 2.0 * math.pi * dt / self.period
        x = self.cx + self.radius * math.cos(self.angle)
        y = self.cy + self.radius * math.sin(self.angle)
        z = self.cz
        yaw = self.angle + math.pi / 2.0

        self.pub.publish(self._build_odom(x, y, z, yaw))

    @staticmethod
    def _build_odom(x, y, z, yaw):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.child_frame_id = "drone"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        half = yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        return msg


class WaypointRecordButton:
    def __init__(self, root):
        self.root = root
        self.root.title("Waypoint Recorder")
        self.root.resizable(False, False)

        rospy.init_node("waypoint_record_button", anonymous=True, disable_signals=True)

        self._wait_for_services()

        self.record_srv = rospy.ServiceProxy("indooruav_controller/waypoint_recorder/record", Trigger)
        self.save_srv = rospy.ServiceProxy("indooruav_controller/waypoint_recorder/save", Trigger)
        self.clear_srv = rospy.ServiceProxy("indooruav_controller/waypoint_recorder/clear", Trigger)

        self.sim = CircleSimulator()

        self._build_ui()

        # 窗口关闭时停止模拟器
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _wait_for_services(self):
        for svc in ["indooruav_controller/waypoint_recorder/record", "indooruav_controller/waypoint_recorder/save", "indooruav_controller/waypoint_recorder/clear"]:
            try:
                rospy.wait_for_service(svc, timeout=3.0)
            except rospy.ROSException:
                messagebox.showerror("Error", f"Service {svc} not available.\nMake sure waypoint_recorder_node is running.")
                sys.exit(1)

    def _build_ui(self):
        frame = tk.Frame(self.root, padx=20, pady=20)
        frame.pack()

        self.status_label = tk.Label(frame, text="Ready", fg="gray", font=("Helvetica", 10))
        self.status_label.pack(pady=(0, 12))

        # --- 模拟器按钮 ---
        self.sim_btn = tk.Button(
            frame, text="Start Circle",
            font=("Helvetica", 12, "bold"),
            width=20, height=1,
            bg="#2196F3", fg="white",
            activebackground="#1976D2",
            command=self._on_toggle_sim,
        )
        self.sim_btn.pack(pady=4)

        self.sim_label = tk.Label(frame, text="Simulator: OFF", fg="gray", font=("Helvetica", 9))
        self.sim_label.pack(pady=(0, 8))

        # --- 分隔线 ---
        tk.Frame(frame, height=2, bd=1, relief=tk.SUNKEN).pack(fill=tk.X, pady=8)

        # --- 航点操作按钮 ---
        btn_record = tk.Button(
            frame, text="Record Waypoint",
            font=("Helvetica", 16, "bold"),
            width=20, height=2,
            bg="#4CAF50", fg="white",
            activebackground="#45a049",
            command=self._on_record,
        )
        btn_record.pack(pady=4)

        btn_save = tk.Button(
            frame, text="Save to File",
            font=("Helvetica", 10),
            width=20, height=1,
            command=self._on_save,
        )
        btn_save.pack(pady=2)

        btn_clear = tk.Button(
            frame, text="Clear All",
            font=("Helvetica", 10),
            width=20, height=1,
            fg="red",
            command=self._on_clear,
        )
        btn_clear.pack(pady=2)

    def _on_toggle_sim(self):
        if self.sim.running:
            self.sim.stop()
            self.sim_btn.config(text="Start Circle", bg="#2196F3")
            self.sim_label.config(text="Simulator: OFF", fg="gray")
        else:
            self.sim.start()
            self.sim_btn.config(text="Stop Circle", bg="#f44336")
            self.sim_label.config(
                text=f"Simulator: ON  r={self.sim.radius}m  T={self.sim.period}s  z={self.sim.cz}m",
                fg="blue",
            )

    def _on_record(self):
        try:
            resp = self.record_srv(TriggerRequest())
            if resp.success:
                self._flash_status(resp.message, "green")
            else:
                self._flash_status(resp.message, "red")
        except rospy.ServiceException as e:
            self._flash_status(f"Service error: {e}", "red")

    def _on_save(self):
        try:
            resp = self.save_srv(TriggerRequest())
            if resp.success:
                self._flash_status(resp.message, "green")
            else:
                self._flash_status(resp.message, "red")
        except rospy.ServiceException as e:
            self._flash_status(f"Service error: {e}", "red")

    def _on_clear(self):
        if not messagebox.askyesno("Confirm", "Clear all in-memory waypoints?"):
            return
        try:
            resp = self.clear_srv(TriggerRequest())
            if resp.success:
                self._flash_status(resp.message, "green")
            else:
                self._flash_status(resp.message, "red")
        except rospy.ServiceException as e:
            self._flash_status(f"Service error: {e}", "red")

    def _flash_status(self, msg, color):
        self.status_label.config(text=msg, fg=color)
        self.root.after(2000, lambda: self.status_label.config(text="Ready", fg="gray"))

    def _on_close(self):
        self.sim.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = WaypointRecordButton(root)
    root.mainloop()


if __name__ == "__main__":
    main()
