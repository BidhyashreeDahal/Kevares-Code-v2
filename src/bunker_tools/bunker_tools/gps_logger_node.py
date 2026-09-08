#!/usr/bin/env python3
"""
GPS Logger Node
- Independently records the robot's actual GPS position during navigation.
- Start/stop via /gps_log_cmd topic ("start" / "stop").
- On stop, plots the actual path vs the planned waypoints and saves the figure.
"""
import rclpy, csv, os
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from datetime import datetime
import matplotlib
matplotlib.use('Agg')  # headless backend
import matplotlib.pyplot as plt


class GpsLoggerNode(Node):
    def __init__(self):
        super().__init__('gps_logger_node')

        # === Config — must match perimeter_dir used across bunker_navigation and bunker_tools ===
        self.declare_parameter('perimeter_dir', '/home/user/bunker_ws/src/bunker_core/bunker_core/perimeter')
        self.declare_parameter('plot_size', 10.0)
        self.declare_parameter('plot_dpi', 150)
        self.base_dir = self.get_parameter('perimeter_dir').get_parameter_value().string_value
        self.plot_size = self.get_parameter('plot_size').get_parameter_value().double_value
        self.plot_dpi = self.get_parameter('plot_dpi').get_parameter_value().integer_value
        self.perimeter_name = None
        self.recording = False
        self.points = []  # [(lat, lon)]

        # === Subscriptions ===
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_cb, 10)
        self.create_subscription(String, '/gps_log_cmd', self.cmd_cb, 10)
        self.create_subscription(String, '/perimeter_name', self.name_cb, 10)

        self.get_logger().info("GPS Logger Node ready. Use /gps_log_cmd 'start' / 'stop'.")

    # ---------- Callbacks ----------
    def name_cb(self, msg: String):
        self.perimeter_name = (msg.data or "").strip() or None
        self.get_logger().info(f"GPS Logger tracking perimeter: {self.perimeter_name}")

    def cmd_cb(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if cmd == 'start':
            self.start_logging()
        elif cmd == 'stop':
            self.stop_logging()
        else:
            self.get_logger().warn("Use /gps_log_cmd: 'start' or 'stop'")

    def gps_cb(self, msg: NavSatFix):
        if not self.recording:
            return
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        self.points.append((lat, lon))

    # ---------- Start / Stop ----------
    def start_logging(self):
        self.points.clear()
        self.recording = True
        self.get_logger().info("GPS logging STARTED.")

    def stop_logging(self):
        if not self.recording:
            self.get_logger().info("Not currently logging.")
            return
        self.recording = False
        self.get_logger().info(f"GPS logging STOPPED. Recorded {len(self.points)} points.")
        self._save_and_plot()

    # ---------- Save & Plot ----------
    def _save_and_plot(self):
        if not self.perimeter_name:
            self.get_logger().warn("No perimeter_name set. Cannot save.")
            return

        folder = os.path.join(self.base_dir, self.perimeter_name)
        os.makedirs(folder, exist_ok=True)

        # --- Save actual GPS log ---
        actual_csv = os.path.join(folder, 'actual_gps_log.csv')
        with open(actual_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["lat", "lon"])
            for lat, lon in self.points:
                w.writerow([f"{lat:.9f}", f"{lon:.9f}"])
        self.get_logger().info(f"Saved actual GPS log: {actual_csv} ({len(self.points)} pts)")

        # --- Load planned waypoints ---
        planned_csv = os.path.join(folder, 'final_combined_waypoints.csv')
        planned_pts = []
        if os.path.exists(planned_csv):
            with open(planned_csv, 'r') as f:
                for row in csv.reader(f):
                    try:
                        lon = float(row[0])
                        lat = float(row[1])
                        planned_pts.append((lat, lon))
                    except Exception:
                        continue
            self.get_logger().info(f"Loaded {len(planned_pts)} planned waypoints.")
        else:
            self.get_logger().warn(f"Planned waypoints not found: {planned_csv}")

        # --- Plot comparison ---
        if len(self.points) < 2:
            self.get_logger().warn("Not enough actual points to plot.")
            return

        fig, ax = plt.subplots(figsize=(self.plot_size, self.plot_size))

        # Plot planned path
        if planned_pts:
            plan_lats, plan_lons = zip(*planned_pts)
            ax.plot(plan_lons, plan_lats, 'b-', lw=2, label='Planned Path', alpha=0.7)
            ax.scatter(plan_lons[0], plan_lats[0], c='blue', s=100, marker='^', zorder=5, label='Planned Start')
            ax.scatter(plan_lons[-1], plan_lats[-1], c='blue', s=100, marker='v', zorder=5, label='Planned End')

        # Plot actual path
        act_lats, act_lons = zip(*self.points)
        ax.plot(act_lons, act_lats, 'r-', lw=2, label='Actual Path', alpha=0.7)
        ax.scatter(act_lons[0], act_lats[0], c='red', s=100, marker='^', zorder=5, label='Actual Start')
        ax.scatter(act_lons[-1], act_lats[-1], c='red', s=100, marker='v', zorder=5, label='Actual End')

        ax.set_title(f"Planned vs Actual Path: {self.perimeter_name}", fontsize=14)
        ax.set_xlabel("Longitude")
        ax.set_ylabel("Latitude")
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        png_path = os.path.join(folder, 'planned_vs_actual.png')
        fig.savefig(png_path, dpi=self.plot_dpi, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f"Comparison plot saved: {png_path}")


def main(args=None):
    rclpy.init(args=args)
    node = GpsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("GPS Logger interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()