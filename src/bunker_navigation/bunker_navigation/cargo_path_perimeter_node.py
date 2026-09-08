#!/usr/bin/env python3
import rclpy, csv, os
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from datetime import datetime
import matplotlib.pyplot as plt

class CargoHaulPathRecorder(Node):
    """
    Recorder for cargo-hauling routes.
      - Set /path_name (String) -> folder under perimeter_dir/<path_name>/
      - Use /record_cmd (String): 'start', 'stop', 'reset'
      - Saves in the folder:
          gps_data.csv                    (timestamp,lat,lon)
          final_combined_waypoints.csv     (lon,lat per line; follower uses this)
          gps_plot.png                     (simple preview)
    """
    def __init__(self):
        super().__init__('cargo_haul_path_recorder')

        # Where to save — must match perimeter_dir used across bunker_navigation and bunker_tools
        self.declare_parameter('perimeter_dir', '/home/user/bunker_ws/src/bunker_core/bunker_core/perimeter')
        self.base_dir = self.get_parameter('perimeter_dir').get_parameter_value().string_value
        self.path_name = "default"
        self.folder = None

        # State
        self.points = []          # [(lon, lat)]
        self.recording = False
        self.csv_log_path = None
        self.csv_log_path_run = None
        self.run_idx = 1

        # Topics
        self.create_subscription(String,  '/perimeter_name', self.set_name, 10)
        self.create_subscription(String,  '/record_cmd',    self.cmd_cb,   10)  # 'start' | 'stop' | 'reset'
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_cb, 10)

        self.get_logger().info("CargoHaulPathRecorder ready. Set /path_name, then /record_cmd 'start'.")

    # --- Topic callbacks ---
    def set_name(self, msg: String):
        self.path_name = (msg.data or "").strip() or "default"
        self._prepare_folder()
        self.get_logger().info(f"path_name = {self.path_name}")

    def cmd_cb(self, msg: String):
        cmd = (msg.data or "").strip().lower()
        if cmd == 'start':
            self.start()
        elif cmd == 'stop':
            self.stop()
        elif cmd == 'reset':
            self.points.clear()
            self.recording = False
            self.get_logger().info("reset: cleared current points.")
        else:
            self.get_logger().warn("Use /record_cmd: 'start' | 'stop' | 'reset'")

    def gps_cb(self, msg: NavSatFix):
        if not self.recording or not self.csv_log_path:
            return
        ts  = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        # store final file format (lon,lat)
        self.points.append((lon, lat))
        # append to timestamped log
        with open(self.csv_log_path_run, 'a', newline='') as f1, open(self.csv_log_path, 'a', newline='') as f2:
            w1, w2 = csv.writer(f1), csv.writer(f2)
            w1.writerow([ts, lat, lon])
            w2.writerow([ts, lat, lon])
        self.get_logger().info(f"{ts}  lat:{lat:.7f}  lon:{lon:.7f}")

    # --- Helpers ---
    def _prepare_folder(self):
        self.folder = os.path.join(self.base_dir, self.path_name)
        os.makedirs(self.folder, exist_ok=True)

        run_idx = 1
        while True:
            test_path = os.path.join(self.folder, f'gps_data_run{run_idx}.csv')
            if not os.path.exists(test_path):
                break
            run_idx += 1
        self.run_idx = run_idx

        self.csv_log_path_run = os.path.join(self.folder, f'gps_data_run{self.run_idx}.csv')
        self.csv_log_path = os.path.join(self.folder, 'gps_data.csv')

        # start fresh log with header
        with open(self.csv_log_path_run, 'w', newline='') as f1, open(self.csv_log_path, 'w', newline='') as f2:
            w1, w2 = csv.writer(f1), csv.writer(f2)
            w1.writerow(["timestamp", "lat", "lon"])
            w2.writerow(["timestamp", "lat", "lon"])
        self.get_logger().info(f"folder ready: {self.folder} (Run {self.run_idx})")

    def start(self):
        if not self.folder:
            self.get_logger().warn("Set /path_name first.")
            return
        self.points.clear()
        self.recording = True
        self.get_logger().info("recording started.")

    def stop(self):
        if not self.recording:
            self.get_logger().info("not recording; nothing to stop.")
            return
        self.recording = False
        self._write_final_csv()
        self._plot_trace()
        self.get_logger().info("recording stopped.")

    def _write_final_csv(self):
        if len(self.points) < 2:
            self.get_logger().warn("not enough points to write final_combined_waypoints.csv")
            return
        out_path_run = os.path.join(self.folder, f'final_combined_waypoints_run{self.run_idx}.csv')
        out_path = os.path.join(self.folder, 'final_combined_waypoints.csv')
        with open(out_path_run, 'w', newline='') as f1, open(out_path, 'w', newline='') as f2:
            w1, w2 = csv.writer(f1), csv.writer(f2)
            for lon, lat in self.points:
                w1.writerow([f"{lon:.9f}", f"{lat:.9f}"])  # lon,lat only
                w2.writerow([f"{lon:.9f}", f"{lat:.9f}"])
        self.get_logger().info(f"wrote: {out_path} and run{self.run_idx} ({len(self.points)} pts)")

    def _plot_trace(self):
        if len(self.points) < 2:
            return
        lons, lats = zip(*self.points)
        plt.figure(figsize=(7, 7))
        plt.plot(lons, lats, '-', lw=2, label='trace')
        plt.scatter(lons, lats, s=8, c='red')
        plt.title(f"Cargo path: {self.path_name}")
        plt.xlabel("Longitude"); plt.ylabel("Latitude"); plt.legend()
        png_run = os.path.join(self.folder, f'gps_plot_run{self.run_idx}.png')
        png = os.path.join(self.folder, 'gps_plot.png')
        plt.savefig(png_run, dpi=120)
        plt.savefig(png, dpi=120)
        plt.close()
        self.get_logger().info(f"plot saved: {png} and run{self.run_idx}")

def main(args=None):
    rclpy.init(args=args)
    node = CargoHaulPathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()