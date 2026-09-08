import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import csv, os
from datetime import datetime
import matplotlib.pyplot as plt

class PathPerimeterNode(Node):
    def __init__(self):
        super().__init__('path_perimeter_node')

        # === Directory / file state ===
        self.perimeter_name = 'default'
        self.declare_parameter('perimeter_dir', '/home/user/bunker_ws/src/bunker_core/bunker_core/perimeter')
        self.perimeter_dir = self.get_parameter('perimeter_dir').get_parameter_value().string_value
        self.file_path = None  # perimeter csv
        self.obstacles_csv_path = None  # obstacles csv

        # === Buffers / state ===
        self.gps_points = []                  # [(lon, lat), ...] perimeter
        self.recording = False                # perimeter recording flag

        # ---- OBSTACLES ----
        self.obstacle_recording = False       # currently recording an obstacle?
        self.current_obstacle_id = None       # string like "1", "2", ...
        self.obstacle_points_current = []     # [(lon, lat), ...] for the obstacle being recorded now
        self.obstacles_by_id = {}             # id -> list of (lon, lat) (for plotting after finish)

        # === Subscriptions ===
        self.cmd_sub = self.create_subscription(String, '/perimeter_cmd', self.cmd_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)
        self.name_sub = self.create_subscription(String, '/perimeter_name', self.name_callback, 10)

        # ---- OBSTACLE SUBS ----
        self.obstacle_id_sub = self.create_subscription(String, '/obstacle_id', self.obstacle_id_callback, 10)
        self.obstacle_cmd_sub = self.create_subscription(String, '/obstacle_cmd', self.obstacle_cmd_callback, 10)
        self.obstacles_finish_sub = self.create_subscription(String, '/obstacles_finish', self.obstacles_finish_callback, 10)

        self.get_logger().info("Path Perimeter Node initialized. Waiting for perimeter name...")

    # ================== Perimeter name ==================
    def name_callback(self, msg):
        self.perimeter_name = msg.data.strip()
        full_dir = os.path.join(self.perimeter_dir, self.perimeter_name)
        os.makedirs(full_dir, exist_ok=True)

        # Perimeter CSV — header must stay lowercase (lat, lon) to match path_generator_node
        self.file_path = os.path.join(full_dir, 'gps_data.csv')
        with open(self.file_path, 'w', newline='') as csvfile:
            csv.writer(csvfile).writerow(["timestamp", "lat", "lon"])

        # Obstacles CSV (id,lat,lon)
        self.obstacles_csv_path = os.path.join(full_dir, 'obstacles.csv')
        with open(self.obstacles_csv_path, 'w', newline='') as csvfile:
            csv.writer(csvfile).writerow(["id", "lat", "lon"])

        # Clear session buffers
        self.gps_points.clear()
        self.obstacles_by_id.clear()
        self.obstacle_points_current.clear()
        self.obstacle_recording = False
        self.current_obstacle_id = None

        self.get_logger().info(f"Perimeter set to: {self.perimeter_name} — {full_dir}")

    # ================== GPS handler ==================
    def gps_callback(self, msg):
        # Nothing to do if no perimeter folder yet
        if not self.perimeter_name:
            return

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        lat = float(msg.latitude)
        lon = float(msg.longitude)

        # If obstacle recording is active, write to obstacles.csv (takes precedence)
        if self.obstacle_recording and self.current_obstacle_id and self.obstacles_csv_path:
            self.obstacle_points_current.append((lon, lat))
            with open(self.obstacles_csv_path, 'a', newline='') as csvfile:
                csv.writer(csvfile).writerow([self.current_obstacle_id, lat, lon])
            self.get_logger().info(f"Obstacle {self.current_obstacle_id}: Lat {lat:.7f}, Lon {lon:.7f}")
            return  # don't also write to perimeter

        # Else, perimeter recording if enabled
        if self.recording and self.file_path:
            self.gps_points.append((lon, lat))
            with open(self.file_path, 'a', newline='') as csvfile:
                csv.writer(csvfile).writerow([timestamp, lat, lon])
            self.get_logger().info(f"Perimeter: {timestamp} Lat {lat:.7f}, Lon {lon:.7f}")

    # ================== Perimeter commands ==================
    def cmd_callback(self, msg):
        command = msg.data.strip().lower()
        if command == 'y':
            self.start_recording()
        elif command == 'n':
            self.stop_recording()
        elif command == 'e':
            self.get_logger().info("Exit command received. Ready for a new perimeter if needed.")
            self.recording = False
            self.gps_points.clear()

    def start_recording(self):
        if self.file_path:
            # If an obstacle recording is active, warn (we don't stop it automatically)
            if self.obstacle_recording:
                self.get_logger().warn("Obstacle recording is active; perimeter start ignored.")
                return
            self.recording = True
            self.get_logger().info("Started PERIMETER GPS recording.")
        else:
            self.get_logger().warn("Cannot start perimeter recording. Perimeter name not set.")

    def stop_recording(self):
        self.recording = False
        self.get_logger().info("Stopped PERIMETER recording. Plotting perimeter...")
        self.plot_perimeter()

    # ================== Obstacle workflow ==================
    def obstacle_id_callback(self, msg):
        oid = msg.data.strip()
        if not oid.isdigit():
            self.get_logger().warn(f"Received non-numeric obstacle ID: '{oid}' — ignoring.")
            return
        self.current_obstacle_id = oid
        self.get_logger().info(f"Obstacle ID set to {self.current_obstacle_id}")

    def obstacle_cmd_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "start":
            if not self.current_obstacle_id:
                self.get_logger().warn("Cannot start obstacle recording: obstacle_id not set.")
                return
            if self.obstacle_recording:
                self.get_logger().warn("Already recording an obstacle; ignoring duplicate start.")
                return
            if self.recording:
                self.get_logger().warn("Perimeter recording is ON; new obstacle will take precedence.")
            self.obstacle_points_current = []
            self.obstacle_recording = True
            self.get_logger().info(f"Started OBSTACLE {self.current_obstacle_id} recording.")
        elif cmd == "stop":
            if not self.obstacle_recording:
                self.get_logger().warn("Not currently recording an obstacle; stop ignored.")
                return
            # Save buffered points into obstacles_by_id
            pts = list(self.obstacle_points_current)
            if pts:
                self.obstacles_by_id.setdefault(self.current_obstacle_id, []).extend(pts)
                self.get_logger().info(f"Stopped OBSTACLE {self.current_obstacle_id} recording. {len(pts)} points captured.")
            else:
                self.get_logger().warn(f"Stopped OBSTACLE {self.current_obstacle_id} but captured 0 points.")
            self.obstacle_points_current = []
            self.obstacle_recording = False
        else:
            self.get_logger().warn(f"Unknown obstacle_cmd: '{cmd}'")

    def obstacles_finish_callback(self, msg):
        text = msg.data.strip().lower()
        if text != "finish":
            return
        # Consolidate any in-progress obstacle
        if self.obstacle_recording and self.current_obstacle_id and self.obstacle_points_current:
            self.obstacles_by_id.setdefault(self.current_obstacle_id, []).extend(self.obstacle_points_current)
            self.obstacle_points_current = []
            self.obstacle_recording = False
            self.get_logger().info(f"Auto-stopped OBSTACLE {self.current_obstacle_id} on finish.")

        self.get_logger().info("Obstacles finished. Generating obstacle and combined plots...")
        self.plot_obstacles()
        self.plot_combined()

    # ================== Plotting helpers ==================
    def _close_loop(self, pts):
        """Return pts closed as polygon (lon,lat) by repeating first at end if needed."""
        if not pts:
            return pts
        if pts[0] != pts[-1]:
            return pts + [pts[0]]
        return pts

    def plot_perimeter(self):
        if len(self.gps_points) < 3:
            self.get_logger().warn("Not enough perimeter points to form a polygon.")
            return

        polygon_points = self._close_loop(self.gps_points)
        longitudes, latitudes = zip(*polygon_points)

        plt.figure(figsize=(8, 8))
        plt.plot(longitudes, latitudes, 'b-', linewidth=2, label='Perimeter')
        plt.scatter(*zip(*self.gps_points), c='red', label='GPS Points')
        for i, point in enumerate(self.gps_points):
            plt.text(point[0], point[1], str(i + 1), fontsize=8)

        plt.title(f"Recorded GPS Perimeter: {self.perimeter_name}")
        plt.xlabel("Longitude"); plt.ylabel("Latitude"); plt.legend()

        output_path = os.path.join(self.perimeter_dir, self.perimeter_name, 'gps_polygon_plot.png')
        plt.savefig(output_path); plt.close()
        self.get_logger().info(f"Perimeter plot saved: {output_path}")

    def plot_obstacles(self):
        if not self.obstacles_by_id:
            self.get_logger().warn("No obstacles to plot.")
            return

        plt.figure(figsize=(8, 8))
        for oid, pts in self.obstacles_by_id.items():
            if len(pts) < 3:
                continue
            poly = self._close_loop(pts)
            xs, ys = zip(*poly)
            plt.plot(xs, ys, linewidth=2, label=f'Obstacle {oid}')
        plt.title(f"Obstacles: {self.perimeter_name}")
        plt.xlabel("Longitude"); plt.ylabel("Latitude"); plt.legend()

        out_path = os.path.join(self.perimeter_dir, self.perimeter_name, 'obstacles_plot.png')
        plt.savefig(out_path); plt.close()
        self.get_logger().info(f"Obstacles plot saved: {out_path}")

    def plot_combined(self):
        per_ok = len(self.gps_points) >= 3
        obs_ok = any(len(pts) >= 3 for pts in self.obstacles_by_id.values())

        if not per_ok and not obs_ok:
            self.get_logger().warn("Nothing to plot in combined view yet.")
            return

        plt.figure(figsize=(8, 8))

        # Perimeter in blue
        if per_ok:
            per_poly = self._close_loop(self.gps_points)
            px, py = zip(*per_poly)
            plt.plot(px, py, 'b-', linewidth=2, label='Perimeter')

        # Obstacles in orange/others (Let matplotlib pick colors)
        for oid, pts in self.obstacles_by_id.items():
            if len(pts) < 3:
                continue
            poly = self._close_loop(pts)
            ox, oy = zip(*poly)
            plt.plot(ox, oy, linewidth=2, label=f'Obstacle {oid}')

        plt.title(f"Perimeter + Obstacles: {self.perimeter_name}")
        plt.xlabel("Longitude"); plt.ylabel("Latitude"); plt.legend()

        out_path = os.path.join(self.perimeter_dir, self.perimeter_name, 'perimeter_with_obstacles.png')
        plt.savefig(out_path); plt.close()
        self.get_logger().info(f"Combined plot saved: {out_path}")

def main(args=None):
    rclpy.init(args=args)
    node = PathPerimeterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()