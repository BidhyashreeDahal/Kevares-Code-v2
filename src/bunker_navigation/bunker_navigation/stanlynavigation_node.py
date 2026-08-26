import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float64, Float32, String
from sensor_msgs.msg import NavSatFix
import math
import csv
import numpy as np
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class NavigationAndControlNode(Node):
    def __init__(self):
        super().__init__('navigation_and_control')

        # === ROS Pub/Sub ===
        self.publisher = self.create_publisher(Int32MultiArray, 'movement_commands', 10)
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_callback, 10)
        self.create_subscription(Float64, '/gps_heading', self.heading_callback, 10)
        self.create_subscription(Float32, '/obstacle_speed_factor', self.speed_factor_callback, 10)
        self.create_subscription(String, '/stanley_start_cmd', self.start_callback, 10)
        self.create_subscription(String, '/perimeter_name', self.perimeter_name_callback, 10)

        # === Waypoints and Params ===
        self.declare_parameter('perimeter_dir', '/home/bidya/Kevares-Code-v2/data/perimeter')
        self.perimeter_dir = self.get_parameter('perimeter_dir').get_parameter_value().string_value
        self.perimeter_name = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_lat = None
        self.current_lon = None
        self.heading = None
        self.speed = 500
        self.speed_factor = 1.0
        self.navigation_enabled = False
        self.k = -0.5
        self.stop_navigation = 1

        # === GPS Logger (automatic) ===
        self.actual_gps_points = []   # [(lat, lon)] recorded during navigation
        self.navigation_complete = False

        # === Timer ===
        self.create_timer(0.01, self.navigate)

    def perimeter_name_callback(self, msg: String):
        self.perimeter_name = msg.data.strip()
        path = os.path.join(self.perimeter_dir, self.perimeter_name, "final_combined_waypoints.csv")
        if os.path.exists(path):
            self.waypoints = self.load_waypoints(path)
            self.current_waypoint_index = 0
            self.get_logger().info(f"Loaded waypoints from: {path}")
        else:
            self.get_logger().warn(f"Waypoint file not found: {path}")

    def start_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "start":
            self.actual_gps_points.clear()
            self.navigation_complete = False
            self.current_waypoint_index = 0
            self.navigation_enabled = True
            self.get_logger().info("Navigation started via /stanley_start_cmd — GPS logging active.")
        elif cmd == "stop":
            self.navigation_enabled = False
            self._finish_navigation("Manual stop")

    def speed_factor_callback(self, msg: Float32):
        self.speed_factor = max(0.0, min(1.0, msg.data))
        self.get_logger().info(f"Received speed factor: {self.speed_factor:.2f}")

    def load_waypoints(self, filepath):
        latlon = []
        with open(filepath, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                try:
                    lat = float(row[1])
                    lon = float(row[0])
                    latlon.append((lat, lon))
                except:
                    continue

        interpolated = []
        for i in range(len(latlon) - 1):
            lat0, lon0 = latlon[i]
            lat1, lon1 = latlon[i + 1]
            for t in np.linspace(0, 1, 101):
                lati = (1 - t) * lat0 + t * lat1
                loni = (1 - t) * lon0 + t * lon1
                interpolated.append((lati, loni))
        return interpolated

    def gps_callback(self, msg):
        self.current_lat = float(msg.latitude)
        self.current_lon = float(msg.longitude)
        # Auto-record GPS during navigation
        if self.navigation_enabled and not self.navigation_complete:
            self.actual_gps_points.append((self.current_lat, self.current_lon))

    def heading_callback(self, msg):
        self.heading = msg.data

    def latlon_to_xy(self, lat1, lon1, lat2, lon2):
        avg_lat_rad = math.radians((lat1 + lat2) / 2.0)
        dx = (lon2 - lon1) * 111000 * math.cos(avg_lat_rad)
        dy = (lat2 - lat1) * 111000
        return dx, dy

    def stanley_control(self, lat, lon, yaw_rad, v, lookahead_dist=2, search_window=3.0):
        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_navigation = 0
            return 0.0

        self.stop_navigation = 1
        nearest_index = self.current_waypoint_index
        min_dist = float('inf')

        for i in range(self.current_waypoint_index, len(self.waypoints)):
            wp_lat, wp_lon = self.waypoints[i]
            dx, dy = self.latlon_to_xy(lat, lon, wp_lat, wp_lon)
            dist = math.hypot(dx, dy)
            if dist < min_dist and dist < search_window:
                min_dist = dist
                nearest_index = i
            elif dist > search_window:
                break
        if nearest_index == self.current_waypoint_index:
            min_dist = float('inf')
            for i in range(self.current_waypoint_index, len(self.waypoints)):
                wp_lat, wp_lon = self.waypoints[i]
                dx, dy = self.latlon_to_xy(lat, lon, wp_lat, wp_lon)
                dist = math.hypot(dx, dy)

                if dist < min_dist:
                    min_dist = dist
                    nearest_index = i

                if dist > min_dist:
                    break  # passed closest point

        lookahead_index = nearest_index
        for j in range(nearest_index + 1, len(self.waypoints)):
            dx, dy = self.latlon_to_xy(*self.waypoints[nearest_index], *self.waypoints[j])
            dist = math.hypot(dx, dy)
            if dist >= lookahead_dist:
                lookahead_index = j
                break

        dx_path, dy_path = self.latlon_to_xy(*self.waypoints[nearest_index], *self.waypoints[lookahead_index])
        path_yaw = math.atan2(dy_path, dx_path)
        heading_error = math.atan2(math.sin(path_yaw - yaw_rad), math.cos(path_yaw - yaw_rad))

        dx_ct, dy_ct = self.latlon_to_xy(lat, lon, *self.waypoints[nearest_index])
        cross_track_error = dx_ct * math.sin(path_yaw) - dy_ct * math.cos(path_yaw)

        steer_angle = heading_error + math.atan2(self.k * cross_track_error, v)
        steer_deg = max(-60, min(60, math.degrees(steer_angle)))

        self.get_logger().info(f"Cross-track error: {cross_track_error:.2f} m, Steering angle: {steer_deg:.2f}°")

        self.current_waypoint_index = nearest_index
        return steer_deg

    def navigate(self):
        if not self.navigation_enabled:
            return

        if None in (self.current_lat, self.current_lon, self.heading):
            return

        if len(self.waypoints) < 2:
            self.get_logger().warn("No valid waypoints loaded.")
            return

        heading_deg = -(self.heading - 90) % 360.0
        yaw_rad = math.radians(heading_deg)

        # Check if we are close to the final waypoint
        final_lat, final_lon = self.waypoints[-1]
        dx_final, dy_final = self.latlon_to_xy(self.current_lat, self.current_lon, final_lat, final_lon)
        dist_to_final = math.hypot(dx_final, dy_final)

        # Also check if we've passed most of the path to avoid triggering at the start of a loop
        progress_ratio = self.current_waypoint_index / len(self.waypoints)

        if dist_to_final < 1.0 and progress_ratio > 0.8:
            self.get_logger().info(f"Navigation complete — reached within {dist_to_final:.2f}m of the final waypoint.")
            # Stop the robot
            stop_msg = Int32MultiArray()
            stop_msg.data = [0, 0]
            self.publisher.publish(stop_msg)
            self.navigation_enabled = False
            self._finish_navigation("All waypoints covered")
            return

        v = self.speed / 1000
        steering_angle = self.stanley_control(self.current_lat, self.current_lon, yaw_rad, v=v)
        self.control_robot(steering_angle)

    def control_robot(self, steering_angle):
        max_steering_units = 576 / 2
        max_steering_angle = 60.0

        steering = int(max_steering_units * steering_angle / max_steering_angle)

        speed = int(self.speed * self.speed_factor)
        msg = Int32MultiArray()
        msg.data = [speed, steering]
        self.publisher.publish(msg)
        self.get_logger().info(f"Command: Speed {self.speed}, Steering {steering}")

    # ==================== GPS Logger ====================
    def _finish_navigation(self, reason: str):
        """Called when navigation ends (complete or manual stop). Saves GPS log and comparison plot."""
        self.navigation_complete = True
        self.get_logger().info(f"Navigation ended ({reason}). Recorded {len(self.actual_gps_points)} GPS points.")

        if not self.perimeter_name:
            self.get_logger().warn("No perimeter_name set — cannot save GPS log.")
            return

        folder = os.path.join(self.perimeter_dir, self.perimeter_name)
        os.makedirs(folder, exist_ok=True)

        # --- Save actual GPS log ---
        actual_csv = os.path.join(folder, 'actual_gps_log.csv')
        with open(actual_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["lat", "lon"])
            for lat, lon in self.actual_gps_points:
                w.writerow([f"{lat:.9f}", f"{lon:.9f}"])
        self.get_logger().info(f"Saved actual GPS log: {actual_csv} ({len(self.actual_gps_points)} pts)")

        # --- Load planned waypoints (raw, non-interpolated) ---
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
            self.get_logger().info(f"Loaded {len(planned_pts)} planned waypoints for comparison.")

        # --- Plot comparison ---
        if len(self.actual_gps_points) < 2:
            self.get_logger().warn("Not enough actual points to plot.")
            return

        try:
            fig, ax = plt.subplots(figsize=(10, 10))

            # Plot planned path
            if planned_pts:
                plan_lats, plan_lons = zip(*planned_pts)
                ax.plot(plan_lons, plan_lats, 'b-', lw=2, label='Planned Path', alpha=0.7)
                ax.scatter(plan_lons[0], plan_lats[0], c='blue', s=100, marker='^', zorder=5, label='Planned Start')
                ax.scatter(plan_lons[-1], plan_lats[-1], c='blue', s=100, marker='v', zorder=5, label='Planned End')

            # Plot actual path
            act_lats, act_lons = zip(*self.actual_gps_points)
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
            fig.savefig(png_path, dpi=150, bbox_inches='tight')
            plt.close(fig)
            self.get_logger().info(f"Comparison plot saved: {png_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate plot: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationAndControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()