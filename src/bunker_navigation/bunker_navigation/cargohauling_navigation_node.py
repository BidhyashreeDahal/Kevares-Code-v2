import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float64, Float32, String
from sensor_msgs.msg import NavSatFix
import math, csv, os

class CargoHaulPathFollower(Node):
    """
    Exact path follower (no zig-zag coverage):
      - Tracks the recorded path segment-by-segment (i -> i+1)
      - Target point stays ON the current segment (small lookahead ahead on segment)
      - Switch to next segment only when endpoint reached (within arrival radius)

    Subscriptions:
      /gps/gps_plugin/out    : NavSatFix (latitude, longitude)
      /gps_heading           : Float64   (degrees, 0°=North, clockwise positive)
      /obstacle_speed_factor : Float32   (0.0..1.0)
      /stanley_start_cmd     : String    ("start"/"stop")
      /path_name             : String    (folder inside path_dir)

    Publication:
      movement_commands      : Int32MultiArray [speed, steering_units]
                               (±60° maps to ±576 units)
    """

    def __init__(self):
        super().__init__('cargo_haul_path_follower')

        # === I/O ===
        self.pub_cmd = self.create_publisher(Int32MultiArray, 'movement_commands', 10)
        self.create_subscription(NavSatFix, '/gps/gps_plugin/out', self.gps_cb, 10)
        self.create_subscription(Float64, '/gps_heading', self.heading_cb, 10)
        self.create_subscription(Float32, '/obstacle_speed_factor', self.speed_factor_cb, 10)
        self.create_subscription(String, '/cargo_start_cmd', self.start_cb, 10)
        self.create_subscription(String, '/path_name', self.path_name_cb, 10)

        # === Files / waypoints ===
        self.path_dir = "/home/user/bunker_ws/src/bunker_core/bunker_core/path"
        self.path_name = None
        self.waypoints = []  # [(lat, lon), ...]

        # === Robot state ===
        self.lat = None
        self.lon = None
        self.heading_deg_north_cw = None  # 0 = North, clockwise +

        # === Command scaling ===
        self.base_speed = 500
        self.speed_factor = 1.0
        self.max_steer_units = 576
        self.max_steer_deg = 60.0

        # === Segment-follow params (minimal) ===
        self.seg_index = 0            # track segment i -> i+1
        self.loop_path = True         # wrap from last to first
        self.arrival_radius = 0.8     # meters to consider segment end reached
        self.lookahead_m = 0.4        # meters ahead ON the current segment
        self.wheelbase_m = 0.8        # for pure-pursuit geometry
        self.started_on_path = False  # snap once at start

        self.navigation_enabled = False
        self.create_timer(0.02, self.navigate)  # 50 Hz

    # ---------- Subscriptions ----------
    def path_name_cb(self, msg: String):
        self.path_name = msg.data.strip()
        path = os.path.join(self.path_dir, self.path_name, 'final_combined_waypoints.csv')
        if not os.path.exists(path):
            self.get_logger().warn(f"Waypoint file not found: {path}")
            return
        self.waypoints = self.load_waypoints(path)
        if len(self.waypoints) < 2:
            self.get_logger().warn('Loaded waypoint list is too short.')
        else:
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {path}")
        self.started_on_path = False
        self.seg_index = 0

    def start_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self.started_on_path = False
            self.seg_index = 0
            self.navigation_enabled = True
            self.get_logger().info('Cargo-hauling exact follow: START')
        elif cmd == 'stop':
            self.navigation_enabled = False
            self.get_logger().info('Cargo-hauling exact follow: STOP')

    def speed_factor_cb(self, msg: Float32):
        self.speed_factor = max(0.0, min(1.0, float(msg.data)))

    def gps_cb(self, msg: NavSatFix):
        self.lat = float(msg.latitude)
        self.lon = float(msg.longitude)

    def heading_cb(self, msg: Float64):
        self.heading_deg_north_cw = float(msg.data)

    # ---------- Waypoint loading ----------
    def load_waypoints(self, filepath):
        """Load lon,lat CSV as-is (no densification) to preserve exact geometry."""
        pts = []
        with open(filepath, 'r') as f:
            for row in csv.reader(f):
                try:
                    lon = float(row[0]); lat = float(row[1])
                    pts.append((lat, lon))
                except Exception:
                    continue
        return pts

    # ---------- Geometry helpers ----------
    @staticmethod
    def wrap_heading_to_yaw_rad(heading_deg_north_cw: float) -> float:
        """Convert 0°=North (CW +) to ENU yaw: 0°=East (CCW +)."""
        heading_deg = (-(heading_deg_north_cw - 90.0)) % 360.0
        return math.radians(heading_deg)

    @staticmethod
    def xy_from_latlon(lat: float, lon: float, ref_lat: float, ref_lon: float):
        avg_lat = math.radians((lat + ref_lat) / 2.0)
        dx = (lon - ref_lon) * 111000.0 * math.cos(avg_lat)
        dy = (lat - ref_lat) * 111000.0
        return dx, dy

    @staticmethod
    def lerp_latlon(a_lat, a_lon, b_lat, b_lon, t: float):
        return (a_lat + t * (b_lat - a_lat), a_lon + t * (b_lon - a_lon))

    def project_onto_segment(self, p_lat, p_lon, a_lat, a_lon, b_lat, b_lon):
        """Return (t_clamped, seg_len_m, proj_lat, proj_lon) with t in [0,1] along A->B."""
        ax, ay = 0.0, 0.0
        bx, by = self.xy_from_latlon(b_lat, b_lon, a_lat, a_lon)
        px, py = self.xy_from_latlon(p_lat, p_lon, a_lat, a_lon)
        vx, vy = bx - ax, by - ay
        v2 = vx * vx + vy * vy
        if v2 < 1e-6:
            return 0.0, 0.0, a_lat, a_lon
        t = ((px - ax) * vx + (py - ay) * vy) / v2
        t = max(0.0, min(1.0, t))
        seg_len = math.sqrt(v2)
        proj_lat, proj_lon = self.lerp_latlon(a_lat, a_lon, b_lat, b_lon, t)
        return t, seg_len, proj_lat, proj_lon

    # ---------- Start helper ----------
    def snap_to_nearest_segment_once(self):
        if self.started_on_path or len(self.waypoints) < 2 or self.lat is None:
            return
        best_i, best_d = 0, float('inf')
        for i in range(len(self.waypoints) - 1):
            alat, alon = self.waypoints[i]
            blat, blon = self.waypoints[i + 1]
            t, seg_len, proj_lat, proj_lon = self.project_onto_segment(self.lat, self.lon, alat, alon, blat, blon)
            dx, dy = self.xy_from_latlon(self.lat, self.lon, proj_lat, proj_lon)
            d = math.hypot(dx, dy)
            if d < best_d:
                best_d, best_i = d, i
        self.seg_index = best_i
        self.started_on_path = True
        self.get_logger().info(f"Snapped to segment {best_i}->{best_i+1} (dist {best_d:.2f} m)")

    # ---------- Core control ----------
    def follow_segment(self, yaw_rad):
        n = len(self.waypoints)
        if n < 2:
            return 0.0, int(self.base_speed * self.speed_factor)

        i = self.seg_index
        alat, alon = self.waypoints[i]
        blat, blon = self.waypoints[(i + 1) % n]

        # Project onto active segment
        t, seg_len, proj_lat, proj_lon = self.project_onto_segment(self.lat, self.lon, alat, alon, blat, blon)
        dist_to_end = max(0.0, seg_len * (1.0 - t))

        # Advance segment if close to end
        if dist_to_end <= self.arrival_radius:
            self.seg_index = (i + 1) % n if self.loop_path else min(i + 1, n - 2)
            # recompute with new segment
            i = self.seg_index
            alat, alon = self.waypoints[i]
            blat, blon = self.waypoints[(i + 1) % n]
            t, seg_len, proj_lat, proj_lon = self.project_onto_segment(self.lat, self.lon, alat, alon, blat, blon)
            dist_to_end = max(0.0, seg_len * (1.0 - t))

        # Target ahead ON the segment (clamped)
        ahead = min(self.lookahead_m, dist_to_end)
        t_target = 0.0 if seg_len < 1e-6 else min(1.0, t + ahead / max(seg_len, 1e-6))
        tlat, tlon = self.lerp_latlon(alat, alon, blat, blon, t_target)

        # Transform target into robot frame (x forward, y left)
        dx, dy = self.xy_from_latlon(self.lat, self.lon, tlat, tlon)
        x_fwd =  math.cos(yaw_rad) * dx + math.sin(yaw_rad) * dy
        y_left = -math.sin(yaw_rad) * dx + math.cos(yaw_rad) * dy
        Ld = max(0.1, math.hypot(x_fwd, y_left))

        # Pure-pursuit steering
        kappa = 2.0 * y_left / (Ld * Ld)
        steer_rad = math.atan(kappa * self.wheelbase_m)
        steer_deg = max(-self.max_steer_deg, min(self.max_steer_deg, math.degrees(steer_rad)))

        speed_cmd = int(self.base_speed * self.speed_factor)
        return steer_deg, speed_cmd

    # ---------- Main loop ----------
    def navigate(self):
        if not self.navigation_enabled:
            return
        if None in (self.lat, self.lon, self.heading_deg_north_cw):
            return
        if len(self.waypoints) < 2:
            self.get_logger().warn('No valid waypoints loaded.')
            return

        yaw_rad = self.wrap_heading_to_yaw_rad(self.heading_deg_north_cw)
        self.snap_to_nearest_segment_once()

        steer_deg, speed_cmd = self.follow_segment(yaw_rad)
        steering_units = int(self.max_steer_units * (steer_deg / self.max_steer_deg))

        msg = Int32MultiArray()
        msg.data = [int(speed_cmd), int(steering_units)]
        self.pub_cmd.publish(msg)
        self.get_logger().info(f"[Seg {self.seg_index}] Cmd: Speed {speed_cmd}, Steering {steering_units} (deg {steer_deg:.1f})")

def main(args=None):
    rclpy.init(args=args)
    node = CargoHaulPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
