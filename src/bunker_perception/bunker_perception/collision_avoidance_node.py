# === collision_avoidance_node.py ===
#
# WARNING: This node publishes directly to 'movement_commands' (emergency stop),
# the same topic used by stanlynavigation_node and read by robot_controller_node.
# This is the multi-node command conflict flagged in the Aug 11 architecture meeting.
# Do NOT add this node to any active launch file until command arbitration
# (motion authority / single publisher to movement_commands) is resolved.
#
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32, String
from cv_bridge import CvBridge
import numpy as np

class DodgingNode(Node):
    def __init__(self):
        super().__init__('collision_avoidance_node')

        self.subscription_depth = self.create_subscription(Image, '/depth_image', self.depth_callback, 10)
        self.subscription_right_depth = self.create_subscription(Image, '/right_depth_image', self.right_depth_callback, 10)
        self.subscription_left_depth = self.create_subscription(Image, '/left_depth_image', self.left_depth_callback, 10)

        self.publisher = self.create_publisher(Int32MultiArray, 'movement_commands', 10)

        self.publisher_factor = self.create_publisher(Float32, '/obstacle_speed_factor', 10)
        self.publisher_steering_left = self.create_publisher(Float32, '/steering_correction_left', 10)
        self.publisher_steering_right = self.create_publisher(Float32, '/steering_correction_right', 10)
        self.publisher_steering_center = self.create_publisher(Float32, '/steering_correction_center', 10)

        self.bridge = CvBridge()
        self.depth_frame = None
        self.left_depth_frame = None
        self.right_depth_frame = None

        self.get_logger().info("DodgingNode initialized")

        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 2.0)
        self.declare_parameter('max_steering_angle', 0.75)
        self.declare_parameter('min_speed_factor', 0.0)
        self.declare_parameter('emergency_stop_distance', 0.5)
        self.declare_parameter('stop_speed_distance', 0.7)
        self.declare_parameter('slow_speed_distance', 1.6)
        self.declare_parameter('front_min_valid_points', 50)
        self.declare_parameter('side_min_valid_points', 50)
        self.declare_parameter('side_sample_count', 200)
        self.min_depth = self.get_parameter('min_depth').get_parameter_value().double_value
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.min_speed_factor = self.get_parameter('min_speed_factor').get_parameter_value().double_value
        self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').get_parameter_value().double_value
        self.stop_speed_distance = self.get_parameter('stop_speed_distance').get_parameter_value().double_value
        self.slow_speed_distance = self.get_parameter('slow_speed_distance').get_parameter_value().double_value
        self.front_min_valid_points = self.get_parameter('front_min_valid_points').get_parameter_value().integer_value
        self.side_min_valid_points = self.get_parameter('side_min_valid_points').get_parameter_value().integer_value
        self.side_sample_count = self.get_parameter('side_sample_count').get_parameter_value().integer_value

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Failed to convert front depth image: {e}")
            return

        self.process_depth_image()

    def left_depth_callback(self, msg):
        try:
            self.left_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f"Left depth conversion failed: {e}")

    def right_depth_callback(self, msg):
        try:
            self.right_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f"Right depth conversion failed: {e}")

    def process_depth_image(self):
        front_valid = False
        steering_center = 0.0
        speed_factor = 1.0

        if self.depth_frame is not None:
            height, width = self.depth_frame.shape
            h_start = int(height * 0.5)
            h_end = int(height * 0.6)
            w_start = int(width * 0)
            w_end = int(width * 1)
            region = self.depth_frame[h_start:h_end, w_start:w_end]
            valid_mask = (region > self.min_depth) & (region < self.max_depth) & np.isfinite(region)

            if np.count_nonzero(valid_mask) > self.front_min_valid_points:
                y_idxs, x_idxs = np.where(valid_mask)
                depths = region[y_idxs, x_idxs]
                min_dist = float(np.min(depths))

                if min_dist < self.emergency_stop_distance:
                    # Emergency stop
                    self.publisher.publish(Int32MultiArray(data=[0, 0]))
                    self.get_logger().warn("Emergency stop! Obstacle too close.")
                    return

                mean_x = np.mean(x_idxs)
                center_x = region.shape[1] / 2
                offset = center_x - mean_x
                offset_norm = offset / center_x

                if min_dist < self.max_depth:
                    strength = (self.max_depth - min_dist) / self.max_depth
                    steering_center = offset_norm * self.max_steering_angle * strength
                    steering_center = float(np.clip(steering_center, -self.max_steering_angle, self.max_steering_angle))

                if min_dist < self.stop_speed_distance:
                    speed_factor = self.min_speed_factor
                elif min_dist < self.slow_speed_distance:
                    speed_factor = max(self.min_speed_factor, (min_dist - self.min_depth) / (self.slow_speed_distance - self.min_depth))
                else:
                    speed_factor = 1.0

                front_valid = True

        self.publisher_factor.publish(Float32(data=speed_factor))

        if front_valid:
            self.publisher_steering_center.publish(Float32(data=steering_center))
            return

        left_angle = self.calculate_side_steering(self.left_depth_frame, is_left=True)
        right_angle = self.calculate_side_steering(self.right_depth_frame, is_left=False)

        if left_angle > 0:
            self.publisher_steering_left.publish(Float32(data=left_angle))
        if right_angle > 0:
            self.publisher_steering_right.publish(Float32(data=right_angle))

    def calculate_side_steering(self, frame, is_left):
        if frame is None:
            return 0.0

        try:
            height, width = frame.shape
            region = frame[int(height * 0.25):int(height * 0.75), int(width * 0.25):int(width * 0.75)]
            valid = (region > self.min_depth) & (region < self.max_depth) & np.isfinite(region)
            if np.count_nonzero(valid) < self.side_min_valid_points:
                return 0.0
            depths = region[valid]
            avg_depth = float(np.mean(np.sort(depths.flatten())[:self.side_sample_count]))
            if avg_depth < 1.0:
                strength = (1.0 - avg_depth) / 1.0
                return strength * self.max_steering_angle
            return 0.0
        except Exception as e:
            self.get_logger().warn(f"Failed to process {'left' if is_left else 'right'} side: {e}")
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = DodgingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()