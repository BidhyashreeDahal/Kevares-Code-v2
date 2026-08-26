import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import numpy as np
import serial
import serial.tools.list_ports

class DodgingNode(Node):
    def __init__(self):
        super().__init__('dodging_node')

        # Update this if your camera uses a different topic
        self.subscription_depth = self.create_subscription(Image, 'depth_image', self.depth_callback, 10)

        self.publisher_factor = self.create_publisher(Float32, '/obstacle_speed_factor', 10)

        # Publisher for the alert system
        self.publisher_alert = self.create_publisher(String, '/obstacle_alert', 10)

        self.subscription_can = self.create_subscription(
            String,
            'can_feedback',
            self.listener_callback_can,
            10)

        self.swd_enabled = False  # False means obstacle avoidance is active

        self.bridge = CvBridge()
        self.depth_frame = None

        # === Arduino Serial Setup ===
        self.declare_parameter('arduino_serial_number', '55834323833351A07090')
        self.arduino_serial_number = self.get_parameter('arduino_serial_number').get_parameter_value().string_value
        self.arduino_port = self.find_arduino_port(self.arduino_serial_number)
        self.ser = None

        if self.arduino_port:
            try:
                self.ser = serial.Serial(self.arduino_port, 9600)
                self.get_logger().info(f"Connected to Arduino on port {self.arduino_port}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial connection failed: {e}")
        else:
            self.get_logger().warn("Arduino with the specified serial number not found.")

    def find_arduino_port(self, serial_number):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return port.device
        return None

    def depth_callback(self, msg):
        try:
            # RealSense usually gives 16UC1 → convert to meters
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return
        self.process_depth_image()

    def process_depth_image(self):
        if self.depth_frame is None:
            return

        if self.swd_enabled:
            # Bypass depth processing when SWD is ON
            self.get_logger().info("SWD active — skipping obstacle check")
            return

        # Convert to meters if 16-bit
        if self.depth_frame.dtype == np.uint16:
            depth_image = self.depth_frame.astype(np.float32) / 1000.0
        else:
            depth_image = self.depth_frame

        height, width = depth_image.shape
        region = depth_image[int(height * 0.6):int(height * 0.65), :]

        # Filter: depth > 0.2 and < 2.0 meters
        valid_depths = region[(region > 0.2) & (region < 2.0) & np.isfinite(region)]

        if valid_depths.size == 0:
            factor = 1.0
            self.control_led('STEADY')
            self.get_logger().warn("No valid depth points found. Defaulting to factor = 1.0")

        else:
            sorted_depths = np.sort(valid_depths.flatten())
            top_depths = sorted_depths[:500] if sorted_depths.size >= 500 else sorted_depths
            min_dist = float(np.mean(top_depths))

            high = 1.5
            low = 0.8

            if min_dist < low:
                factor = 0.0
                self.control_led('BLINK')
                alert_msg = String()
                alert_msg.data = "speed_zero"
                self.publisher_alert.publish(alert_msg)

            elif low <= min_dist <= high:
                factor = (min_dist - low) / (high - low)  # Linear scale 1.0m → 2.0m
                self.control_led('BLINK')
                alert_msg = String()
                alert_msg.data = "speed_normal"
                self.publisher_alert.publish(alert_msg)

            else:
                factor = 1.0
                self.control_led('STEADY')
                alert_msg = String()
                alert_msg.data = "speed_normal"
                self.publisher_alert.publish(alert_msg)

            factor = max(0.0, min(1.0, factor))  # Ensure within bounds
            self.get_logger().info(f"Min depth: {min_dist:.2f} m → Speed factor: {factor:.2f}")

        self.publisher_factor.publish(Float32(data=factor))

    def listener_callback_can(self, msg):
        self.get_logger().info(f"Received CAN data: {msg.data}")

        try:
            data = json.loads(msg.data)
            arbitration_id = data.get("id")
            if arbitration_id == "0x241":
                swd = data.get("swd")
                self.get_logger().info(f"swd: {swd}")
                if swd == 1:
                    self.swd_enabled = True
                    self.publisher_factor.publish(Float32(data=1.0))
                    self.get_logger().info("SWD ON - Obstacle avoidance bypassed, speed factor = 1.0")
                else:
                    self.swd_enabled = False
                    self.get_logger().info("SWD OFF - Obstacle avoidance re-enabled")

        except Exception as e:
            self.get_logger().error(f"Error processing CAN data: {e}")

    def control_led(self, command):
        if self.ser and self.ser.is_open:
            if command == 'BLINK':
                self.ser.write(b'BLINK\n')
            elif command == 'STEADY':
                self.ser.write(b'STEADY\n')
            self.get_logger().info(f"LED command sent: {command}")

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