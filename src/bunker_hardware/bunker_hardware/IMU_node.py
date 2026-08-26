import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
import serial
import serial.tools.list_ports
import time
import math

def parse_quaternion(line):
    """
    Parse the quaternion data from a JSON-like line.
    Expected format: '"quat_w":1.000, "quat_x":-0.000, "quat_y":-0.001, "quat_z":-0.002'
    """
    try:
        # Clean up the line and split into key-value pairs
        line = line.replace('"', '').strip()
        parts = {item.split(':')[0].strip(): float(item.split(':')[1]) for item in line.split(', ') if ':' in item}

        q0 = parts.get("quat_w", 0.0)
        q1 = parts.get("quat_x", 0.0)
        q2 = parts.get("quat_y", 0.0)
        q3 = parts.get("quat_z", 0.0)

        return q1, q2, q3, q0
    except Exception as e:
        print(f"Error parsing line: {line}. Error: {e}")
        return None

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Publishers
        self.quaternion_publisher = self.create_publisher(Quaternion, 'quaternion', 10)
        self.roll_publisher = self.create_publisher(Float32, '/roll_degrees', 10)
        self.pitch_publisher = self.create_publisher(Float32, '/pitch_degrees', 10)
        self.yaw_publisher = self.create_publisher(Float32, '/yaw_degrees', 10)

        # Serial connection
        self.declare_parameter('arduino_serial_number', '24333313131351A0A121')
        self.arduino_serial_number = self.get_parameter('arduino_serial_number').get_parameter_value().string_value
        self.arduino_port = self.find_arduino_port(self.arduino_serial_number)
        self.ser = None

        if self.arduino_port:
            self.ser = serial.Serial(self.arduino_port, 115200)
            self.get_logger().info(f"Connected to Arduino on port {self.arduino_port}")
        else:
            self.get_logger().error("Arduino with the specified serial number not found.")

        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 10 Hz
        self.buffer = ""  # Buffer to accumulate data

    def find_arduino_port(self, serial_number):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.serial_number == serial_number:
                return port.device  # e.g., '/dev/ttyACM0' or 'COM3'
        return None

    def publish_imu_data(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port is not open.")
            return

        try:
            raw_data = self.ser.read(self.ser.in_waiting or 1).decode('utf-8', errors='ignore')
            self.buffer += raw_data

            # Process complete lines
            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                line = line.strip()
                if line:
                    quaternion = parse_quaternion(line)
                    if quaternion:
                        q1, q2, q3, q0 = quaternion
                        
                        # Publish Quaternion
                        msg = Quaternion()
                        msg.x = q1
                        msg.y = q2
                        msg.z = q3
                        msg.w = q0
                        self.quaternion_publisher.publish(msg)

                        # Convert Quaternion to Roll, Pitch, Yaw
                        roll, pitch, yaw = self.quaternion_to_euler(q0, q1, q2, q3)

                        # Publish Roll
                        roll_msg = Float32()
                        roll_msg.data = roll
                        self.roll_publisher.publish(roll_msg)

                        # Publish Pitch
                        pitch_msg = Float32()
                        pitch_msg.data = pitch
                        self.pitch_publisher.publish(pitch_msg)

                        # Publish Yaw (Theta)
                        yaw_msg = Float32()
                        yaw_msg.data =  yaw
                        #yaw_msg.data = (yaw - 90) % 360
                        self.yaw_publisher.publish(yaw_msg)

                        self.get_logger().info(
                            f"Quaternion: [x: {q1:.3f}, y: {q2:.3f}, z: {q3:.3f}, w: {q0:.3f}] | "
                            f"Roll: {roll:.2f}° | Pitch: {pitch:.2f}° | Yaw: {yaw:.2f}°"
                        )

                    else:
                        self.get_logger().warn(f"Failed to parse line: {line}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def quaternion_to_euler(self, w, x, y, z):
        """Convert a quaternion into roll, pitch, and yaw angles (in degrees)."""
        # Roll (rotation around X-axis)
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        # Pitch (rotation around Y-axis)
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))  # Clamped for numerical stability
        # Yaw (rotation around Z-axis)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        imu_publisher.get_logger().info("Node stopped by user.")
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
