import rclpy
from rclpy.node import Node
import can
import json
from std_msgs.msg import String

class CanFeedbackNode(Node):
    def __init__(self):
        super().__init__('can_feedback_node')
        # Initialize CAN interface
        self.declare_parameter('can_interface', 'can0')
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        try:
            self.bus = can.interface.Bus(self.can_interface, bustype='socketcan', bitrate=500000)
        except OSError as e:
            self.get_logger().error(f"CAN interface {self.can_interface} initialization failed: {e}")
            raise RuntimeError(f"CAN interface {self.can_interface} initialization failed")

        self.publisher_ = self.create_publisher(String, 'can_feedback', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("CAN Feedback node started")

    def timer_callback(self):
        try:
            while True:
                message = self.bus.recv()  # Timeout of 0.01 seconds to allow periodic checking
                self.process_message(message)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def process_message(self, message):
        data = message.data

        if (message.arbitration_id) == int('0x251', 16):
            motor_speed = int.from_bytes(data[0:2], byteorder='big', signed=True)
            motor_current = int.from_bytes(data[2:4], byteorder='big', signed=True)
            motor_position = int.from_bytes(data[4:8], byteorder='big', signed=True)
            motor_speed_corrected = motor_speed  # assuming the offset is +1500
            output = {
                "id": "0x251",
                "motor_speed": motor_speed_corrected,
                "motor_current": motor_current,
                "motor_position": motor_position
            }
            self.publish_data(output)
            # self.get_logger().info(f"Received CAN message: {output}")

        if (message.arbitration_id) == int('0x261', 16):
            drive_voltage_raw = int.from_bytes(data[0:2], byteorder='big', signed=False)
            drive_voltage = drive_voltage_raw * 0.1
            drive_temperature_raw = int.from_bytes(data[2:4], byteorder='big', signed=True)
            drive_temperature = drive_temperature_raw * 1
            motor_temperature = int.from_bytes(data[4:5], byteorder='big', signed=True)
            motor_temperature = motor_temperature * 1
            drive_status = int.from_bytes(data[5:6], byteorder='big', signed=False)
            output = {
                "id": "0x261",
                "drive_voltage": drive_voltage,
                "drive_temperature": drive_temperature,
                "motor_temperature": motor_temperature,
                "drive_status": drive_status
            }
            self.publish_data(output)
            # self.get_logger().info(f"Received CAN message: {output}")

        if (message.arbitration_id) == int('0x311', 16):
            left_wheel_mileometer = int.from_bytes(data[0:4], byteorder='big', signed=True)
            right_wheel_mileometer = int.from_bytes(data[4:8], byteorder='big', signed=True)
            output = {
                "id": "0x311",
                "left_wheel_mileometer": left_wheel_mileometer,
                "right_wheel_mileometer": right_wheel_mileometer
            }
            # self.publish_data(output)
            # self.get_logger().info(f"Received CAN message: {output}")

        if (message.arbitration_id) == int('0x241', 16):
            sw_feedback = data[0]
            swa = (sw_feedback >> 0) & 0x01
            swb = (sw_feedback >> 2) & 0x03
            swc = (sw_feedback >> 4) & 0x03
            swd = (sw_feedback >> 6) & 0x01
            right_joystick_lr = int.from_bytes(data[1:2], byteorder='big', signed=True)
            right_joystick_ud = int.from_bytes(data[2:3], byteorder='big', signed=True)
            left_joystick_ud = int.from_bytes(data[3:4], byteorder='big', signed=True)
            left_joystick_lr = int.from_bytes(data[4:5], byteorder='big', signed=True)
            left_knob_vra = int.from_bytes(data[5:6], byteorder='big', signed=True)
            count_parity_bit = data[7]
            output = {
                "id": "0x241",
                "swa": swa,
                "swb": swb,
                "swc": swc,
                "swd": swd,
                "right_joystick_lr": right_joystick_lr,
                "right_joystick_ud": right_joystick_ud,
                "left_joystick_ud": left_joystick_ud,
                "left_joystick_lr": left_joystick_lr,
                "left_knob_vra": left_knob_vra,
                "count_parity_bit": count_parity_bit
            }
            self.publish_data(output)
            # self.get_logger().info(f"Received CAN message: {output}")

        if (message.arbitration_id) == 0x151:
            current_status = data[0]
            mode_control = data[1]
            battery_voltage_high = data[2]
            battery_voltage_low = data[3]
            failure_info = data[5]
            count_parity = data[7]
            battery_voltage = (battery_voltage_high << 8) | battery_voltage_low
            actual_voltage = battery_voltage * 0.1
            output = {
                "id": "0x151",
                "current_status": current_status,
                "mode_control": mode_control,
                "actual_voltage": actual_voltage,
                "failure_info": failure_info,
                "count_parity": count_parity
            }
            self.publish_data(output)
            self.get_logger().info(f"Received CAN message: {output}")

    def publish_data(self, data):
        json_data = json.dumps(data)
        self.publisher_.publish(String(data=json_data))
        # self.get_logger().info(f"Published CAN data: {json_data}")

def main(args=None):
    rclpy.init(args=args)
    node = CanFeedbackNode()
    try:
        rclpy.spin(node)  # Keeps the node running
    except KeyboardInterrupt:
        node.get_logger().info("Script terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
