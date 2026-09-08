import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import can
import struct

class CanCommandSubscriber(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'movement_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Initialize CAN bus
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('bitrate', 500000)
        self.declare_parameter('can_send_timeout', 0.01)
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value
        self.can_send_timeout = self.get_parameter('can_send_timeout').get_parameter_value().double_value
        try:
            self.bus = can.interface.Bus(bustype='socketcan', channel=self.can_interface, bitrate=self.bitrate)
        except OSError as e:
            self.get_logger().error(f"CAN interface {self.can_interface} initialization failed: {e}")
            raise RuntimeError(f"CAN interface {self.can_interface} initialization failed")

        self.activate_can_mode()
        self.get_logger().info('CAN mode activated')

    def activate_can_mode(self):
        message = can.Message(arbitration_id=0x421, data=[0x01], is_extended_id=False)
        self.bus.send(message)
        self.get_logger().info('Sent CAN mode activation command')

    def listener_callback(self, msg):
        linear_speed, steer = msg.data
        self.send_movement_command(self.bus, linear_speed, steer)

    def send_movement_command(self, bus, linear_speed, steer):
        data = struct.pack('>hh4x', linear_speed, steer)  # '4x' adds four zero bytes '>h4xh for hunter and >hh4x for bunker pro'
        message = can.Message(arbitration_id=0x111, data=data, is_extended_id=False)
        bus.send(message, timeout=self.can_send_timeout)
        self.get_logger().info(f'Sent linear {linear_speed} mm/s, steer {steer}')

    def destroy_node(self):
        self.get_logger().info('Shutting down CAN bus interface')
        self.bus.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CanCommandSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()