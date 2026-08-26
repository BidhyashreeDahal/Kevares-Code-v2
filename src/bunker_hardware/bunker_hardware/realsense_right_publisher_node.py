import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge

class RealSensePublisherRight(Node):

    def __init__(self):
        super().__init__('realsense_publisher_right')
        self.publisher_color = self.create_publisher(Image, 'right_color_image', 10)
        self.publisher_depth = self.create_publisher(Image, 'right_depth_image', 10)
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.declare_parameter('camera_serial_number', '242322073736')
        serial_number = self.get_parameter('camera_serial_number').get_parameter_value().string_value
        self.config.enable_device(serial_number)

        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        try:
            self.pipeline.start(self.config)
        except RuntimeError as e:
            self.get_logger().error(f"Failed to start RealSense camera (serial {serial_number}): {e}")
            raise

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.timer = self.create_timer(1/30, self.publish_images)

    def publish_images(self):
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        color_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, '16UC1')

        self.publisher_color.publish(color_msg)
        self.publisher_depth.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePublisherRight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()