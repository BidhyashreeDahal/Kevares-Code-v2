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
        self.declare_parameter('color_width', 1280)
        self.declare_parameter('color_height', 720)
        self.declare_parameter('depth_width', 1280)
        self.declare_parameter('depth_height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_period', 1/30)
        serial_number = self.get_parameter('camera_serial_number').get_parameter_value().string_value
        color_width = self.get_parameter('color_width').get_parameter_value().integer_value
        color_height = self.get_parameter('color_height').get_parameter_value().integer_value
        depth_width = self.get_parameter('depth_width').get_parameter_value().integer_value
        depth_height = self.get_parameter('depth_height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.config.enable_device(serial_number)

        self.config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, fps)

        try:
            self.pipeline.start(self.config)
        except RuntimeError as e:
            self.get_logger().error(f"Failed to start RealSense camera (serial {serial_number}): {e}")
            raise

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        publish_period = self.get_parameter('publish_period').get_parameter_value().double_value
        self.timer = self.create_timer(publish_period, self.publish_images)

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