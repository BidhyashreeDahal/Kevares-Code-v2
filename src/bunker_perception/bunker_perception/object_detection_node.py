#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import csv
import os

try:
    from ultralytics import YOLO
except ImportError:
    # If ultralytics is not installed, we'll log an error later but allow the node to be imported.
    YOLO = None

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.declare_parameter('model_path', '/home/bidya/Kevares-Code-v2/models/exp.pt')
        self.declare_parameter('image_topic', 'color_image')
        self.declare_parameter('output_csv', '/home/bidya/Kevares-Code-v2/logs/detection_statistics.csv')
        self.declare_parameter('log_every_n_frames', 30)

        model_path = self.get_parameter('model_path').value
        image_topic = self.get_parameter('image_topic').value
        self.output_csv = self.get_parameter('output_csv').value
        self.log_every_n_frames = self.get_parameter('log_every_n_frames').get_parameter_value().integer_value

        if YOLO is None:
            self.get_logger().error("ultralytics library is not installed. Please install it using 'pip install ultralytics'")
            raise RuntimeError("ultralytics library is not installed")

        self.get_logger().info(f"Loading YOLO model from {model_path}...")
        try:
            self.model = YOLO(model_path)
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise RuntimeError(f"Failed to load YOLO model from {model_path}: {e}")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(Image, 'object_detection/annotated_image', 10)

        # Initialize CSV file
        os.makedirs(os.path.dirname(self.output_csv), exist_ok=True)
        self.init_csv()

        self.frame_count = 0
        self.start_time = time.time()

    def init_csv(self):
        file_exists = os.path.isfile(self.output_csv)
        with open(self.output_csv, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(['timestamp', 'frame_number', 'fps', 'total_detections', 'class_counts'])
        self.get_logger().info(f"Logging statistics to {self.output_csv}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        inference_start = time.time()

        # Run inference
        results = self.model(cv_image, verbose=False)

        inference_time = time.time() - inference_start
        fps = 1.0 / inference_time if inference_time > 0 else 0.0

        self.frame_count += 1

        total_detections = 0
        class_counts = {}

        # Process results
        for r in results:
            total_detections += len(r.boxes)
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                class_counts[cls_name] = class_counts.get(cls_name, 0) + 1

        # Draw annotations
        annotated_frame = results[0].plot()

        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.publisher_.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Could not publish image: {e}")

        # Log to CSV
        timestamp = time.time()
        with open(self.output_csv, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, self.frame_count, f"{fps:.2f}", total_detections, str(class_counts)])

        if self.frame_count % self.log_every_n_frames == 0:
            self.get_logger().info(f"Frame {self.frame_count}: FPS: {fps:.1f}, Detections: {total_detections}, Stats: {class_counts}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ObjectDetectionNode()
    except RuntimeError as e:
        print(f"Failed to start object_detection_node: {e}")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()