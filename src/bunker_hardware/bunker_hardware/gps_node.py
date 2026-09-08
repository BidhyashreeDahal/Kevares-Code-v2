import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2
import serial.tools.list_ports
import time
import csv
import os
import json  # Import JSON library


class GPSDataPublisher(Node):
    def __init__(self):
        super().__init__('gps_data_publisher')

        # Set up publisher for GPS data
        self.gps_publisher = self.create_publisher(NavSatFix, '/gps/gps_plugin/out', 10)

        self.heading_publisher = self.create_publisher(Float64, '/gps_heading', 10)

        # Log directory is now a parameter instead of a hardcoded path
        self.declare_parameter('log_dir', os.path.expanduser('~/Kevares-Code-v2/logs/gps'))
        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        os.makedirs(self.log_dir, exist_ok=True)

        # Set up serial port for u-blox GPS
        ports = serial.tools.list_ports.comports()
        self.port = None
        for port in ports:
            if port.manufacturer == 'u-blox AG - www.u-blox.com':
                self.port = port.device
                break

        if self.port is None:
            self.get_logger().error("u-blox GPS device not found.")
            return

        self.baudrate = 9600  # Adjust as per your setup
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.001)

        self.last_heading = 0.0  # Initialize with a default heading

        # Start the timer to publish GPS data at 1Hz (every second)
        self.timer = self.create_timer(0.001, self.publish_gps_data)

    def read_nmea_sentence(self):
        line = self.ser.readline().decode('ascii', errors='replace').strip()
        if line.startswith('$'):
            try:
                nmea_sentence = pynmea2.parse(line)
                return nmea_sentence
            except pynmea2.ParseError as e:
                self.get_logger().warn(f"Error parsing NMEA sentence: {e}")
        return None

    def publish_gps_data(self):
        Latitude = 0.0
        Longitude = 0.0
        heading = 0.0
        Altitude = 0.0
        HDOP = 0.0
        FixQuality = 0.0
        NumSats = 0.0
        Timestamp = 0.0

        sentence = self.read_nmea_sentence()

        if sentence and isinstance(sentence, pynmea2.types.talker.GGA):
            Latitude = sentence.latitude
            Longitude = sentence.longitude

            Altitude = sentence.altitude
            HDOP = sentence.horizontal_dil
            FixQuality = sentence.gps_qual
            NumSats = sentence.num_sats
            Timestamp = time.time()

            # Create and populate NavSatFix message
            navsat_msg = NavSatFix()
            navsat_msg.header.stamp = self.get_clock().now().to_msg()
            navsat_msg.header.frame_id = "gps_frame"  # Replace with your frame ID
            navsat_msg.latitude = Latitude
            navsat_msg.longitude = Longitude

            # Publish the NavSatFix message
            self.gps_publisher.publish(navsat_msg)

        if sentence and isinstance(sentence, pynmea2.types.talker.VTG):  # or VTG
            heading = sentence.true_track  # in degrees
            if heading is None:
                self.get_logger().warn("Heading is None, using last known heading")
                heading = self.last_heading
            else:
                self.last_heading = heading

            heading_msg = Float64()
            heading_msg.data = float(heading)

            self.heading_publisher.publish(heading_msg)

        # Save GPS data to JSON file
        gps_data = {
            "latitude": Latitude,
            "longitude": Longitude,
            "timestamp": time.time()
        }
        with open(os.path.join(self.log_dir, "gps_data.json"), "w") as json_file:
            json.dump(gps_data, json_file)

        # Append to CSV
        csv_path = os.path.join(self.log_dir, "gps_data_log.csv")
        file_exists = os.path.isfile(csv_path)

        with open(csv_path, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            if not file_exists:
                csv_writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude', 'hdop', 'fix_quality', 'num_sats'])
            csv_writer.writerow([Timestamp, Latitude, Longitude, Altitude, HDOP, FixQuality, NumSats])


def main(args=None):
    rclpy.init(args=args)
    node = GPSDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("GPS Data Publisher Node terminated by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()