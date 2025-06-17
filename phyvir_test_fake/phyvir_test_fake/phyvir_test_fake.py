


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random

class phyvir_test_fake(Node):
    def __init__(self):
        super().__init__('fake_lidar_node')

        # Subscribe to the real LIDAR topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publisher for the fake LIDAR topic
        self.publisher = self.create_publisher(LaserScan, '/virtual_scan', 10)

        # Timer to inject discrepancy every 30 seconds
        self.timer = self.create_timer(30.0, self.introduce_discrepancy)

        self.latest_scan = None

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.publisher.publish(msg)

    def introduce_discrepancy(self):
        if self.latest_scan:
            modified_scan = LaserScan()
            modified_scan.header = self.latest_scan.header
            modified_scan.angle_min = self.latest_scan.angle_min
            modified_scan.angle_max = self.latest_scan.angle_max
            modified_scan.angle_increment = self.latest_scan.angle_increment
            modified_scan.time_increment = self.latest_scan.time_increment
            modified_scan.scan_time = self.latest_scan.scan_time
            modified_scan.range_min = self.latest_scan.range_min
            modified_scan.range_max = self.latest_scan.range_max
            modified_scan.ranges = list(self.latest_scan.ranges)
            modified_scan.intensities = list(self.latest_scan.intensities)

            # Simulate an obstacle by modifying 10 consecutive range values
            num_values_to_modify = 10
            start_index = random.randint(0, len(modified_scan.ranges) - num_values_to_modify)
            for i in range(start_index, start_index + num_values_to_modify):
                modified_scan.ranges[i] = 0.5  # Simulated obstacle

            self.publisher.publish(modified_scan)

def main(args=None):
    rclpy.init(args=args)
    node = phyvir_test_fake()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

