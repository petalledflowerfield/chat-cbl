import numpy as np
from np.linalg import norm

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


class PhyVir(Node):
    subscriber_virtual = None
    subscriber_physical = None
    scan_pub = None
    discrepancy_pub = None

    physical_scan = None
    virtual_scan = None

    def __init__(self):
        super().__init__("phyvir_filter")

        self.subscriber_virtual = self.create_subscription(
            LaserScan,
            "virtual_scan",
            self.sub_callback_virtual,
            qos_profile=qos_profile_sensor_data,
        )
        self.subscriber_physical = self.create_subscription(
            LaserScan,
            "scan",
            self.sub_callback_physical,
            qos_profile=qos_profile_sensor_data,
        )

        self.discrepancy_pub = self.create_publisher(PoseStamped, "discrepancies", 100)

        pub_period = 0.1
        self.timer = self.create_timer(pub_period, self.pub_callback)

    def sub_callback_virtual(self, msg):
        self.virtual_scan = msg
        self.get_logger().info("Got message from virtual scanner")

    def sub_callback_physical(self, msg):
        self.physical_scan = msg
        self.get_logger().info("Got message from physical scanner")

    def find_discrepancies(self, threshold: float) -> PoseStamped:
        discrepancies = []

        phy_scan = self.physical_scan
        vir_scan = self.virtual_scan

        phy_ranges = np.array(phy_scan.ranges)
        vir_ranges = np.array(vir_scan.ranges)

        phy_valid_ranges = phy_ranges[np.isfinite(phy_ranges) & (phy_ranges > 0)]
        vir_valid_ranges = vir_ranges[np.isfinite(vir_ranges) & (vir_ranges > 0)]

        def angle_to_point(distance, scan, angle):
            x = distance * np.cos(scan.angle_min + angle * scan.angle_increment)
            y = distance * np.sin(scan.angle_min + angle * scan.angle_increment)
            return (x, y)

        # Convert into a cartesian plane so we can then give the relative coordinates
        # of a discrepancy in the robot's field of vision
        phy_points = set()
        for angle, distance in enumerate(phy_valid_ranges):
            phy_points.add(angle_to_point(distance, phy_scan, angle))

        for angle, distance in enumerate(vir_valid_ranges):
            point = angle_to_point(distance, vir_scan, angle)

            if not any(
                norm(np.array(point) - np.array(phy_point)) < threshold for phy_point in phy_points
            ):
                pose = PoseStamped()
                pose.header.stamp = phy_scan.header.stamp
                pose.header.frame_id = phy_scan.header.frame_id
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0

                discrepancies.appened(pose)

        return discrepancies

    def pub_callback(self):
        # We need to wait for data from both scanners to be available
        if self.virtual_scan is None or self.physical_scan is None:
            return

        discrepancies = self.find_discrepancies()

        if len(discrepancies) >= 1:
            self.discrepancy_pub.publish(discrepancies[0])

        self.physical_scan = None
        self.virtual_scan = None


def main(args=None):
    rclpy.init(args=args)

    phyvir = PhyVir()
    rclpy.spin(phyvir)

    phyvir.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
