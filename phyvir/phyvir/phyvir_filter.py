import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class PhyVir(Node):
    subscriber = None
    publisher = None

    logged_scan = None

    def __init__(self):
        super().__init__("phyvir_filter")

        self.subscriber = self.create_subscription(
            LaserScan, "scan", self.sub_callback, qos_profile=qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(
            LaserScan, "filtered_scan", qos_profile=qos_profile_sensor_data
        )

        pub_period = 0.1
        self.timer = self.create_timer(pub_period, self.pub_callback)

    def sub_callback(self, msg):
        self.logged_scan = msg
        self.get_logger().info("Got message")

    def pub_callback(self):
        if self.logged_scan is None:
            return

        self.publisher.publish(self.logged_scan)
        self.logged_scan = None


def main(args=None):
    rclpy.init(args=args)

    phyvir = PhyVir()
    rclpy.spin(phyvir)

    phyvir.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
