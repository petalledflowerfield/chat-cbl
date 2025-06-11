import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import random
import math


class fake_node(Node):
    def __init__(self):
        super().__init__("fake_node")
        print("working")
        self.publisher = self.create_publisher(PoseStamped, "/phyvir", 10)
        self.timer = self.create_timer(45, self.timer_callback)

    def timer_callback(self):
        object_detected = PoseStamped()
        object_detected.header.frame_id = "map"
        object_detected.header.stamp = self.get_clock().now().to_msg()
        object_detected.pose.position.x = random.uniform(-4, 4)
        object_detected.pose.position.y = random.uniform(-4, 4)
        object_detected.pose.orientation.w = 1.0
        self.publisher.publish(object_detected)
        self.get_logger().info(
            f"Published pose at x={object_detected.pose.position.x}, y={object_detected.pose.position.y}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = fake_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
