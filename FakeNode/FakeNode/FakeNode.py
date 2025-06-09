import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import random
import math

class FakeNode(Node):
    def __init__(self):
        super().__init__("smart_explorer")
        self.publisher = self.create_publisher(PoseStamped, "/phyvir", 10)
        self.timer = self.create_timer(15.0, self.timer_callback)

    def timer_callback(self):
        object_detected = PoseStamped()
        object_detected.header.frame_id = "map"
        object_detected.header.stamp = self.get_clock().now().to_msg()
        object_detected.pose.position.x = random.uniform(-2, 2)
        object_detected.pose.position.y = random.uniform(-2, 2)
        object_detected.pose.orientation.w = 1.0
        self.publisher.publish(object_detected)
        self.get_logger().info(
            f"Published pose at x={object_detected.pose.position.x}, y={object_detected.pose.position.y}")














def main():
    rclpy.init()
    node = FakeNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()
