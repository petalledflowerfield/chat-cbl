from rclpy.node import Node  # Base class for writing in ros2 node
from std_msgs.msg import Bool


class Signaling(Node):
    def __init__(self):
        super().__init__("Signaling")
        self.trash_num_subscription = self.create_subscription(
            int,
            "/trash_num",
            self.signal_callback,
            10,  # queue size (can change based on how dense the trashes are)
        )
        self.publisher = self.create_publisher(Bool, "Signal", 10)
        self.signal = False

    def signal_callback(self, msg):
        if msg.object_count >= 5:
            self.get_logger().info("A lot of trash detected, extra robots called!")
            self.signal = True
            self.publisher.publish(self.signal)
