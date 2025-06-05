import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
from scipy.spatial.transform import Rotation as R
import random
import math

class SmartExplorer(Node):
    def __init__(self):
        super().__init__('smart_explorer')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.object_detected = False
        self.wander_radius = 2.5


        self.create_subscription(Bool, '/object_detected', self.object_callback, 10)
        self.timer = self.create_timer(5.0, self.control_loop)

        self.active_goal = None
        self.state = 'wander'

    def object_callback(self, msg: Bool):
        self.object_detected = msg.data
        if self.object_detected:
            self.get_logger().info("Object detected! Switching to object mode.")
            self.state = 'object'

    def control_loop(self):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 server not available.")
            return

        if self.state == 'wander':
            self.send_random_goal()
        elif self.state == 'object':
            self.approach_object()

    def send_random_goal(self):


        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(1.0, self.radius)

        dx = distance * math.cos(angle)
        dy = distance * math.sin(angle)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x =  dx
        goal_pose.pose.position.y =  dy
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending wander goal to x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}")
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=goal_pose))

    def approach_object(self):



        self.get_logger().info("Approaching detected object...")
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=goal_pose))

        self.state = 'wander'
        self.object_detected = False



def main():
    rclpy.init()
    node = SmartExplorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
