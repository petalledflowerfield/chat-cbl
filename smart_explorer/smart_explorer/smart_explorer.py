import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool

import random
import math


class SmartExplorer(Node):
    def __init__(self):
        super().__init__("smart_explorer")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.disc_angle = 0
        self.disc_distance = 0
        self.object_detected = False
        self.wander_radius = 2.5
        self.current_loc = (0, 0)
        self.arrived_at_Object = False
        self.subscription = self.create_subscription(
            PoseStamped, "/phyvir", self.object_callback, 10
        )
        self.publisher = self.create_publisher(Bool, "/arrived_at_Object", 10)
        self.timer = self.create_timer(15.0, self.control_loop)

        self.resume_timer = None
        self.state = 'wander'
        self.active_goal = None


    def object_callback(self, msg):


        obj_pose = PoseStamped()
        obj_pose.header.frame_id = "map"
        obj_pose.header.stamp = self.get_clock().now().to_msg()
        obj_pose.pose.position.x = msg.pose.position.x
        obj_pose.pose.position.y = msg.pose.position.y
        obj_pose.pose.orientation.w = 1.0
        self.arrived_at_Object = True

        msg = Bool()
        msg.data = self.arrived_at_Object
        self.publisher.publish(msg)
        self.get_logger().info("Approaching detected object...")
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=obj_pose))
        self.state = 'object'
        self.resume_timer = self.create_timer(15.0, self.resume_wandering)

    def resume_wandering(self):
        self.get_logger().info("Done waiting. Switching back to wander mode.")
        self.state = "wander"


        if self.resume_timer is not None:
            self.resume_timer.cancel()
            self.resume_timer = None

    def control_loop(self):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 server not available.")
            return

        if self.state == 'wander':
            self.send_random_goal()


    def send_random_goal(self):
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(1.0, self.wander_radius)

        dx = distance * math.cos(angle)
        dy = distance * math.sin(angle)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = dx
        goal_pose.pose.position.y = dy
        goal_pose.pose.orientation.w = 1.0
        self.current_loc = goal_pose.pose.position.x, goal_pose.pose.position.y

        self.get_logger().info(
            f"Sending wander goal to x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}"
        )
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=goal_pose))




def main():
    rclpy.init()
    node = SmartExplorer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
