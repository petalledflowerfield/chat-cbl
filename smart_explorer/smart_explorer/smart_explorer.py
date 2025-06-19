import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import random
import math



YAW_DEG      = 0.0


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class SmartExplorer(Node):
    def __init__(self):
        super().__init__("smart_explorer")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")


        self.active_goal = None
        self.has_obj_goal = False
        self.has_wander_goal = False

        self.disc_angle = 0
        self.disc_distance = 0
        self.object_detected = False
        self.wander_radius = 0.6
        self.current_loc = (0, 0)
        self.arrived_at_Object = False
        self.subscription = self.create_subscription(
            PoseStamped, "discrepancies", self.object_callback, 10
        )
        self.publisher = self.create_publisher(Bool, "/arrived_at_object", 10)
        self.control_loop()
        self.timer = self.create_timer(30, self.control_loop)



    def object_callback(self, msg):


        if self.active_goal and self.has_wander_goal:
            self.get_logger().info("Cancelling current wander goal...")
            cancel_future = self.active_goal.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info("Wander goal cancelled."))
            self.has_wander_goal = False




        if not self.has_obj_goal:

            msg.pose.position.x = msg.pose.position.x + self.current_loc[0]
            msg.pose.position.y = msg.pose.position.y + self.current_loc[1]
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = msg
            send_goal_future= self.nav_client.send_goal_async(goal_msg)
            self.get_logger().info(
                f"Sending to object goal"
            )
            msgB = Bool()
            msgB.data = self.arrived_at_Object
            self.publisher.publish(msgB)
            self.get_logger().info("Approaching detected object...")
            self.has_obj_goal = True
            send_goal_future.add_done_callback(self.object_goal_response_callback)
            self.current_loc = msg.pose.position.x, msg.pose.position.y






    def control_loop(self):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 server not available.")
            return

        if not self.has_wander_goal and not self.has_obj_goal:
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
        goal_pose.pose.orientation = yaw_to_quaternion(math.radians(YAW_DEG))
        self.current_loc = goal_pose.pose.position.x, goal_pose.pose.position.y

        self.get_logger().info(
            f"Sending wander goal to x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}"
        )


        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.has_wander_goal = True
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.wander_goal_response_callback)


    def wander_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Wander goal rejected.")
            return
        self.get_logger().info("Wander goal accepted.")

        goal_handle.get_result_async().add_done_callback(self._result_cb)
        self.active_goal = goal_handle

    def object_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("object goal rejected.")
            return
        self.get_logger().info("object goal accepted.")

        goal_handle.get_result_async().add_done_callback(self._result_cb)

        self.active_goal = goal_handle

    def _result_cb(self, fut):
        status = fut.result().status
        txt = {4:'SUCCEEDED',6:'ABORTED',5:'CANCELED'}.get(status,'DONE')
        self.get_logger().info(f'Goal result: {txt}')
        if self.has_obj_goal:
            self.arrived_at_Object = True
            self.has_obj_goal = False
        if self.has_wander_goal:
            self.has_wander_goal = False


def main():
    rclpy.init()
    node = SmartExplorer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
