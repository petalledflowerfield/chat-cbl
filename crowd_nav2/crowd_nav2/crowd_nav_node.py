import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import numpy as np
import random


class CrowdNavNode(Node):
    """
    A class used to initialize ROS2 node, simulating a crowd density api, which gives the navigation node a destination.



    Attributes
    ----------
    navigator:
        An instance of the Navigator class.
    cell_height:
        Real world height in meters.
    cell_width:
        Real world width in meters.
    grid_size_a:
        The height of the grid
    grid_size_b:
        The width of the grid
    start_loc:
        Start location of the robot.
    hour:
        Hour of the day simulated by the API.
    weight_D
        Weight of how much the distance of the target location affects its priority.
    weight_H:
        Weight of how much the High crowd density of the target location affects its priority.
    weight_L:
        Weight of how much the Low crowd density of the target location affects its priority.
    high_density_zone:
        Predefined list of coordinates which indicates the high density zone.
    obstacles:



    Methods
    -------
    gen_grid(self, hour)
        Generates the grid for the specific hour of the day.

    apply_obstacles(self,grid)
        Applies the pre-given obstacles to the given grid.

    find_target_location(self, prev_grid, current_grid)
        Based of the previous grid and the current grid, this method finds a list of target locations.

    create_priority(self, targets, prev_grid, current_grid)
        Gives a priority value to each of the target locations.

    grid_to_world(self, grid_x, grid_y, cell_width, cell_height, origin)
        Scales the simulated API grid coordinates to real world coordinates.

    send_goal(self, x, y)
        Send the highest priority targets locations coordinates to the navigator node.

    run_cycle(self)
        Runs the cycle of the API for each hour, with a set timer between cycles.
    """

    def __init__(self):
        super().__init__("crowd_nav_node")
        print("working")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.cell_height = 1.5 / 8
        self.cell_width = 2 / 12
        self.grid_size_a = 8
        self.grid_size_b = 12
        self.start_loc = (0, 0)
        self.hour = 0
        self.weight_D = 0.2
        self.weight_H = 0.6
        self.weight_L = 0.2
        self.high_density_zone = [
            (1, 4),
            (1, 5),
            (1, 6),
            (2, 4),
            (2, 5),
            (2, 6),
            (3, 4),
            (3, 5),
            (3, 6),
        ]
        self.obstacles = [
            (1, 0),
            (1, 1),
            (1, 2),
            (3, 0),
            (3, 1),
            (3, 2),
            (1, 9),
            (1, 10),
            (2, 9),
            (2, 10),
            (3, 9),
            (3, 10),
            (5, 3),
            (5, 4),
            (5, 5),
            (5, 6),
            (6, 3),
            (6, 4),
            (6, 5),
            (6, 6),
            (7, 3),
            (7, 4),
            (7, 5),
            (7, 6),
            (5, 10),
            (6, 10),
            (7, 10),
            (5, 11),
            (6, 11),
            (7, 11),
        ]
        self.prev_grid = self.gen_grid(self.hour)
        self.prev_grid = self.apply_obstacles(self.prev_grid)

        self.timer = self.create_timer(30.0, self.run_cycle)

    # Generates the grid
    # hour = integer determining the hour of the day for the grid to display
    # returns a size_a * size_b grid containing 0-1 crowd density
    def gen_grid(self, hour):
        grid = np.zeros((self.grid_size_a, self.grid_size_b))
        for x in range(self.grid_size_a):
            for y in range(self.grid_size_b):
                loc_id = f"loc_{x}_{y}"
                random.seed(hash(loc_id) % 10000 + hour)
                base_density = random.uniform(0.0, 1.0)
                if (x, y) in self.high_density_zone:
                    base_density = min(1.0, base_density + 0.4)
                grid[x, y] = base_density
        return grid

    # Puts the obstacles into the grid
    def apply_obstacles(self, grid):
        for x, y in self.obstacles:
            grid[x][y] = -1
        return grid

    # Puts the locations fitting the criteria into a list
    # A location that was high density in prev_grid and low density in current_grid
    def find_target_location(self, prev_grid, current_grid):
        targets = []
        for i in range(prev_grid.shape[0]):
            for j in range(prev_grid.shape[1]):
                if prev_grid[i][j] > 0.7 and current_grid[i][j] < 0.3:
                    targets.append((i, j))
        return targets

    # Creates a priority list of the target locations
    # Uses a weighted equation of priority = distance_from_point + low_density_value + high_density_value
    # The equation prioritizes targets that are close, had higher density in prev_grid, and now has lower density in current_grid
    def create_priority(self, targets, prev_grid, current_grid):
        priorities = []
        max_dist = np.sqrt(self.grid_size_a**2 + self.grid_size_b**2)
        for tx, ty in targets:
            dist = np.sqrt(
                (tx - self.start_loc[0]) ** 2 + (ty - self.start_loc[1]) ** 2
            )
            norm_dist = dist / max_dist
            high_density = prev_grid[tx][ty]
            low_density = current_grid[tx][ty]
            score = (
                self.weight_H * high_density
                + self.weight_L * (1 - low_density)
                + self.weight_D * (1 / (norm_dist + 1e-6))
            )
            priorities.append(score)
        return priorities

    # Scales to real world
    def grid_to_world(self, grid_x, grid_y, cell_width, cell_height, origin):
        world_x = origin[0] + (grid_y - 4) * cell_width

        world_y = origin[1] + (grid_x - 6) * cell_height
        return world_x, world_y

    # Send location to the nav node
    def send_goal(self, x, y):
        world_x, world_y = self.grid_to_world(
            x, y, self.cell_width, self.cell_height, self.start_loc
        )
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(world_x)
        goal.pose.position.y = float(world_y)
        goal.pose.orientation.w = 1.0
        self.nav_client.send_goal_async(NavigateToPose.Goal(pose=goal))

    # Runs the cycle of the program
    def run_cycle(self):
        self.hour += 1
        current_grid = self.gen_grid(self.hour)
        current_grid = self.apply_obstacles(current_grid)
        targets = self.find_target_location(self.prev_grid, current_grid)

        if not targets:
            self.get_logger().info("No valid targets found.")
            return

        priorities = self.create_priority(targets, self.prev_grid, current_grid)
        best_index = np.argmax(priorities)
        best_target = targets[best_index]
        self.get_logger().info(f"Navigating to target: {best_target}")
        self.get_logger().info(
            f"Navigating to target: {self.grid_to_world(best_target[1], best_target[0], self.cell_width, self.cell_height, self.start_loc)}"
        )
        self.send_goal(best_target[1], best_target[0])  # x = col, y = row
        self.start_loc = best_target
        self.prev_grid = current_grid


# Running the ROS2 stuff
def main(args=None):
    rclpy.init(args=args)
    node = CrowdNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
