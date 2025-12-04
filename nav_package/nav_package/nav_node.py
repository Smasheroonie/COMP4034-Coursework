import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from random import randrange
import math

class NavNode(Node):

    def __init__(self):
        super().__init__('nav_node')
        
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.waypoints = [
            (-1.5, 4.0),
            (-1.5, -3.75),
            (-0.207, 4.25),
            (1.45, 2.3),
            (3.50, 2.44),
            (3.50, 4.0),
            (-0.207, 4.25),
            (0.229, -3.6),
            (2.6, -3.6),
            (-1.41, -2.54),
            (2.51, -1.77),
            (1.65, -0.275),
            (2.5, 0.751),
            (5.57, 3.8),
            (-5.57, -3.8),
            (4.35, -2.25)
        ]

        self.arena_waypoints = [
            (-0.8061, 0.53), # top left corner
            (-2.19, -0.844), # next to U-shaped walls
            (-3.97, -2.57), # bottom left corner
            (-3.32, -3.49), # a bit along to the right
            (-2.14, -2.34), # facing the bottom of the U-shaped walls
            (-1.36, -2.84),# turn right
            (-0.274, -1.59), # turn left, facing bottom of top L-shaped wall
            (1.12, -1.37), # top right corner
            (-1.91, -4.43) # bottom right corner
        ]

        self.current_wp = 0

        # self.pos_sub = self.create_subscription()

        self.green_sub = self.create_subscription(
            Point,
            'green_object',
            self.green_callback,
            10
        )
        self.latest_green = None

        self.red_sub = self.create_subscription(
            Point,
            'red_object',
            self.red_callback,
            10
        )
        self.latest_red = None

        self.timer = self.create_timer(30.0, self.timer_callback)

        self.get_logger().info("nav_node started and waiting for Nav2 action server...")

    def timer_callback(self):

        # PRIORITY: go to green object if detected
        if self.latest_green is not None:
            gx, gy = self.latest_green.x, self.latest_green.y
            self.get_logger().info(f"Going to GREEN OBJECT at ({gx:.2f}, {gy:.2f})")
            self.send_goal(gx, gy)
            self.latest_green = None
            return
        
        elif self.latest_red is not None:
            rx,ry = self.latest_red.x, self.latest_red.y
            self.get_logger().info(f"Going to RED OBJECT at ({rx:.2f}, {ry:.2f})")
            self.send_goal(rx, ry)
            self.latest_red = None
            return

        # Otherwise: follow waypoint patrol
        x, y = self.arena_waypoints[self.current_wp]
        self.get_logger().info(f"Going to waypoint {self.current_wp}: ({x}, {y})")
        self.send_goal(x, y)

        # Move to next waypoint
        self.current_wp = (self.current_wp + 1) % len(self.arena_waypoints)

    def send_goal(self, x, y):

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # simple orientation (facing forward)
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted!")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("Navigation finished.")

    def green_callback(self, msg: Point):
        self.get_logger().info(
            f"Green object detected at ({msg.x:.2f}, {msg.y:.2f})"
        )
        self.latest_green = msg

    def red_callback(self, msg: Point):
        self.get_logger().info(
            f"Red object detected at ({msg.x:.2f}, {msg.y:.2f})"
        )
        self.latest_red = msg

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()