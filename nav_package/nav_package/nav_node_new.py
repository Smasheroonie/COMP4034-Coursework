import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav2_msgs.action import NavigateToPose
import math

class NavNode(Node):
    
    def __init__(self):
        super().__init__('nav_node')
        
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
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
        self.is_navigating = False
        self.current_goal_x = None
        self.current_goal_y = None
        self.current_x = None
        self.current_y = None
        self.pause_timer = None
        
        self.goal_tolerance = 0.05  # cm
        self.check_interval = 0.2   # milliseconds
        self.pause_duration = 5.0   # seconds
        
        self.pos_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pos_callback,
            10
        )
        
        self.distance_timer = self.create_timer(self.check_interval, self.check_distance)
        
        self.get_logger().info("nav_node started and waiting for Nav2 action server...")
        
        self.navigate()
        
    def pos_callback(self, msg):
                
        # self.get_logger().info(msg)
        
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        self.get_logger().info(f'X: {self.current_x}, Y: {self.current_y}')
        
    def check_distance(self):
        
        if not self.is_navigating or self.current_goal_x is None or self.current_goal_y is None or self.current_x is None or self.current_y is None:
            return
        
        distance = math.sqrt((self.current_x - self.current_goal_x) ** 2 + (self.current_y - self.current_goal_y) ** 2)
        
        self.get_logger().info(f'Distance to goal: {distance}')
        
        if distance < self.goal_tolerance:
            self.get_logger().info('Point reached.')
            self.pause_navigation()
            
    def pause_navigation(self):
        
        self.is_navigating = False
        
        if self.pause_timer:
            self.pause_timer.cancel()

        self.pause_timer = self.create_timer(self.pause_duration, self.resume_navigation)
        
    def resume_navigation(self):
        
        if self.pause_timer:
            self.pause_timer.cancel()
            self.pause_timer = None
        
        self.current_wp = (self.current_wp + 1) % len(self.arena_waypoints)
        
        self.get_logger().info('Resuming navigation.')
        
        self.navigate()
        
    def navigate(self):
        
        if self.is_navigating:
            return
        
        x, y = self.arena_waypoints[self.current_wp]
        self.get_logger().info(f"Going to waypoint {self.current_wp}: ({x}, {y})")
        self.send_goal(x, y)
                
    def send_goal(self, x, y):

        self.is_navigating = True

        self.current_goal_x = x
        self.current_goal_y = y

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.w = 1.0
        
        
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        self.get_logger().info("Point reached but navigation stopped.")
        
def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()