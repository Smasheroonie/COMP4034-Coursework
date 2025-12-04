import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose

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
        
        self.get_logger().info("nav_node started and waiting for Nav2 action server...")
        
    def nav_callback(self):
        x, y = self.arena_waypoints[self.current_wp]
        self.get_logger().info(f"Going to waypoint {self.current_wp}: ({x}, {y})")
        self.send_goal(x, y)
        
        self.current_wp = (self.current_wp + 1) % len(self.arena_waypoints)
        
    def send_goal(self, x, y):
        
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
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        self.get_logger().info("Navigation finished.")
        
def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()