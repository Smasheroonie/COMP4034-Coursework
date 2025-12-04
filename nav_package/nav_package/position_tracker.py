import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class PositionTracker(Node):
    def __init__(self):
        super().__init__('position_tracker')

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.current_position = None
        self.total_distance = 0.0
        self.previous_position = None

    def odom_callback(self, msg):
        
        position = msg.pose.pose.position

        self.current_position = {'x': position.x, 'y': position.y}
        
        if self.previous_position:
            
            distance = ((position.x - self.previous_position['x']) ** 2 + (position.y - self.previous_position['y']) ** 2) ** 0.5
            
            self.total_distance += distance

        self.previous_position = {'x': position.x, 'y': position.y}

        print(f"Current position: ({self.current_position['x']:.3f}, {self.current_position['y']:.3f})")
        print(f"Total distance: {self.total_distance:.3f} m")

def main(args=None):
    rclpy.init(args=args)
    tracker = PositionTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()