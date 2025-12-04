import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, PointStamped, Point
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
import numpy as np
import cv2
import pyrealsense2 as rs

def quaternion_to_rpy(q: Quaternion):
    """
    Convert a quaternion into roll, pitch, and yaw (in radians).
    """
    x, y, z, w = q.x, q.y, q.z, q.w

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class ImageSubscriber(Node):
    
    def __init__(self):
        super().__init__('image_subscriber')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.subscription_image = self.create_subscription(
            Image,
            '/camera_depth/image_raw',
            self.image_callback,
            10)
        
        self.subscription_dimage = self.create_subscription(
            Image,
            '/camera_depth/depth/image_raw',
            self.dimage_callback,
            10)
        
        self.subscription_int = self.create_subscription(
            CameraInfo,
            '/camera_depth/camera_info',
            self.ins_callback,
            10)

        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.green_pub = self.create_publisher(Point, 'green_object', 10)
        
        self.ins = None
        self.image = []
        self.dimage = []
        
        self.br = CvBridge()
        
        self.timer = self.create_timer(0.2, self.timer_callback)
    
    def odom_callback(self, msg):
        self.location = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        _, _, self.orientation = quaternion_to_rpy(msg.pose.pose.orientation)
    
    def ins_callback(self, data):
        self.ins = data
    
    def tf_from_cam_to_map(self):
        from_frame = 'camera_rgb_optical_frame'
        to_frame = 'map'
        
        now = rclpy.time.Time()
        
        try:
            tf = self.tf_buffer.lookup_transform(to_frame, from_frame, now, timeout=rclpy.duration.Duration(seconds=1.0))
            return tf
        except:
            return None
        
    def image_callback(self, data):
        self.get_logger().info('Receiving video frame')
    
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        self.image = current_frame
        
    def dimage_callback(self, data):
        self.get_logger().info('Receiving dvideo frame')
    
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        self.dimage = current_frame
        
    def timer_callback(self):
        current_frame = self.image
        if current_frame == []:
            return
        
        centroids = [] #from the rgb image
        depths = [] #from the depth image (pixel of centroid)
        
        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Clean up mask (open/close to remove noise)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)
            if area < 100:
                continue    # ignore tiny specks (artifacts)

            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            centroids.append((cx, cy))
            depths.append(self.dimage[cy, cx])
                
        for (cx, cy) in centroids:
            cv2.circle(current_frame, (cx, cy), 5, (0, 0, 255), -1)
        
        # Do some vision processing here to get the centroids of any green objects
        # `centroids` should contain a list pixel points (x,y) for each green centroid
        # `depths' should contain a list of the corresponding depth at those pixel points
        
        
        if self.ins is None:
            return

        cameraInfo = self.ins

        _intrinsics = rs.intrinsics()
        _intrinsics.width = cameraInfo.width
        _intrinsics.height = cameraInfo.height
        _intrinsics.ppx = cameraInfo.k[2]
        _intrinsics.ppy = cameraInfo.k[5]
        _intrinsics.fx = cameraInfo.k[0]
        _intrinsics.fy = cameraInfo.k[4]
        _intrinsics.model = rs.distortion.none
        _intrinsics.coeffs = [i for i in cameraInfo.d]

        points_3d = [rs.rs2_deproject_pixel_to_point(_intrinsics, centroids[x], depths[x]) for x in range(len(centroids))]
        
        #todo error handling - points_3d will be empty if there are no green object centroids in the image
        if len(points_3d) == 0:
            print(points_3d)
            return
        else:            
            print(f"points_3d (camera to object): {points_3d} with len {len(points_3d)}")
            #print("poins_3d[0]: ", points_3d[0])
            
            point = PointStamped()
            point.header.frame_id = 'map'
            point.point.x = points_3d[0][0]
            point.point.y = points_3d[0][1]
            point.point.z = points_3d[0][2]

            tf = self.tf_from_cam_to_map()
            print("tf (camera to map frame transformation): ", tf)
            
            if tf == None: return #Exit function if tf failed.
            point_world = do_transform_point(point, tf)
            print("point_world (map to object?): ", point_world)
            
            point_msg = Point()
            point_msg.x = point_world.point.x
            point_msg.y = point_world.point.y
            point_msg.z = point_world.point.z

            self.green_pub.publish(point_msg)
            
            print(point_world.point.x, point_world.point.y)
            
            cv2.imshow("camera", current_frame)
            cv2.imshow("camera2", mask)
            
            cv2.waitKey(5)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()