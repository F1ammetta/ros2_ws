import rclpy
import tf_transformations
from rclpy.node import Node
import math

from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_sensor_data

WINDOW_SIZE = 30
FRONT_SAMPLE_COUNT = 9
MAX_WEIGHT = FRONT_SAMPLE_COUNT
MAX_TURN_ANGLE = 100.0  # <<<--- ADDED THIS LIMIT (in degrees)

class Controller(Node):
    theta = 0
    state = 'moving'
    w = 0.0
    yaw0 = 0.0
    yaw = 0.0


    def __init__(self):
        super().__init__('controller')

        self.sub = self.create_subscription(LaserScan, "/scan", self.lidar_cb,
                                            qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.i = 0


    # Updated function to only find windows within the MAX_TURN_ANGLE
    def get_max_dist_windowed(self, d, minang, dang):
        n = len(d)
        if n == 0:
            return 0

        half_window = WINDOW_SIZE // 2
        
        potential_windows = [] # Will store (window_sum, center_index)

        # Helper to get distance from front (index 0)
        def get_ring_distance_from_front(index, n):
            return min(index, n - index)

        # 1. Calculate all window sums, but only for *valid* forward angles
        max_forward_sum = -math.inf
        forward_windows = [] # Will store (window_sum, center_index)

        for i in range(n):
            # Calculate the angle for this index
            current_angle = i * dang + minang
            # Normalize to [-180, 180]
            turn_angle = current_angle if current_angle < 180.0 else current_angle - 360.0
            
            # *** THIS IS THE FIX ***
            # Only consider this index if it's within the allowed turn radius
            if abs(turn_angle) > MAX_TURN_ANGLE:
                continue # Skip this window, it's too far to the side or behind

            # This is a valid forward window, so calculate its sum
            current_sum = 0
            for k in range(-half_window, half_window + 1):
                index = (i + k) % n
                current_sum += d[index]
            
            forward_windows.append((current_sum, i))
            
            if current_sum > max_forward_sum:
                max_forward_sum = current_sum

        # 2. Handle case where no forward windows were found (e.g. trapped)
        if not forward_windows:
            self.get_logger().warn("No valid escape route found within +/- 100 degrees. Stopping.")
            return 0 # Return index 0 (front) as a fallback

        # 3. Filter the forward windows to get "good enough" candidates
        # We want windows that are *close* to the best forward sum
        candidate_windows = [
            (window_sum, center_index) for window_sum, center_index in forward_windows 
            if window_sum >= max_forward_sum - 2
        ]

        # 4. Sort the "good enough" candidates
        # Primary sort: by sum (descending)
        # Secondary sort: by distance from front (ascending)
        candidate_windows.sort(
            key=lambda x: (
                -x[0],  # Sort by sum (highest first)
                get_ring_distance_from_front(x[1], n) # Tie-break by closeness to front
            )
        )
        
        # 5. The best index is the first one in the sorted list
        best_center_index = candidate_windows[0][1]

        return best_center_index


    def odom_cb(self, msg: Odometry):
        pose: PoseWithCovariance = msg.pose
        twist: Twist = msg.twist.twist
        qu: Quaternion = pose.pose.orientation
        quat = [qu.x, qu.y, qu.z, qu.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)

        self.yaw = yaw * 180.0 / math.pi
        self.w = twist.angular.z



    def lidar_cb(self, msg: LaserScan):
        vel = Twist()
        vel.linear = Vector3()
        vel.angular = Vector3()

        minang = msg.angle_min * 180 / math.pi
        dang = msg.angle_increment * 180 / math.pi

        weighted_sum = 0.0
        min_dist = math.inf

        # Check front-right
        for i in range(FRONT_SAMPLE_COUNT):
            distance = msg.ranges[i]
            if distance < min_dist:
                min_dist = distance

        # Check front-left
        for i in range(FRONT_SAMPLE_COUNT):
            distance = msg.ranges[-(i + 1)]
            if distance < min_dist:
                min_dist = distance

        dist = min_dist


        if dist < 0.5 and self.state == 'moving':
            # Pass minang and dang to the updated function
            angle_idx = self.get_max_dist_windowed(msg.ranges, minang, dang)
            
            angle = angle_idx * dang + minang
            self.theta = angle if angle < 180.0 else angle - 360.0
            
            vel.linear.x = 0.0
            self.state = 'rotating'
            self.yaw0 = self.yaw
            
            # *** THIS IS THE BUG FIX ***
            # Rotate left (positive) if theta > 0, right (negative) if theta < 0
            vel.angular.z = 1.5 if self.theta >= 0.0 else -1.5

        elif self.state == 'moving': 
            vel.linear.x = 0.15
            vel.angular.z = 0.0
            
        elif self.state == 'rotating':
            target = (self.yaw0 + self.theta)
            if target > 180.0:
                target -= 360.0
            if target < -180.0:
                target += 360.0


            if self.w > 0.0: # Rotating positive (CCW)
                if self.yaw > target: 
                    # self.get_logger().info(f'bingo')
                    vel.linear.x = 0.15
                    vel.angular.z = 0.0
                    self.state = 'moving'
                else:
                    vel.angular.z = 1.5 # Keep rotating

            elif self.w < 0.0: # Rotating negative (CW)
                if self.yaw < target: 
                    # self.get_logger().info(f'bingo')
                    vel.angular.z = 0.0
                    vel.linear.x = 0.15
                    self.state = 'moving'
                else:
                    vel.angular.z = -1.5 # Keep rotating
            
            # This handles the first frame of rotation where self.w might still be 0
            elif self.w == 0.0:
                 vel.angular.z = 1.5 if self.theta >= 0.0 else -1.5


        self.publisher_.publish(vel)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
