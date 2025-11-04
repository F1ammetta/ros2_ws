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
FRONT_SAMPLE_COUNT = 30
MAX_WEIGHT = FRONT_SAMPLE_COUNT

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

        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0



    def get_max_dist_windowed(self, d):
        n = len(d)
        if n == 0:
            return 0

        half_window = WINDOW_SIZE // 2
        
        max_window_sum_absolute = -math.inf
        max_window_sum_preferred = -math.inf
        
        array_center = n / 2 
        
        center_range_size = n // 4 
        center_low = (n - center_range_size) // 2
        center_high = center_low + center_range_size
        
        potential_windows = []

        for i in range(n):
            current_sum = 0
            
            for k in range(-half_window, half_window + 1):
                index = (i + k) % n
                current_sum += d[index]
            
            potential_windows.append((current_sum, i))

            if current_sum > max_window_sum_absolute:
                max_window_sum_absolute = current_sum
                
        best_center_index = -1 
        
        max_sum_windows = [
            (window_sum, center_index) for window_sum, center_index in potential_windows 
            if window_sum >= max_window_sum_absolute - 2 
        ]
        
        if not max_sum_windows:
            return 0

        def get_preference_metric(center_index, n, center_low, center_high):
            
            
            dist_from_center = abs(center_index - array_center)
            ring_dist = min(dist_from_center, n - dist_from_center)
            
            return ring_dist

        def custom_sort_key(x):
            window_sum, center_index = x
            
            dist_from_center = abs(center_index - array_center)
            ring_dist = min(dist_from_center, n - dist_from_center)
            
            return (-window_sum, get_preference_metric(center_index, n, center_low, center_high))


        candidate_windows = [
            (window_sum, center_index) for window_sum, center_index in potential_windows 
            if window_sum >= max_window_sum_absolute - 2
        ]
        
        def get_ring_distance_from_front(index, n):
            return min(index, n - index)
        
        candidate_windows.sort(
            key=lambda x: (
                -x[0], 
                get_ring_distance_from_front(x[1], n) 
            )
        )
        
        best_center_index = candidate_windows[0][1]

        # self.get_logger().info(f'Max Window Sum (Absolute): {max_window_sum_absolute}, Selected Index: {best_center_index}, Selected Sum: {candidate_windows[0][0]}')
        return best_center_index


    def odom_cb(self, msg: Odometry):
        pose: PoseWithCovariance = msg.pose
        twist: Twist = msg.twist.twist
        qu: Quaternion = pose.pose.orientation
        quat = [qu.x, qu.y, qu.z, qu.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)

        self.yaw = yaw *  180.0 / math.pi
        self.w = twist.angular.z



    def lidar_cb(self, msg: LaserScan):
        vel = Twist()
        vel.linear = Vector3()
        vel.angular = Vector3()

        minang = msg.angle_min * 180 / math.pi
        dang = msg.angle_increment * 180 / math.pi

        self.get_logger().info(f' min {msg.range_min}, max {msg.range_max}')
        self.get_logger().info(f'range  inc {msg.angle_increment}, min {msg.angle_min}, max {msg.angle_max}')
        self.get_logger().info(f'range  inc {msg.angle_increment}, min {msg.angle_min}, max {msg.angle_max}')

        weighted_sum = 0.0
        total_weight = 0.0

        for i in range(FRONT_SAMPLE_COUNT):
            weight = MAX_WEIGHT - i
            distance = msg.ranges[i]
            
            if math.isfinite(distance):
                weighted_sum += distance * weight
                total_weight += weight

        for i in range(FRONT_SAMPLE_COUNT):
            weight = MAX_WEIGHT - i
            distance = msg.ranges[-(i + 1)]
            
            if math.isfinite(distance):
                weighted_sum += distance * weight
                total_weight += weight

        if total_weight > 0:
            dist = weighted_sum / total_weight
        else:
            dist = float('nan')


        # self.get_logger().info(f'dist front {dist}')

        if dist < 0.6 and self.state == 'moving':
            angle = self.get_max_dist_windowed(msg.ranges) * dang + minang
            self.theta = angle if angle < 180.0 else angle - 360.0
            vel.linear.x = 0.0
            self.state = 'rotating'
            self.yaw0 = self.yaw
            # vel.angular.z = 1.5 if self.theta > self.yaw0 else -1.5

        elif self.state == 'moving': 
            # vel.linear.x = 0.15
            vel.angular.z = 0.0
        elif self.state == 'rotating':
            target = (self.yaw0 + self.theta)
            if target > 180.0:
                target -= 360.0

            if target < -180.0:
                target += 360.0


            if self.w > 0.0:
                if self.yaw > target: 
                    # self.get_logger().info(f'bingo')
                    # vel.linear.x = 0.15
                    vel.angular.z = 0.0
                    self.state = 'moving'
                else:
                    # vel.angular.z = 1.5

            elif self.w < 0.0:
                if self.yaw < target: 
                    # self.get_logger().info(f'bingo')
                    vel.angular.z = 0.0
                    # vel.linear.x = 0.15
                    self.state = 'moving'
                else:
                    # vel.angular.z = -1.5



        self.publisher_.publish(vel)
        self.i += 1


        

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
