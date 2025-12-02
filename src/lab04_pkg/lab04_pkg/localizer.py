
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion
import numpy as np
import math

def wrap_angle(angle):
    """Wrap angle to [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

class LocalizerNode(Node):
    def __init__(self):
        super().__init__('ekf_localizer')
        
        # State [x, y, theta] and Covariance
        self.mu = np.zeros(3) # x, y, theta
        self.Sigma = np.eye(3) * 0.1
        self.Sigma[2,2] = 0 # We are certain about the initial orientation

        # Noise parameters
        # Motion noise (from velocity commands)
        self.alphas = np.array([0.1, 0.01, 0.1, 0.01, 0.0, 0.0]) # a1, a2, a3, a4, ...
        # Measurement noise
        self.Q = np.diag([0.1**2, (5*np.pi/180)**2]) # range_std^2, bearing_std^2

        # Landmark ground truth positions (from Table 1 in the PDF)
        self.landmarks = {
            11: np.array([-1.1, -1.1]), 12: np.array([-1.1, 0.0]), 13: np.array([-1.1, 1.1]),
            21: np.array([0.0, -1.1]),  22: np.array([0.0, 0.0]),  23: np.array([0.0, 1.1]),
            31: np.array([1.1, -1.1]),  32: np.array([1.1, 0.0]),  33: np.array([1.1, 1.1]),
        }

        self.last_odom_vel = np.zeros(2) # [v, w]
        self.dt = 1.0 / 20.0 # 20 Hz

        # ROS Publishers and Subscribers
        self.ekf_pub = self.create_publisher(Odometry, '/ekf', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.landmark_sub = self.create_subscription(LandmarkArray, '/landmarks', self.landmark_callback, 10)
        
        # EKF Prediction Timer
        self.timer = self.create_timer(self.dt, self.prediction_step)

        self.get_logger().info('EKF Localizer Node started.')

    def odom_callback(self, msg):
        # Store the latest velocity command
        self.last_odom_vel[0] = msg.twist.twist.linear.x
        self.last_odom_vel[1] = msg.twist.twist.angular.z

    def prediction_step(self):
        v = self.last_odom_vel[0]
        w = self.last_odom_vel[1]
        theta = self.mu[2]

        # Handle case w is close to zero
        if abs(w) < 1e-6:
            # Straight line motion
            G = np.array([
                [1, 0, -v * self.dt * np.sin(theta)],
                [0, 1,  v * self.dt * np.cos(theta)],
                [0, 0, 1]
            ])
            # Motion update
            self.mu[0] += v * self.dt * np.cos(theta)
            self.mu[1] += v * self.dt * np.sin(theta)
            
            V = np.array([
                [self.dt * np.cos(theta), -v * self.dt**2 / 2 * np.sin(theta)],
                [self.dt * np.sin(theta),  v * self.dt**2 / 2 * np.cos(theta)],
                [0, self.dt]
            ])

        else:
            # Curvilinear motion
            v_w = v / w
            G = np.array([
                [1, 0, -v_w * np.cos(theta) + v_w * np.cos(wrap_angle(theta + w * self.dt))],
                [0, 1, -v_w * np.sin(theta) + v_w * np.sin(wrap_angle(theta + w * self.dt))],
                [0, 0, 1]
            ])
            # Motion update
            self.mu[0] += -v_w * np.sin(theta) + v_w * np.sin(wrap_angle(theta + w * self.dt))
            self.mu[1] +=  v_w * np.cos(theta) - v_w * np.cos(wrap_angle(theta + w * self.dt))
            self.mu[2] = wrap_angle(theta + w * self.dt)

            # Jacobian of motion with respect to control
            V = np.array([
                [(-np.sin(theta) + np.sin(wrap_angle(theta + w * self.dt))) / w, v * (np.sin(theta) - np.sin(wrap_angle(theta + w * self.dt)))/w**2 + v * np.cos(wrap_angle(theta + w * self.dt)) * self.dt / w],
                [(np.cos(theta) - np.cos(wrap_angle(theta + w * self.dt))) / w, -v * (np.cos(theta) - np.cos(wrap_angle(theta + w * self.dt)))/w**2 + v * np.sin(wrap_angle(theta + w * self.dt)) * self.dt / w],
                [0, self.dt]
            ])
        
        # Control noise covariance
        M = np.diag([self.alphas[0]*v**2 + self.alphas[1]*w**2, self.alphas[2]*v**2 + self.alphas[3]*w**2])

        # Predict covariance
        self.Sigma = G @ self.Sigma @ G.T + V @ M @ V.T

        # Publish the predicted state
        self.publish_ekf_state()

    def landmark_callback(self, msg):
        for landmark_obs in msg.landmarks:
            landmark_id = landmark_obs.id
            if landmark_id in self.landmarks:
                # Perform EKF update for this landmark
                self.update_step(landmark_obs)

    def update_step(self, landmark_obs):
        lx = self.landmarks[landmark_obs.id][0]
        ly = self.landmarks[landmark_obs.id][1]
        
        # Observed measurement
        z = np.array([landmark_obs.range, landmark_obs.bearing])

        # Predicted measurement
        dx = lx - self.mu[0]
        dy = ly - self.mu[1]
        q = dx**2 + dy**2
        z_hat = np.array([np.sqrt(q), wrap_angle(np.arctan2(dy, dx) - self.mu[2])])

        # Jacobian of measurement model H
        H = np.array([
            [-dx/np.sqrt(q), -dy/np.sqrt(q), 0],
            [ dy/q,         -dx/q,        -1]
        ])

        # Kalman Gain
        S = H @ self.Sigma @ H.T + self.Q
        K = self.Sigma @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        y = z - z_hat
        y[1] = wrap_angle(y[1]) # Wrap bearing innovation
        self.mu = self.mu + K @ y
        self.mu[2] = wrap_angle(self.mu[2])
        self.Sigma = (np.eye(3) - K @ H) @ self.Sigma

    def publish_ekf_state(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Pose
        msg.pose.pose.position.x = self.mu[0]
        msg.pose.pose.position.y = self.mu[1]
        q = self.quaternion_from_euler(0, 0, self.mu[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        # Covariance
        cov = np.zeros((6, 6))
        cov[0:2, 0:2] = self.Sigma[0:2, 0:2] # x, y
        cov[5, 5] = self.Sigma[2, 2]         # theta
        cov[0, 5] = self.Sigma[0, 2]
        cov[1, 5] = self.Sigma[1, 2]
        cov[5, 0] = self.Sigma[2, 0]
        cov[5, 1] = self.Sigma[2, 1]
        msg.pose.covariance = cov.flatten().tolist()

        # Twist (velocity) - we are not estimating it in this version
        # So we leave it as zero
        
        self.ekf_pub.publish(msg)

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q


def main(args=None):
    rclpy.init(args=args)
    node = LocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
