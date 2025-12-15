import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math

def normalize_angle(theta):
    """
    Normalize angles between [-pi, pi)
    """
    theta = theta % (2 * np.pi)  # force in range [0, 2 pi)
    if np.isscalar(theta):
        if theta > np.pi:  # move to [-pi, pi)
            theta -= 2 * np.pi
    else:
        theta_ = theta.copy()
        theta_[theta>np.pi] -= 2 * np.pi
        return theta_
    
    return theta

def normalize(arr: np.ndarray):
    """ normalize array of values """
    if np.isclose(np.max(arr) - np.min(arr), 0.0):
        return np.zeros_like(arr)
    else:
        return (arr - np.min(arr)) / (np.max(arr) - np.min(arr))

class DifferentialDriveRobot:
    def __init__(self, init_pose, max_linear_acc, max_ang_acc, max_lin_vel, min_lin_vel, max_ang_vel, min_ang_vel, radius):
        self.pose = init_pose
        self.vel = np.array([0.0, 0.0])
        self.max_linear_acc = max_linear_acc
        self.max_ang_acc = max_ang_acc
        self.max_lin_vel = max_lin_vel
        self.min_lin_vel = min_lin_vel
        self.max_ang_vel = max_ang_vel
        self.min_ang_vel = min_ang_vel
        self.radius = radius
        self.trajectory = np.array([init_pose[0], init_pose[1], init_pose[2], 0.0, 0.0]).reshape(1, -1)

    def update_state(self, u, dt):
        if isinstance(u, list):
            u = np.array(u)
        self.vel = u
        next_x = self.pose[0] + self.vel[0] * math.cos(self.pose[2]) * dt
        next_y = self.pose[1] + self.vel[0] * math.sin(self.pose[2]) * dt
        next_th = self.pose[2] + self.vel[1] * dt
        self.pose = np.array([next_x, next_y, next_th])
        traj_state = np.array([next_x, next_y, next_th, self.vel[0], self.vel[1]]).reshape(1, -1)
        self.trajectory = np.concatenate([self.trajectory, traj_state], axis=0)
        return self.pose

class DWA:
    def __init__(self, dt, sim_time, time_granularity, v_samples, w_samples, goal_dist_tol, collision_tol, weight_angle, weight_vel, weight_obs, weight_target_dist, robot):
        self.dt = dt
        self.sim_step = round(sim_time / time_granularity)
        self.robot = robot
        self.goal_dist_tol = goal_dist_tol
        self.collision_tol = collision_tol
        self.v_samples = v_samples
        self.w_samples = w_samples
        self.weight_angle = weight_angle
        self.weight_vel = weight_vel
        self.weight_obs = weight_obs
        self.weight_target_dist = weight_target_dist

    def compute_cmd(self, goal_pose, robot_state, obstacles):
        paths, velocities = self.get_trajectories(robot_state)
        opt_idx = self.evaluate_paths(paths, velocities, goal_pose, obstacles)
        u = velocities[opt_idx]
        return u

    def get_trajectories(self, robot_pose):
        min_lin_vel, max_lin_vel, min_ang_vel, max_ang_vel = self.compute_dynamic_window(self.robot.vel)
        v_values = np.linspace(min_lin_vel, max_lin_vel, self.v_samples)
        w_values = np.linspace(min_ang_vel, max_ang_vel, self.w_samples)
        n_paths = w_values.shape[0] * v_values.shape[0]
        velocities = np.zeros((n_paths, 2))
        vv, ww = np.meshgrid(v_values, w_values)
        velocities = np.dstack([vv, ww]).reshape(n_paths, 2)
        sim_paths = self.simulate_paths(n_paths, robot_pose, velocities)
        return sim_paths, velocities

    def simulate_paths(self, n_paths, pose, u):
        sim_paths = np.zeros((n_paths, self.sim_step, pose.shape[0]))
        sim_paths[:, 0] = pose.copy()
        for i in range(1, self.sim_step):
            sim_paths[:, i, 0] = sim_paths[:, i - 1, 0] + u[:, 0] * np.cos(sim_paths[:, i - 1, 2]) * self.dt
            sim_paths[:, i, 1] = sim_paths[:, i - 1, 1] + u[:, 0] * np.sin(sim_paths[:, i - 1, 2]) * self.dt
            sim_paths[:, i, 2] = sim_paths[:, i - 1, 2] + u[:, 1] * self.dt
        return sim_paths

    def compute_dynamic_window(self, robot_vel):
        min_vel = max(self.robot.min_lin_vel, robot_vel[0] - self.dt * self.robot.max_linear_acc)
        max_vel = min(self.robot.max_lin_vel, robot_vel[0] + self.dt * self.robot.max_linear_acc)
        min_ang_vel = max(self.robot.min_ang_vel, robot_vel[1] - self.dt * self.robot.max_ang_acc)
        max_ang_vel = min(self.robot.max_ang_vel, robot_vel[1] + self.dt * self.robot.max_ang_acc)
        return min_vel, max_vel, min_ang_vel, max_ang_vel

    def evaluate_paths(self, paths, velocities, goal_pose, obstacles):
        score_heading_angles = self.score_heading_angle(paths, goal_pose)
        score_vel = self.score_vel(velocities, paths, goal_pose)
        score_obstacles = self.score_obstacles(paths, obstacles)
        score_target_dist = self.score_target_dist(paths, goal_pose)

        score_heading_angles = normalize(score_heading_angles)
        score_vel = normalize(score_vel)
        score_obstacles = normalize(score_obstacles)
        score_target_dist = normalize(score_target_dist)

        opt_idx = np.argmax(np.sum(
            np.array([score_heading_angles, score_vel, score_obstacles, score_target_dist])
            * np.array([[self.weight_angle, self.weight_vel, self.weight_obs, self.weight_target_dist]]).T,
            axis=0,
        ))
        return opt_idx

    def score_heading_angle(self, path, goal_pose):
        last_x = path[:, -1, 0]
        last_y = path[:, -1, 1]
        last_th = path[:, -1, 2]
        angle_to_goal = np.arctan2(goal_pose[1] - last_y, goal_pose[0] - last_x)
        score_angle = np.pi - np.fabs(normalize_angle(angle_to_goal - last_th))
        return score_angle

    def score_vel(self, u, path, goal_pose):
        dist_to_goal = np.linalg.norm(path[:, -1, 0:2] - goal_pose, axis=-1)
        vel = u[:, 0]
        # Slow down near the goal
        return vel * (1 - np.exp(-dist_to_goal / self.goal_dist_tol))

    def score_target_dist(self, path, goal_pose):
        dist_to_goal = np.linalg.norm(path[:, -1, 0:2] - goal_pose, axis=-1)
        # Ideal distance is 1.0 meters
        return 1.0 - np.abs(dist_to_goal - 1.0)

    def score_obstacles(self, path, obstacles):
        score_obstacle = np.full(path.shape[0], 2.0)
        if obstacles.size == 0:
            return score_obstacle
        for obs in obstacles:
            dx = path[:, :, 0] - obs[0]
            dy = path[:, :, 1] - obs[1]
            dist = np.hypot(dx, dy)
            min_dist = np.min(dist, axis=-1)
            score_obstacle = np.minimum(score_obstacle, min_dist)
        score_obstacle[score_obstacle < self.robot.radius + self.collision_tol] = -100
        return score_obstacle

class DWANode(Node):
    def __init__(self):
        super().__init__('dwa_node')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_subscriber = self.create_subscription(Odometry, '/dynamic_goal_pose', self.goal_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_publisher = self.create_publisher(Odometry, '/feedback', 10)
        
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)
        self.timer_started = False

        self.robot_pose = None
        self.scan_data = None
        self.goal_pose = None
        self.obstacles = np.array([])

        robot = DifferentialDriveRobot(
            init_pose=np.array([0.0, 0.0, 0.0]),
            max_linear_acc=0.5,
            max_ang_acc=math.pi/2,
            max_lin_vel=0.3,
            min_lin_vel=0.0,
            max_ang_vel=2.82,
            min_ang_vel=-2.82,
            radius=0.2
        )
        self.dwa = DWA(
            dt=1.0 / 15.0,
            sim_time=2.0,
            time_granularity=0.1,
            v_samples=10,
            w_samples=20,
            goal_dist_tol=0.2,
            collision_tol=0.15,
            weight_angle=0.06,
            weight_vel=0.2,
            weight_obs=0.1,
            weight_target_dist=0.1,
            robot=robot
        )
        self.control_steps = 0
        self.max_control_steps = 500

    def odom_callback(self, msg):
        self.get_logger().info('Odom received', once=True)
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation_q)
        self.robot_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
        self.dwa.robot.pose = self.robot_pose
        self.check_and_start_timer()

    def scan_callback(self, msg):
        self.get_logger().info('Scan received', once=True)
        self.scan_data = msg
        self.check_and_start_timer()

    def goal_callback(self, msg):
        self.get_logger().info('Goal received', once=True)
        self.goal_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.check_and_start_timer()

    def check_and_start_timer(self):
        if self.robot_pose is not None and self.scan_data is not None and self.goal_pose is not None:
            if not self.timer_started:
                self.get_logger().info('All topics received, starting timer.')
                self.timer_started = True

    def timer_callback(self):
        if not self.timer_started:
            return
        self.get_logger().info('Timer callback called', once=True)
        if self.robot_pose is None or self.scan_data is None or self.goal_pose is None:
            return

        self.process_scan()
        
        dist_to_goal = np.linalg.norm(self.robot_pose[0:2] - self.goal_pose)
        if dist_to_goal < self.dwa.goal_dist_tol:
            self.get_logger().info("Goal reached!")
            self.stop_robot()
            return

        if self.check_collision():
            self.get_logger().info("Collision imminent!")
            self.stop_robot()
            return

        if self.control_steps > self.max_control_steps:
            self.get_logger().info("Timeout!")
            self.stop_robot()
            return
        
        self.process_scan()
        
        u = self.dwa.compute_cmd(self.goal_pose, self.robot_pose, self.obstacles)
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = u[0]
        cmd_vel_msg.angular.z = u[1]
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        self.dwa.robot.update_state(u, self.dwa.dt)

        if self.control_steps % 50 == 0:
            feedback_msg = Odometry()
            feedback_msg.pose.pose.position.x = dist_to_goal
            self.feedback_publisher.publish(feedback_msg)
            self.get_logger().info(f"Current distance to goal: {dist_to_goal}")

        self.control_steps += 1

    def process_scan(self):
        if self.scan_data is None:
            return
        ranges = np.array(self.scan_data.ranges)
        ranges[np.isnan(ranges)] = self.scan_data.range_min
        ranges[np.isinf(ranges)] = self.scan_data.range_max
        ranges = np.clip(ranges, self.scan_data.range_min, 3.5)
        
        num_ranges = 20
        angle_increment = len(ranges) / num_ranges
        filtered_ranges = [np.min(ranges[int(i*angle_increment):int((i+1)*angle_increment)]) for i in range(num_ranges)]
        
        angles = np.linspace(self.scan_data.angle_min, self.scan_data.angle_max, num_ranges)
        
        obstacle_x = filtered_ranges * np.cos(angles + self.robot_pose[2]) + self.robot_pose[0]
        obstacle_y = filtered_ranges * np.sin(angles + self.robot_pose[2]) + self.robot_pose[1]
        self.obstacles = np.vstack((obstacle_x, obstacle_y)).T

    def check_collision(self):
        if self.scan_data is None:
            return False
        return np.min(np.array(self.scan_data.ranges)) < self.dwa.robot.radius + self.dwa.collision_tol

    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.timer.cancel()

    def euler_from_quaternion(self, q):
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    dwa_node = DWANode()
    rclpy.spin(dwa_node)
    dwa_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
