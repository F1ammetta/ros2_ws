import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

import threading


class Subscriber_r(Node):
    _x = 0.0
    _y = 0.0
    _reset = False

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.subscription2 = self.create_subscription(
            Bool,
            '/reset',
            self.listener_callback2,
            10)
        self.subscription2  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
        self.i = 0
        self._lock = threading.Lock()

    def listener_callback2(self, msg):
        if msg.data == True:
            with self._lock:
                self._reset = True
                self._x = 1.0
                self._y = 0.0
                self.get_logger().info(f'Reset position')
                self._reset = False
            

    def listener_callback(self, msg):
        with self._lock:
            if not self._reset:
                self._x += msg.linear.x
                self._y += msg.linear.y

                msg = Pose()
                msg.position = Point()
                msg.position.x = self._x
                msg.position.y = self._y

                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: x:{msg.position.x} y:{msg.position.y}')






def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber_r()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
