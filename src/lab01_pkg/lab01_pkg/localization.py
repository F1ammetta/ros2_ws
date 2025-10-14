import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point


class Subscriber(Node):
    _x = 0.0
    _y = 0.0

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
        self.i = 0

    def listener_callback(self, msg):
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

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
