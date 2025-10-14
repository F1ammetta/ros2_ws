import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class Publisher(Node):
    _last_i = 0
    _n = 1
    _j = 0

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear = Vector3()


        if self._j == 0:
            msg.linear.x = 1.0
            msg.linear.y = 0.0
        if self._j == 1:
            msg.linear.y = 1.0
            msg.linear.x = 0.0

        if self._j == 2:
            msg.linear.x = -1.0
            msg.linear.y = 0.0
        if self._j == 3:
            msg.linear.y = -1.0
            msg.linear.x = 0.0


        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x:{msg.linear.x} y:{msg.linear.y}')
        self.i += 1
        if self.i - self._last_i == self._n:
            self._j+= 1
            self._last_i = self.i
        if self._j > 3:
            self._n += 1
            self._j = 0



def main(args=None):
    rclpy.init(args=args)

    publisher = Publisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
