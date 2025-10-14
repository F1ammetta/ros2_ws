import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Bool, '/reset', 10)

    def listener_callback(self, msg):
        position: Point = msg.position
        dst = math.sqrt(position.x**2 + position.y**2)
        msg = Bool()

        if dst > 6:
            msg.data = True
        else:
            msg.data = False

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: reset:{msg.data}')








def main(args=None):
    rclpy.init(args=args)

    subscriber = PoseSubscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
