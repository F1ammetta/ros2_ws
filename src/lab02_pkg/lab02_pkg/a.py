import rclpy
import tf_transformations
from rclpy.node import Node
import math

import controller


        



def main(args=None):
    rclpy.init(args=args)

    controller = controller.Controller()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
