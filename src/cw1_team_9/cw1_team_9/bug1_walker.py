import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Bug1Walker(Node):

    def __init__(self):
        super().__init__('minimal_bug1_node')
        self.get_logger().info('Bug1: I am ready')


def main(args=None):
    rclpy.init(args=args)

    walker = Bug1Walker()

    rclpy.spin(walker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    walker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()