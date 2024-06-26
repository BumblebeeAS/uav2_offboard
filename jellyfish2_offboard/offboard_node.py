import rclpy
from rclpy.node import Node


class OffboardNode(Node): 
    def __init__(self):
        super().__init__("offboard_node")
        self.get_logger().info("helo")


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()