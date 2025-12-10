"""
ROS 2 Code Example Template

This template provides the standard structure for ROS 2 Python code examples in the textbook.
All code examples must be complete, runnable, and PEP 8 compliant.
"""

import rclpy
from rclpy.node import Node


class ExampleNode(Node):
    """
    Example ROS 2 node demonstrating [specific concept].

    Replace this docstring with a description of what this node does.
    """

    def __init__(self):
        super().__init__('example_node')
        self.get_logger().info('Example node initialized')

        # Add your node initialization here
        # Examples:
        # - Create publishers: self.publisher_ = self.create_publisher(...)
        # - Create subscribers: self.subscription = self.create_subscription(...)
        # - Create timers: self.timer = self.create_timer(...)
        # - Declare parameters: self.declare_parameter(...)


def main(args=None):
    """Main entry point for the ROS 2 node."""
    rclpy.init(args=args)
    node = ExampleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
