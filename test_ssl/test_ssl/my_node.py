#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from grsim_ros_bridge_msgs.msg import SSL

class TestSSL(Node):
    def __init__(self):
        super().__init__('test_ssl')
        self.publisher = self.create_publisher(SSL, '/robot_blue_0/cmd', 10)
        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = SSL()
        msg.cmd_vel.linear.x = 0.1
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: %s' % msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestSSL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()