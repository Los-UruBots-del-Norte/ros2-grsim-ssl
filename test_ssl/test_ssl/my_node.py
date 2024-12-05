#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from grsim_ros_bridge_msgs.msg import SSL
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy
from krssg_ssl_msgs.msg import SSLDetectionFrame

NUMBER_OF_ROBOTS = 3

FORMATION_X = [-700, -1000, -700]
FORMATION_Y = [1000, 0, -1000]

class TestSSL(Node):
    def __init__(self):
        super().__init__('test_ssl')
        self.publisher = self.create_publisher(SSL, '/robot_blue_0/cmd', 10)

        self.subscription = self.create_subscription(SSLDetectionFrame, '/vision', self.position_callback, 
                                                        QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10) )
        self.subscription

        self.ball_pos_x = 0
        self.ball_pos_y = 0

        self.robots_blue_pos_x = [0]*NUMBER_OF_ROBOTS
        self.robots_blue_pos_y = [0]*NUMBER_OF_ROBOTS
        self.robots_orientation = [0]*NUMBER_OF_ROBOTS

    def position_callback(self, msg):

        if len(msg.balls) > 0:
            self.ball_pos_x = msg.balls[0].x
            self.ball_pos_y = msg.balls[0].y

        if len(msg.robots_blue) > 0:
            for i in range(0, len(msg.robots_blue)):
                self.robots_blue_pos_x[msg.robots_blue[i].robot_id] = msg.robots_blue[i].x
                self.robots_blue_pos_y[msg.robots_blue[i].robot_id] = msg.robots_blue[i].y
                self.robots_orientation[msg.robots_blue[i].robot_id] = msg.robots_blue[i].orientation

        goal_angle = math.atan2(self.ball_pos_y - self.robots_blue_pos_y[0], self.ball_pos_x - self.robots_blue_pos_x[0])
        heading = goal_angle - self.robots_orientation[0]
        distance = math.sqrt((self.ball_pos_y - self.robots_blue_pos_y[0])**2 + (self.ball_pos_x - self.robots_blue_pos_x[0])**2)

        self.get_logger().info('Ball position: ' + str(self.ball_pos_x) + ', ' + str(self.ball_pos_y))
        self.get_logger().info('Robot position: ' + str(self.robots_blue_pos_x[0]) + ', ' + str(self.robots_blue_pos_y[0]))
        self.get_logger().info('Heading: ' + str(heading))
        self.get_logger().info('---------------' )

        msg = SSL()

        if (abs(heading) > 0.1):
            msg.cmd_vel.angular.z = 0.5
        else:
            msg.cmd_vel.angular.z = 0.0

        if (distance > 0.1):
            if (abs(heading) < 0.1):
                msg.cmd_vel.linear.x = 0.5
        else:
            msg.cmd_vel.linear.x = 0.0

        self.publisher.publish(msg)

    def position_callback_move_1_robot_target(self, msg):

        if len(msg.robots_blue) > 0:
            for i in range(0, len(msg.robots_blue)):
                self.robots_blue_pos_x[msg.robots_blue[i].robot_id] = msg.robots_blue[i].x
                self.robots_blue_pos_y[msg.robots_blue[i].robot_id] = msg.robots_blue[i].y

        error_x = FORMATION_X[0] - self.robots_blue_pos_x[0]
        error_y = FORMATION_Y[0] - self.robots_blue_pos_y[0]

        msg = SSL()

        if (error_x < -50):
            msg.cmd_vel.linear.x = -0.5
        elif (error_x > 50):
            msg.cmd_vel.linear.x = 0.5
        else:
            msg.cmd_vel.linear.x = 0.0

        if (error_y < -50):
            msg.cmd_vel.linear.y = -0.5
        elif (error_y > 50):
            msg.cmd_vel.linear.y = 0.5
        else:
            msg.cmd_vel.linear.y = 0.0

        self.publisher.publish(msg)

        self.get_logger().info('I heard: "%s"' % error_x)
        self.get_logger().info('I heard: "%s"' % error_y)
        self.get_logger().info('---------------' )


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

