#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from grsim_ros_bridge_msgs.msg import SSL

from erforce.model import RobotCommand, MoveLocalVelocity
from erforce.client import ErForceClientBlue
from erforce.client import ErForceClientYellow
clientblue = ErForceClientBlue()
clientyellow = ErForceClientYellow()


KICKER_SPEED = 1.0
DIBBLER_SPEED = 1.0

class GrSimRosBridge(Node):
    def __init__(self):
        super().__init__('grsim_ros_bridge')
        self.get_logger().info("grsim_ros_bridge started")
        
        # Subscribers for robots blue
        self.create_subscription(SSL, 'robot_blue_0/cmd', self.cmd_blue_0_callback,10)
        self.create_subscription(SSL, 'robot_blue_1/cmd', self.cmd_blue_1_callback,10)
        self.create_subscription(SSL, 'robot_blue_2/cmd', self.cmd_blue_2_callback,10)
        self.create_subscription(SSL, 'robot_blue_3/cmd', self.cmd_blue_3_callback,10)
        self.create_subscription(SSL, 'robot_blue_4/cmd', self.cmd_blue_4_callback,10)

        # Subscribers for robots yellow
        self.create_subscription(SSL, 'robot_yellow_0/cmd', self.cmd_yellow_0_callback,10)
        self.create_subscription(SSL, 'robot_yellow_1/cmd', self.cmd_yellow_1_callback,10)
        self.create_subscription(SSL, 'robot_yellow_2/cmd', self.cmd_yellow_2_callback,10)
        self.create_subscription(SSL, 'robot_yellow_3/cmd', self.cmd_yellow_3_callback,10)
        self.create_subscription(SSL, 'robot_yellow_4/cmd', self.cmd_yellow_4_callback,10)


    def move_robot_blue(self, data, id):
        global clientblue

        move_command = MoveLocalVelocity(forward=data.cmd_vel.linear.x, left=data.cmd_vel.linear.y, angular=data.cmd_vel.angular.z)

        if (data.kicker and data.dribbler):
            robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=KICKER_SPEED, kick_angle=0, dribbler_speed=DIBBLER_SPEED)
        elif (not data.kicker and data.dribbler):
            robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=0, kick_angle=0, dribbler_speed=DIBBLER_SPEED)
        elif (data.kicker and not data.dribbler):
            robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=KICKER_SPEED, kick_angle=0, dribbler_speed=0)
        else:
            robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=0, kick_angle=0, dribbler_speed=0)

        clientblue.send_action_command(robot_command)

    def move_robot_yellow(self, data, id):
        global clientyellow

        move_command = MoveLocalVelocity(forward=data.cmd_vel.linear.x, left=data.cmd_vel.linear.y, angular=data.cmd_vel.angular.z)

        if (data.kicker and data.dribbler):
            robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=KICKER_SPEED, kick_angle=0, dribbler_speed=DIBBLER_SPEED)
        elif (not data.kicker and data.dribbler):
            robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=0, kick_angle=0, dribbler_speed=DIBBLER_SPEED)
        elif (data.kicker and not data.dribbler):
            robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=KICKER_SPEED, kick_angle=0, dribbler_speed=0)
        else:
            robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=0, kick_angle=0, dribbler_speed=0)

        clientyellow.send_action_command(robot_command)

    def cmd_blue_0_callback(self, data):
        self.move_robot_blue(data, 0)

    def cmd_blue_1_callback(self, data):
        self.move_robot_blue(data, 1)

    def cmd_blue_2_callback(self, data):
        self.move_robot_blue(data, 2)

    def cmd_blue_3_callback(self, data):
        self.move_robot_blue(data, 3)

    def cmd_blue_4_callback(self, data):
        self.move_robot_blue(data, 4)

    def cmd_yellow_0_callback(self, data):
        self.move_robot_yellow(data, 0)

    def cmd_yellow_1_callback(self, data):
        self.move_robot_yellow(data, 1)

    def cmd_yellow_2_callback(self, data):
        self.move_robot_yellow(data, 2)

    def cmd_yellow_3_callback(self, data):
        self.move_robot_yellow(data, 3)

    def cmd_yellow_4_callback(self, data):
        self.move_robot_yellow(data, 4)

def main(args = None):
    rclpy.init(args=args)
    node = GrSimRosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


