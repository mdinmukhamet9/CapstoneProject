#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveRobotOnce(Node):
    def __init__(self):
        super().__init__('move_robot_once')
        self.publisher_ = self.create_publisher(Twist, '/j100_0001/cmd_vel', 10)
        self.cmd = Twist()

        self.get_logger().info('Moving the robot for 5 seconds...')
        for i in range(4):
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            time.sleep(1)
            
        # Stop the robot after the movement
        self.stop_robot()

        # Shutdown the node
        self.get_logger().info('Movement completed. Shutting down the node.')
        rclpy.shutdown()

    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    move_robot_once = MoveRobotOnce()

    # Keep the node alive until it completes the task and shuts down
    rclpy.spin(move_robot_once)

if __name__ == '__main__':
    main()
