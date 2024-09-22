#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquarePathPublisher(Node):
    def __init__(self):
        super().__init__('square_path_publisher')
        self.publisher_ = self.create_publisher(Twist, '/j100_0001/cmd_vel', 10)
        self.cmd = Twist()

        self.state = 0
        self.move_start_time = time.time()
        self.counter = 0
        self.finish = False
        self.timer = self.create_timer(0.1, self.move_in_square) 

    def move_in_square(self):
        current_time = time.time()
        elapsed_time = current_time - self.move_start_time
        if self.counter < 4:
            if self.state % 2 == 0: #move
                self.cmd.linear.x = 0.5
                self.cmd.angular.z = 0.0
            else:  #rotate
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.5
            self.publisher_.publish(self.cmd)
            if self.state % 2 == 0 and elapsed_time > 4.0: 
                self.state += 1
                self.move_start_time = current_time
            elif self.state % 2 == 1 and elapsed_time > 3.16:  # value 3.16 is better than 3.14
                self.state += 1
                self.move_start_time = current_time
                self.counter += 1
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Movement completed. Shutting down the node.')

            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    square_path_publisher = SquarePathPublisher()
#        rclpy.spin_once(square_path_publisher, timeout_sec=0.1)
    rclpy.spin(square_path_publisher)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
