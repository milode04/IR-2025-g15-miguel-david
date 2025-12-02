#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class VelocityPublisherNode(Node):
    
    def __init__(self):
        super().__init__("number_publisher")
        self.number_ = 2
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Velocity publisher has been started.")

    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()