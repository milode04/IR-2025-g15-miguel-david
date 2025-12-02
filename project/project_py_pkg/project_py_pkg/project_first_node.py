#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.number_ = 2
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Velocity publisher has been started.")
    
    def move_robot(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_publisher_.publish(msg)
    
    def scan_callback(self, msg):
        ranges = msg.ranges
        # Example: read front, left, and right distances
        front = msg.ranges[0]
        left = msg.ranges[90]
        right = msg.ranges[270]

        if front > 0.1:
            self.move_robot(0.1, 0.0) 

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()