#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
import random

class LidarPublisherNode(Node):
    
    def __init__(self):
        super().__init__("lidar_publisher")
        self.left_sensor_ = random.randint (2,200)
        self.left_sensor_publisher_ = self.create_publisher(Int64, "left_sensor", 10)
        self.left_sensor_timer_ = self.create_timer(1.0, self.publish_left_sensor)
        self.get_logger().info("Number publisher has been started.")
       
        self.right_sensor_ = random.randint (2,200)
        self.right_sensor_publisher_ = self.create_publisher(Int64, "right_sensor", 10)
        self.right_sensor_timer_ = self.create_timer(1.0, self.publish_right_sensor)
        self.get_logger().info("Number publisher has been started.")

    def publish_left_sensor(self):
        msg = Int64()
        msg.data = random.randint (2,200)
        self.left_sensor_publisher_.publish(msg)

    def publish_right_sensor(self):
        msg = Int64()
        msg.data = random.randint (2,200)
        self.right_sensor_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
