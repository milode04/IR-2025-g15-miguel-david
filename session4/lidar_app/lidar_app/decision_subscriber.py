#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
class DecisionSubscriberNode(Node):
    def __init__(self):
        super().__init__('decision_subscriber')
        self.left = 0
        self.right = 0
        self.left_sensor_subscriber_ = self.create_subscription(Int64, "left_sensor",
        self.callback_left, 10)
        self.get_logger().info("Number Counter has been started.")

        self.right_sensor_subscriber_ = self.create_subscription(Int64, "right_sensor",
        self.callback_right, 10)
        self.get_logger().info("Number Counter has been started.")

    def callback_left(self, msg1: Int64):
        self.left = msg1.data

    def callback_right(self, msg2: Int64):
        self.right = msg2.data
        

    def callback_decision(self):
        if (self.left<40 and self.right<40):
            self.get_logger().info("stop")
        elif (self.left<40 and self.right>40):
            self.get_logger().info("turn right")
        elif (self.left>40 and self.right<40):
            self.get_logger().info("turn left")
        else:
            self.get_logger().info("forward")

def main(args=None):
    rclpy.init(args=args)
    node = DecisionSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()