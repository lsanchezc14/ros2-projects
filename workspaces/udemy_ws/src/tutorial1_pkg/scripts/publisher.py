#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_world', 10)
        self.timer_ = self.create_timer(0.5, self.publish_message)
        self.counter_ = 0

    def publish_message(self):
        msg = String()
        msg.data = 'Hello World' + str(self.counter_)
        self.publisher_.publish(msg)
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)
    my_node = HelloWorldPublisher()

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        print('Shutting down node')
        my_node.destroy_node()

if __name__ == '__main__':
    main()
