#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__('hello_world_subscriber')
        self.subscriber_ = self.create_subscription(String, 'hello_world', self.message_callback, 10)

    def message_callback(self, msg):
        print('Received message: ' + msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_node = HelloWorldSubscriber()
    print('Subscriber node is ready...')

    try:
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        print('Shutting down node')
        my_node.destroy_node()

if __name__ == '__main__':
    main()
