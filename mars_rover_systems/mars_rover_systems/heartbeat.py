#!/usr/bin/env python
import rclpy
import time
from rclpy.node import Node

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # Create a node
    node = Node('mars_rover_1')

    i = 0
    max_i = 50
    while i < max_i:
        i += 1
        time_stamp = time.time()
        # print a message to the terminal
        print(str(i)+":Mars rover 1 is alive..."+str(time_stamp))
        # Wait for 1 second
        time.sleep(1)
    
    # shutdown the ROS communication
    node.destroy_node()
    rclpy.shutdown()

def main_shutdown(args=None):
    rclpy.init(args=args)
    print("Shutting down Mars rover 1...")
    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function