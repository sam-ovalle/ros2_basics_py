#!/usr/bin/env python
import rclpy

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # print a message to the terminal
    print("Mars rover 1 is alive...")
    # shutdown the ROS communication
    rclpy.shutdown()

def main_shutdown(args=None):
    rclpy.init(args=args)
    print("Shutting down Mars rover 1...")
    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function