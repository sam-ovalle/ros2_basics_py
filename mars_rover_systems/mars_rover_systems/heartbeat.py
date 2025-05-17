#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class HeartbeatNode(Node):
    def __init__(self, rover_name, timer_period=0.2):
        # call super() in the constructor to initialize the Node object
        # the parameter we pass is the node name
        self._rover_name = rover_name
        super().__init__(self._rover_name)
        # create a timer sending two parameters:
        # - the duration between two callbacks (0.2 seconds)
        # - the timer function (timer_callback)
        self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        ros_time_stamp = self.get_clock().now()
        self.get_logger().info(self._rover_name +" is alive..."+str(ros_time_stamp))

def main(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = HeartbeatNode(rover_name="mars_rover_1", timer_period=1.0)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()

def main2(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = HeartbeatNode(rover_name="mars_rover_2", timer_period=1.0)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()