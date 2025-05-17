#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile

class AutonomousExplorationNode(Node):
    def __init__(self):
        super().__init__('autonomous_exploration_node')

        # Subscriber to LaserScan
        self.subscriber = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Publisher for movement commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize state variables
        self.turning = False
        self.turn_direction = -0.5  # Default to turning right

        self.get_logger().info("Autonomous Exploration Node Ready...")

    def laserscan_callback(self, msg):
        # Define the sectors
        sectors = {
            "Right_Rear": (0, 33),
            "Right": (34, 66),
            "Front_Right": (67, 100),
            "Front_Left": (101, 133),
            "Left": (134, 166),
            "Left_Rear": (167, 199)
        }

        # Initialize the minimum distances for each sector
        min_distances = {key: float('inf') for key in sectors.keys()}

        # Find the minimum distance in each sector
        for sector, (start_idx, end_idx) in sectors.items():
            # Ensure the index range is within bounds and not empty
            if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                sector_ranges = msg.ranges[start_idx:end_idx + 1]
                if sector_ranges:
                    min_distances[sector] = min(sector_ranges)

        # Define the threshold for obstacle detection
        obstacle_threshold = 0.8  # meters

        # Determine detected obstacles
        detections = {sector: min_distance < obstacle_threshold for sector, min_distance in min_distances.items()}

        # Determine suggested action based on detection
        action = Twist()

        # If obstacles are detected in both front sectors, continue turning
        if detections["Front_Left"] or detections["Front_Right"]:
            if not self.turning:
                # Start turning if not already turning
                self.turning = True
                self.turn_direction = -0.5  # Turning right
            action.angular.z = self.turn_direction  # Continue turning
            self.get_logger().info('Obstacle ahead, turning to clear path.')
        else:
            self.turning = False  # Stop turning when the front is clear
            # Priority 2: Side detections
            if detections["Left"]:
                action.linear.x = 0.2  # Move forward slowly
                action.angular.z = -0.3  # Slight right turn
                self.get_logger().info('Obstacle on the left, turning slightly right.')
            elif detections["Right"]:
                action.linear.x = 0.2  # Move forward slowly
                action.angular.z = 0.3  # Slight left turn
                self.get_logger().info('Obstacle on the right, turning slightly left.')
            # Priority 3: Rear detections
            elif detections["Right_Rear"]:
                action.linear.x = 0.3  # Move forward
                self.get_logger().info('Obstacle on the right rear, moving forward.')
            elif detections["Left_Rear"]:
                action.linear.x = 0.3  # Move forward
                self.get_logger().info('Obstacle on the left rear, moving forward.')
            else:
                action.linear.x = 0.5  # Move forward
                self.get_logger().info('No obstacles, moving forward.')

        # Publish the action command
        self.publisher_.publish(action)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()