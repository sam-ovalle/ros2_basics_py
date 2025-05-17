import rclpy
from rclpy.node import Node
import random

class TemperatureMonitorNode(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.temperature_threshold = 70.0  # Set the temperature threshold
        self.get_logger().info('Temperature Monitor Node has been started.')
        
        # Create a timer that triggers every second (1 Hz)
        self.timer = self.create_timer(1.0, self.monitor_temperature_callback)

    def get_temperature(self):
        # Simulate getting a temperature reading (in reality, this would come from a sensor)
        temperature = random.uniform(20.0, 100.0)
        return temperature

    def monitor_temperature_callback(self):
        current_temperature = self.get_temperature()
        self.get_logger().info(f'Current temperature: {current_temperature:.2f}°C')

        if current_temperature > self.temperature_threshold:
            self.get_logger().warn(f'Warning: High temperature detected! {current_temperature:.2f}°C')

def start_monitor(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitorNode()

    try:
        rclpy.spin(node)  # Keep the node running, processing callbacks
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    start_monitor()