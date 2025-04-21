import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32MultiArray
import numpy as np
from rclpy.timer import Timer

class AnalogProcessor(Node):
    def __init__(self):
        super().__init__('analog_processor')
        
        # Data buffer to store incoming measurements
        self.data_buffer = []
        
        # Subscribe to raw analog pins data
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            '/analog_pins',
            self.collect_analog_data,
            10)
            
        # Publisher for processed data
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/processed_analog_input',
            10)
            
        # Create 1Hz timer for publishing
        self.timer = self.create_timer(1.0, self.process_and_publish)
            
        self.get_logger().info('Analog processor node initialized - averaging data at 1Hz')
        
    def collect_analog_data(self, msg):
        """Collect incoming data points into the buffer"""
        # Add new data to the buffer
        self.data_buffer.append(list(msg.data))
        self.get_logger().debug(f'Received data: {msg.data}, buffer size: {len(self.data_buffer)}')
    
    def process_and_publish(self):
        """Process buffered data and publish at 1Hz"""
        if not self.data_buffer:
            self.get_logger().warn('No data collected in the last second')
            return
            
        # Convert buffer to numpy array for easier averaging
        data_array = np.array(self.data_buffer)
        
        # Calculate average for each of the 6 analog pins
        avg_values = np.mean(data_array, axis=0)
        
        # Convert 10-bit ADC values (0-1023) to voltage (0-5V)
        processed_data = [float(value) * (5.0/1023.0) for value in avg_values]
        
        # Create and publish the processed data
        processed_msg = Float32MultiArray()
        processed_msg.data = processed_data
        self.publisher.publish(processed_msg)
        
        # Log information (use actual value count for accuracy)
        samples_count = len(self.data_buffer)
        self.get_logger().info(f'Published processed data from {samples_count} samples: {processed_data}')
        
        # Clear the buffer for the next cycle
        self.data_buffer = []

def main(args=None):
    rclpy.init(args=args)
    
    node = AnalogProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()