import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import numpy as np

class AnalogMeanProcessor(Node):
    def __init__(self):
        super().__init__('analog_mean_processor')
        
        # Buffer to store incoming data
        self.data_buffer = []
        
        # Subscribe to the raw analog pins topic
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            '/analog_pins',
            self.collect_analog_data,
            10)
            
        # Create publisher for the averaged data
        self.mean_publisher = self.create_publisher(
            UInt16MultiArray,
            '/mean_analog',
            10)
            
        # Create a timer that processes data at 1Hz
        self.timer = self.create_timer(1.0, self.process_and_publish)
        
        self.get_logger().info('Analog Mean Processor started - collecting data at raw rate, publishing at 1Hz')

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
            
        try:
            # Convert buffer to numpy array for easier averaging
            data_array = np.array(self.data_buffer)
            
            # Calculate average for each of the analog pins
            avg_values = np.mean(data_array, axis=0)
            
            # Round values to integers since original data is UInt16
            avg_values = [int(round(x)) for x in avg_values]
            
            # Create message
            msg = UInt16MultiArray()
            msg.data = avg_values
            
            # Publish averaged data
            self.mean_publisher.publish(msg)
            
            self.get_logger().debug(f'Published averaged data: {avg_values} from {len(self.data_buffer)} samples')
            
            # Clear the buffer after processing
            self.data_buffer = []
            
        except Exception as e:
            self.get_logger().error(f'Error processing data: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    mean_processor = AnalogMeanProcessor()
    
    try:
        rclpy.spin(mean_processor)
    except KeyboardInterrupt:
        pass
    finally:
        mean_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()