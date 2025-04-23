import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class AnalogProcessor(Node):
    def __init__(self):
        super().__init__('analog_processor')
        
        # Data buffer to store incoming measurements
        self.data_buffer = []
        
        # Define sensor configurations with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sensors.temp.pin', 0),
                ('sensors.temp.enabled', True),
                ('sensors.temp.unit', '°C'),
                ('sensors.temp.conversion', 'temperature'),

                ('sensors.rh.pin', 1),
                ('sensors.rh.enabled', True),
                ('sensors.rh.unit', '%'),
                ('sensors.rh.conversion', 'humidity'),
                
                ('sensors.no2.pin', 2),
                ('sensors.no2.enabled', True), 
                ('sensors.no2.unit', 'V'),
                ('sensors.no2.conversion', 'voltage'),
                
                ('sensors.nh3.pin', 3),
                ('sensors.nh3.enabled', True),
                ('sensors.nh3.unit', 'V'),
                ('sensors.nh3.conversion', 'voltage'),
            
                ('sensors.co.pin', 4),
                ('sensors.co.enabled', True),
                ('sensors.co.unit', 'V'),
                ('sensors.co.conversion', 'voltage'),
                
                ('publish_diagnostic_array', True),
                ('publish_float_array', True),
                ('publish_mean_analog', True),  # New parameter to control mean_analog publishing
                ('voltage_offset', 0.0),  # Default voltage offset of 0.0V
                ('update_rate', 1.0)      # Default update rate of 1.0 Hz
            ]
        )
        
        # Load configuration
        self.load_config()
        
        # Subscribe to raw analog pins data
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            '/analog_pins',
            self.collect_analog_data,
            10)
            
        # Publisher for mean analog data (raw averaged values)
        self.mean_publisher = self.create_publisher(
            UInt16MultiArray,
            '/mean_analog',
            10)
        
        # Publisher for processed data as Float32MultiArray
        self.float_publisher = self.create_publisher(
            Float32MultiArray,
            '/processed_analog',
            10)
            
        # Publisher for diagnostic array (includes names and units)
        self.diag_publisher = self.create_publisher(
            DiagnosticArray,
            '/sensor_readings',
            10)
            
        # Create timer for publishing using the configured update rate
        update_period = 1.0 / self.update_rate  # Convert frequency to period
        self.timer = self.create_timer(update_period, self.process_and_publish)
            
        self.get_logger().info(f'Analog processor node initialized - averaging data at {self.update_rate} Hz')
        self.get_logger().info(f'Active sensors: {[s["name"] for s in self.sensors.values() if s["enabled"]]}')
        self.get_logger().info(f'Voltage offset: {self.voltage_offset}V')
        
    def load_config(self):
        """Load sensor configuration from parameters"""
        self.sensors = {}
        
        # Create sensor configurations from parameters
        for sensor in ['temp', 'rh', 'no2', 'nh3', 'co']:
            pin = self.get_parameter(f'sensors.{sensor}.pin').value
            enabled = self.get_parameter(f'sensors.{sensor}.enabled').value
            unit = self.get_parameter(f'sensors.{sensor}.unit').value
            conversion = self.get_parameter(f'sensors.{sensor}.conversion').value
            
            self.sensors[pin] = {
                "name": sensor,
                "enabled": enabled,
                "unit": unit,
                "conversion": conversion,
                "pin": pin
            }
        
        # Filter to only enabled sensors and sort by pin
        self.enabled_sensors = {k: v for k, v in self.sensors.items() if v["enabled"]}
        
        # Get voltage offset parameter
        self.voltage_offset = self.get_parameter('voltage_offset').value
        
        # Get update rate parameter
        self.update_rate = self.get_parameter('update_rate').value
        
        # Check for publishing options
        self.publish_diagnostic = self.get_parameter('publish_diagnostic_array').value
        self.publish_float_array = self.get_parameter('publish_float_array').value
        self.publish_mean_analog = self.get_parameter('publish_mean_analog').value
        
    def collect_analog_data(self, msg):
        """Collect incoming data points into the buffer"""
        # Add new data to the buffer
        self.data_buffer.append(list(msg.data))
        self.get_logger().debug(f'Received data: {msg.data}, buffer size: {len(self.data_buffer)}')
    
    def process_and_publish(self):
        """Process buffered data and publish at configured rate"""
        if not self.data_buffer:
            self.get_logger().warn(f'No data collected in the last {1.0/self.update_rate} seconds')
            return
            
        # Convert buffer to numpy array for easier averaging
        data_array = np.array(self.data_buffer)
        
        # Calculate average for each of the analog pins
        avg_values = np.mean(data_array, axis=0)
        
        # Publish mean analog values first (raw averaged values as integers)
        if self.publish_mean_analog:
            # Round values to integers for UInt16MultiArray
            int_avg_values = [int(round(x)) for x in avg_values]
            
            # Create and publish UInt16MultiArray message
            mean_msg = UInt16MultiArray()
            mean_msg.data = int_avg_values
            self.mean_publisher.publish(mean_msg)
            self.get_logger().debug(f'Published mean analog data: {int_avg_values}')
        
        # Create a dictionary to store processed values by sensor
        processed_values = {}
        float_array_values = []
        
        # Process each sensor according to its configuration
        for pin, sensor in self.enabled_sensors.items():
            if pin < len(avg_values):  # Ensure pin is within range of data
                # Convert to voltage first (0-1023 ADC to 0-5V) with offset
                voltage = float(avg_values[pin]) * (5.0/1023.0) + self.voltage_offset
                
                # Apply specific conversion if needed
                if sensor["conversion"] == "voltage":
                    # Keep as voltage
                    value = voltage
                elif sensor["conversion"] == "humidity":
                    # RH (%) = -12.5 + 125 * V/5
                    value = -12.5 + 125 * voltage / 5.0
                elif sensor["conversion"] == "temperature":
                    # T (°C) = -66.875 + 218.75 * V/5
                    value = -66.875 + 218.75 * voltage / 5.0
                else:
                    # Default to voltage
                    value = voltage
                
                processed_values[sensor["name"]] = {
                    "value": value,
                    "unit": sensor["unit"]
                }
                
                # Also add to flat array for the Float32MultiArray message
                float_array_values.append(value)
        
        # Publish diagnostic array with named values
        if self.publish_diagnostic:
            self.publish_diagnostic_array(processed_values)
            
        # Publish float array for backward compatibility or simple plotting
        if self.publish_float_array:
            self.publish_float_array_msg(float_array_values)
        
        # Log information
        samples_count = len(self.data_buffer)
        self.get_logger().info(f'Published processed data from {samples_count} samples')
        
        # Clear the buffer for the next cycle
        self.data_buffer = []

    def publish_diagnostic_array(self, processed_values):
        """Publish data as DiagnosticArray with names and units"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Create a status message for environmental sensors
        status = DiagnosticStatus()
        status.name = "Environmental Sensors"
        status.level = DiagnosticStatus.OK
        status.message = "Sensor readings"
        
        # Add each sensor reading as a key-value pair
        for sensor_name, data in processed_values.items():
            key_value = KeyValue()
            key_value.key = f"{sensor_name} ({data['unit']})"
            key_value.value = f"{data['value']:.2f}"
            status.values.append(key_value)
            
        diag_array.status.append(status)
        self.diag_publisher.publish(diag_array)

    def publish_float_array_msg(self, values):
        """Publish data as Float32MultiArray"""
        float_msg = Float32MultiArray()
        float_msg.data = values
        self.float_publisher.publish(float_msg)

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