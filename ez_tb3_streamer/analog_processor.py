import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
from collections import deque


class AnalogProcessor(Node):
    def __init__(self):
        super().__init__('analog_processor')

        # Data buffer to store incoming measurements
        self.data_buffer = []

        # Moving buffer for each sensor (35 points)
        self.sensor_buffers = {}

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

                ('sensors.s1.pin', 2),
                ('sensors.s1.enabled', True),
                ('sensors.s1.unit', 'V'),
                ('sensors.s1.conversion', 'voltage'),

                ('sensors.s2.pin', 3),
                ('sensors.s2.enabled', True),
                ('sensors.s2.unit', 'V'),
                ('sensors.s2.conversion', 'voltage'),

                ('sensors.s3.pin', 4),
                ('sensors.s3.enabled', True),
                ('sensors.s3.unit', 'V'),
                ('sensors.s3.conversion', 'voltage'),

                ('publish_diagnostic_array', True),
                ('publish_float_array', True),
                ('publish_mean_analog', True),
                ('voltage_offset', 0.0),  # Default voltage offset of 0.0V
                ('temperature_offset', 0.0),
                ('humidity_offset', 0.0),
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

        self.get_logger().info(
            f'Analog processor node initialized - averaging data at {self.update_rate} Hz'
        )
        self.get_logger().info(
            f'Active sensors: {[s["name"] for s in self.sensors.values() if s["enabled"]]}'
        )
        self.get_logger().info(f'Voltage offset: {self.voltage_offset}V')

    def load_config(self):
        """Load sensor configuration from parameters."""
        self.sensors = {}

        # Create sensor configurations from parameters
        for sensor in ['temp', 'rh', 's1', 's2', 's3']:
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

            # Initialize moving buffer for each enabled sensor
            if enabled:
                self.sensor_buffers[pin] = deque(maxlen=35)

        # Filter to only enabled sensors and sort by pin
        self.enabled_sensors = {k: v for k, v in self.sensors.items() if v["enabled"]}

        # Get voltage offset parameter
        self.voltage_offset = self.get_parameter('voltage_offset').value

        # Get temperature and humidity offset parameters
        self.temperature_offset = self.get_parameter('temperature_offset').value
        self.humidity_offset = self.get_parameter('humidity_offset').value

        # Get update rate parameter
        self.update_rate = self.get_parameter('update_rate').value

        # Check for publishing options
        self.publish_diagnostic = self.get_parameter('publish_diagnostic_array').value
        self.publish_float_array = self.get_parameter('publish_float_array').value
        self.publish_mean_analog = self.get_parameter('publish_mean_analog').value

    def calculate_trimmean(self, values):
        """Calculate the trimmean of values, using middle 15 values if 35 points available."""
        if len(values) < 35:
            return np.mean(values)

        sorted_values = np.sort(values)
        # Take middle 15 values (indices 10-24)
        middle_values = sorted_values[10:25]
        return np.mean(middle_values)

    def collect_analog_data(self, msg):
        """Collect incoming data points into the buffer."""
        # Add new data to the buffer
        self.data_buffer.append(list(msg.data))

        # Update moving buffers for each sensor
        for pin in self.enabled_sensors:
            if pin < len(msg.data):
                self.sensor_buffers[pin].append(msg.data[pin])

        self.get_logger().debug(f'Received data: {msg.data}, buffer size: {len(self.data_buffer)}')

    def process_and_publish(self):
        """Process buffered data and publish at configured rate."""
        if not self.data_buffer:
            self.get_logger().warn(
                f'No data collected in the last {1.0/self.update_rate} seconds'
            )
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
                # Get the trimmean of the moving buffer
                raw_value = self.calculate_trimmean(list(self.sensor_buffers[pin]))

                # Convert to voltage first (0-4095 ADC to 0-3.3V) with offset
                voltage = float(raw_value) * (3.3/4095.0) + self.voltage_offset

                # Apply specific conversion if needed
                if sensor["conversion"] == "voltage":
                    # Keep as voltage
                    value = voltage
                elif sensor["conversion"] == "humidity":
                    # RH (%) = -12.5 + 125 * V/3.3  => Equation from sensor documentation
                    value = (-12.5 + self.humidity_offset) + 125 * voltage / 3.3
                elif sensor["conversion"] == "temperature":
                    # T (°C) = -66.875 + 218.75 * V/3.3  => Equation from sensor documentation
                    value = (-66.875 + self.temperature_offset) + 218.75 * voltage / 3.3
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
        samples_count = len(next(iter(self.sensor_buffers.values()))) if self.sensor_buffers else 0
        self.get_logger().info(f'Published processed data from {samples_count} samples')

        # Clear the buffer for the next cycle
        self.data_buffer = []

    def publish_diagnostic_array(self, processed_values):
        """Publish data as DiagnosticArray with names and units."""
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
        """Publish processed values as Float32MultiArray."""
        msg = Float32MultiArray()
        msg.data = values
        self.float_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AnalogProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
