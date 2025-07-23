import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import numpy as np
from collections import deque
from ez_interfaces.msg import ChemSensor


class AnalogProcessor(Node):
    """ROS 2 node for processing analog sensor data.

    Collects raw ADC values, applies statistical smoothing (trimmean),
    converts to physical units (temperature, humidity, voltage), and
    publishes as ChemSensor messages.
    """

    # Hardware constants
    ADC_MAX_VALUE = 4095
    ADC_VOLTAGE_RANGE = 3.3
    PINS = [0, 1, 2, 3, 4, 5]

    def __init__(self):
        super().__init__('analog_processor')

        # Define sensor configurations with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('board_id', 1),
                ('pin.0', 'temp'),
                ('pin.1', 'rh'),
                ('pin.2', 's1'),
                ('pin.3', 's2'),
                ('pin.4', 's3'),
                ('pin.5', 'ground'),
                ('voltage_offset', 0.0),  # Default voltage offset of 0.0V
                ('update_rate', 1.0),  # Default update rate of 1.0 Hz
                ('moving_buffer', 35),
                ('trimmean_percent', 50),
                ('temp_coefficients', [1.0] * 6),  # 3 boards × 2 coefficients each
                ('rh_coefficients', [1.0] * 6)
            ]
        )

        # Load configuration
        self.load_config()

        # Create moving buffers for each pin
        self.sensor_buffers = {pin: deque(maxlen=self.moving_buffer) for pin in self.PINS}

        # Subscribe to raw analog pins data
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            '/analog_pins',
            self.collect_analog_data,
            10)

        # Publisher for processed data as ChemSensor message
        self.publisher = self.create_publisher(
            ChemSensor,
            '/chem_sensor',
            10)

        # Create timer for publishing using the configured update rate
        update_period = 1.0 / self.update_rate  # Convert frequency to period
        self.timer = self.create_timer(update_period, self.process_and_publish)

        self.get_logger().info(
            f'Analog processor node initialized - averaging data at {self.update_rate} Hz'
        )

    def load_config(self):
        """Load sensor configuration from parameters."""
        # Map pins to sensors
        self.pin_map = {}
        for pin in self.PINS:
            pin_name = f"pin.{pin}"
            self.pin_map[pin] = self.get_parameter(pin_name).value

        # Get voltage offset parameter
        self.voltage_offset = self.get_parameter('voltage_offset').value

        # Get board ID and temperature/humidity coefficients
        self.board_id = self.get_parameter('board_id').value
        self.temp_coefficients = (self.get_parameter('temp_coefficients')
                                  .get_parameter_value().double_array_value)
        self.rh_coefficients = (self.get_parameter('rh_coefficients')
                                .get_parameter_value().double_array_value)

        # Get update rate and moving buffer parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.moving_buffer = self.get_parameter('moving_buffer').value
        self.trimmean_percent = self.get_parameter('trimmean_percent').value

    def collect_analog_data(self, msg):
        """Collect incoming data points into the buffer."""
        # Update moving buffers for each sensor
        for pin in self.PINS:
            if pin < len(msg.data):
                self.sensor_buffers[pin].append(msg.data[pin])

    def calculate_trimmean(self, values):
        """Calculate the trimmean of values in the buffer.

        Uses middle half of values once buffer is full.
        """
        # Return mean value if buffer is not full
        if len(values) < self.moving_buffer:
            return np.mean(values)

        # Sort values
        sorted_values = np.sort(values)

        # Determine slice indices
        percent_to_cut = self.trimmean_percent / 2
        first = round(self.moving_buffer * percent_to_cut / 100)
        last = self.moving_buffer - first
        # Take middle values
        middle_values = sorted_values[first:last]
        return np.mean(middle_values)

    def process_and_publish(self):
        """Process buffered data and publish at configured rate."""
        msg = ChemSensor()

        # Process data one pin at a time
        for pin in self.PINS:
            # Use pin map to determine which sensor is connected to this pin
            sensor = self.pin_map[pin]

            # Do nothing if pin is connected to ground, otherwise...
            if sensor != "ground":
                # Create reference to the part of the ChemSensor 'msg' for this sensor
                sensor_msg = getattr(msg, sensor)

                # Calculate mean of recorded bits and convert to voltage
                mean_bits = self.calculate_trimmean(self.sensor_buffers[pin])
                voltage = (mean_bits / self.ADC_MAX_VALUE * self.ADC_VOLTAGE_RANGE +
                           self.voltage_offset)

                # Assign fields of ChemSensor msg depending on sensor type
                match sensor:
                    case "temp" | "rh":
                        sensor_msg.base.mean_bits = mean_bits
                        sensor_msg.base.voltage = voltage
                        sensor_msg.base.pin = pin

                        # Convert units for environmental sensors
                        match sensor:
                            case "temp":
                                sensor_msg.celsius = self.calc_env_var(voltage, "temperature")
                            case "rh":
                                sensor_msg.percent = self.calc_env_var(voltage, "humidity")
                    case _:
                        sensor_msg.mean_bits = mean_bits
                        sensor_msg.voltage = voltage
                        sensor_msg.pin = pin

        # Add timestamp to message
        msg.header.stamp = self.get_clock().now().to_msg()
        # Publish ChemSensor message
        self.publisher.publish(msg)

        # Log publishing information
        samples_count = (len(next(iter(self.sensor_buffers.values())))
                         if self.sensor_buffers else 0)
        self.get_logger().info(f'Published processed data from {samples_count} samples')

    def calc_env_var(self, voltage, conversion_type):
        """Calculate humidity or temperature from voltage.

        Uses equations determined from experiments.
        """
        # extract coefficients corresponding to sensor board id
        # n_rows = 3  # 3 possible sensors
        n_cols = 2  # 2 coefficients for linear polynomial
        list_index = self.board_id - 1
        match conversion_type:
            case "humidity":
                coeffs_list = self.rh_coefficients
            case "temperature":
                coeffs_list = self.temp_coefficients
            case _:
                self.get_logger().warn(
                    f"Invalid conversion type: {conversion_type}. No coefficients loaded."
                )
                coeffs_list = []  # I'm not sure what to put for this case

        if 0 <= list_index < len(coeffs_list) / n_cols:  # Check bounds
            # Calculate the starting index of the desired row in the flat list
            start_index = list_index * n_cols
            # Slice the list to get the coefficients for the selected sensor
            c = coeffs_list[start_index:(start_index + n_cols)]

            self.get_logger().info(
                f"Using {conversion_type} coefficients for sensor {self.board_id}: {c}"
            )

        else:
            self.get_logger().warn(
                f"Invalid sensor ID: {self.board_id}. No coefficients loaded."
            )
            c = [0, 0]

        return c[0] * voltage + c[1]


def main(args=None):
    rclpy.init(args=args)
    node = AnalogProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
