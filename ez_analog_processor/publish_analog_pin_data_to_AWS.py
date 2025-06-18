"""
ROS2 Node that subscribes to analog pin data and publishes it to AWS IoT Core.

This script creates a ROS2 node that:
1. Subscribes to the '/processed_analog' topic to receive Float32MultiArray messages
2. Connects to AWS IoT Core using MQTT
3. Publishes received sensor data to AWS IoT Core in JSON format

The published JSON message includes:
- sensor_readings: List of analog pin values
- device_id: The Greengrass thing name
- timestamp: UTC timestamp of the reading
- sensor_type: Type of sensor (default: "Analog")

Dependencies:
- ROS2 (rclpy)
- AWS IoT SDK
- PyYAML

Environment Variables Required:
- AWS_IOT_ENDPOINT: AWS IoT Core endpoint
- DEVICE_CERTIFICATE: Path to device certificate (default: /greengrass/v2/device.pem.crt)
- DEVICE_PRIVATE_KEY: Path to private key (default: /greengrass/v2/private.pem.key)
- AMAZON_ROOT_CERTIFICATE: Path to root CA (default: /greengrass/v2/AmazonRootCA1.pem)
- MQTT_TOPIC: AWS IoT Core topic (default: ez-tb/sensordata/analog)
- SENSOR_TYPE: Type of sensor data (default: Analog)

Usage:
    python3 publish_analog_pin_data_to_AWS.py
"""

import datetime
import json
import logging
import uuid
from concurrent.futures import Future

import yaml
from awscrt import mqtt
from awsiot import mqtt_connection_builder
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def get_device_name():
    with open("/greengrass/v2/config.yaml", "r") as f:
        config = yaml.safe_load(f)
        thing_name = config["system"]["thingName"]
    return thing_name


# --- Logging Setup ---
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class AnalogPinDataPublisher(Node):
    def __init__(self):
        super().__init__("analog_pin_data_publisher")

        # Declare parameters
        self.declare_parameter("aws_iot.endpoint", "")
        self.declare_parameter(
            "aws_iot.device_certificate", "/greengrass/v2/device.pem.crt"
        )
        self.declare_parameter(
            "aws_iot.device_private_key", "/greengrass/v2/private.pem.key"
        )
        self.declare_parameter(
            "aws_iot.amazon_root_certificate", "/greengrass/v2/AmazonRootCA1.pem"
        )
        self.declare_parameter("aws_iot.mqtt_topic", "ez-tb/sensordata/analog")
        self.declare_parameter("aws_iot.sensor_type", "Analog")
        self.declare_parameter("aws_iot.mqtt_port", 8883)
        self.declare_parameter("aws_iot.publish_interval", 1.0)

        # Get parameters
        self.aws_endpoint = self.get_parameter("aws_iot.endpoint").value
        if not self.aws_endpoint:
            self.get_logger().error("AWS IoT endpoint not set in parameters")
            raise ValueError("AWS IoT endpoint must be set in parameters")

        self.device_certificate = self.get_parameter("aws_iot.device_certificate").value
        self.device_private_key = self.get_parameter("aws_iot.device_private_key").value
        self.amazon_root_certificate = self.get_parameter(
            "aws_iot.amazon_root_certificate"
        ).value
        self.mqtt_topic = self.get_parameter("aws_iot.mqtt_topic").value
        self.sensor_type = self.get_parameter("aws_iot.sensor_type").value
        self.mqtt_port = self.get_parameter("aws_iot.mqtt_port").value
        self.publish_interval = self.get_parameter("aws_iot.publish_interval").value

        # Initialize MQTT connection
        self.mqtt_connection = None
        self.connection_established = Future()
        self.setup_mqtt()

        # Create subscription
        self.subscription = self.create_subscription(
            Float32MultiArray, "/processed_analog", self.sensor_callback, 10
        )
        self.get_logger().info("Analog pin data publisher node initialized")

    def setup_mqtt(self):
        """Set up the MQTT connection to AWS IoT Core."""
        try:
            self.mqtt_connection = mqtt_connection_builder.mtls_from_path(
                endpoint=self.aws_endpoint,
                port=self.mqtt_port,
                cert_filepath=self.device_certificate,
                pri_key_filepath=self.device_private_key,
                ca_filepath=self.amazon_root_certificate,
                client_id=f"{get_device_name()}-{uuid.uuid4()}",
                clean_session=True,
                keep_alive_secs=30,
                on_connection_success=self.on_connection_success,
                on_connection_failure=self.on_connection_failure,
                on_connection_interrupted=self.on_connection_interrupted,
                on_connection_resumed=self.on_connection_resumed,
            )

            # Connect to AWS IoT Core
            self.get_logger().info(f"Connecting to {self.aws_endpoint}...")
            connect_future = self.mqtt_connection.connect()
            connect_future.result()

            if not self.connection_established.result():
                raise ConnectionError("Could not establish MQTT connection")

        except Exception as e:
            self.get_logger().error(f"Failed to setup MQTT connection: {e}")
            raise

    def sensor_callback(self, msg):
        """Process sensor data and publish to AWS IoT Core."""
        try:
            utc_datetime = datetime.datetime.now()
            message_json = json.dumps(
                {
                    "sensor_readings": list(msg.data),
                    "device_id": get_device_name(),
                    "timestamp": utc_datetime.strftime("%Y-%m-%d %H:%M:%S"),
                    "sensor_type": self.sensor_type,
                }
            )

            self.get_logger().debug(f"Publishing message to topic '{self.mqtt_topic}'")
            publish_future, packet_id = self.mqtt_connection.publish(
                topic=self.mqtt_topic, payload=message_json, qos=mqtt.QoS.AT_LEAST_ONCE
            )

            try:
                publish_future.result(timeout=5.0)
                self.get_logger().debug(
                    f"Message published successfully with packet ID {packet_id}"
                )
            except Exception as e:
                self.get_logger().warning(f"Publish timed out or failed: {e}")

        except Exception as e:
            self.get_logger().error(f"Error processing sensor data: {e}")

    def on_connection_success(self, connection, callback_data):
        self.get_logger().info(f"Connection Successful to endpoint {self.aws_endpoint}")
        self.connection_established.set_result(True)

    def on_connection_failure(self, connection, error):
        self.get_logger().error(f"Connection Failed with error: {error}")
        self.connection_established.set_result(False)

    def on_connection_interrupted(self, connection, error):
        self.get_logger().warning(
            f"Connection Interrupted with error: {error}. Reconnecting..."
        )

    def on_connection_resumed(self, connection, return_code, session_present):
        self.get_logger().info(
            f"Connection Resumed. Return code: {return_code}, Session present: {session_present}"
        )

    def destroy_node(self):
        """Clean up when node is destroyed."""
        if self.mqtt_connection:
            self.get_logger().info("Disconnecting MQTT...")
            disconnect_future = self.mqtt_connection.disconnect()
            disconnect_future.result()
            self.get_logger().info("Disconnected.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = AnalogPinDataPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
