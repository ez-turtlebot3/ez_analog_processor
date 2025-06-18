#!/usr/bin/env python3
"""
ROS2 Node that subscribes to analog pin data and publishes it to AWS IoT Core.

This script creates a ROS2 node that:
1. Subscribes to the '/processed_analog' topic to receive Float32MultiArray messages
2. Rate limits publishing to 4 Hz (configurable)
3. Connects to AWS IoT Core using MQTT
4. Publishes received sensor data to AWS IoT Core in JSON format

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
- PUBLISH_RATE_HZ: Publishing rate in Hz (default: 4.0)

Usage:
    python3 publish_analog_pin_data_to_AWS.py
"""

import datetime
import json
import logging
import os
import signal
import time
import uuid
from concurrent.futures import Future

import yaml
from awscrt import mqtt
from awsiot import mqtt_connection_builder
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class Config:
    """Configuration management class using environment variables."""

    def __init__(self):
        # Required environment variables
        self.aws_iot_endpoint = os.getenv('AWS_IOT_ENDPOINT')
        if not self.aws_iot_endpoint:
            raise ValueError("AWS_IOT_ENDPOINT environment variable is required")

        # Optional environment variables with defaults
        self.device_certificate = os.getenv('DEVICE_CERTIFICATE', "/greengrass/v2/device.pem.crt")
        self.device_private_key = os.getenv('DEVICE_PRIVATE_KEY', "/greengrass/v2/private.pem.key")
        self.amazon_root_certificate = os.getenv(
            'AMAZON_ROOT_CERTIFICATE', "/greengrass/v2/AmazonRootCA1.pem"
            )
        self.mqtt_topic = os.getenv('MQTT_TOPIC', "ez-tb/sensordata/analog")
        self.sensor_type = os.getenv('SENSOR_TYPE', "Analog")
        self.publish_rate_hz = float(os.getenv('PUBLISH_RATE_HZ', "4.0"))

        # Fixed configuration
        self.mqtt_port = 8883
        self.device_id = self._get_device_name()
        self.mqtt_client_id = f"{self.device_id}-{uuid.uuid4()}"
        self.publish_interval = 1.0 / self.publish_rate_hz

        # Validate configuration
        self._validate()

    def _get_device_name(self) -> str:
        """Get device name from Greengrass config."""
        try:
            with open('/greengrass/v2/config.yaml', 'r') as f:
                config = yaml.safe_load(f)
                return config['system']['thingName']
        except Exception as e:
            logging.error(f"Failed to read device name from Greengrass config: {e}")
            return f"unknown-device-{uuid.uuid4().hex[:8]}"

    def _validate(self):
        """Validate configuration values."""
        if self.publish_rate_hz <= 0:
            raise ValueError("PUBLISH_RATE_HZ must be positive")
        if self.publish_rate_hz > 100:
            logging.warning(
                f"High publish rate ({self.publish_rate_hz} Hz) may cause performance issues"
                )


class RateLimiter:
    """Simple rate limiter to control publishing frequency."""

    def __init__(self, rate_hz: float):
        self.interval = 1.0 / rate_hz
        self.last_publish_time = 0.0

    def can_publish(self) -> bool:
        """Check if enough time has passed to allow publishing."""
        current_time = time.time()
        if current_time - self.last_publish_time >= self.interval:
            self.last_publish_time = current_time
            return True
        return False


class AWSIoTManager:
    """Manages AWS IoT MQTT connection and publishing."""

    def __init__(self, config: Config):
        self.config = config
        self.connection = None
        self.connection_established = Future()
        self._setup_logging()

    def _setup_logging(self):
        """Set up logging configuration."""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler(),
                logging.FileHandler('aws_publisher.log')
            ]
        )
        self.logger = logging.getLogger(__name__)

    def _on_connection_success(self, connection, callback_data):
        """Handle successful connection."""
        self.logger.info(f"Successfully connected to {self.config.aws_iot_endpoint}")
        self.connection_established.set_result(True)

    def _on_connection_failure(self, connection, error):
        """Handle connection failure."""
        self.logger.error(f"Connection failed: {error}")
        self.connection_established.set_result(False)

    def _on_connection_interrupted(self, connection, error):
        """Handle connection interruption."""
        self.logger.warning(f"Connection interrupted: {error}")

    def _on_connection_resumed(self, connection, return_code, session_present):
        """Handle connection resumption."""
        self.logger.info(
            f"Connection resumed. Return code: {return_code}, Session present: {session_present}"
            )

    def build_connection(self) -> bool:
        """Build and return MQTT connection."""
        self.logger.info("Building MQTT connection...")
        try:
            self.connection = mqtt_connection_builder.mtls_from_path(
                endpoint=self.config.aws_iot_endpoint,
                port=self.config.mqtt_port,
                cert_filepath=self.config.device_certificate,
                pri_key_filepath=self.config.device_private_key,
                ca_filepath=self.config.amazon_root_certificate,
                client_id=self.config.mqtt_client_id,
                clean_session=True,
                keep_alive_secs=30,
                on_connection_success=self._on_connection_success,
                on_connection_failure=self._on_connection_failure,
                on_connection_interrupted=self._on_connection_interrupted,
                on_connection_resumed=self._on_connection_resumed,
            )
            return True
        except Exception as e:
            self.logger.error(f"Failed to build MQTT connection: {e}")
            return False

    def connect(self) -> bool:
        """Connect to AWS IoT Core."""
        if not self.connection:
            return False

        self.logger.info(f"Connecting to {self.config.aws_iot_endpoint}...")
        try:
            connect_future = self.connection.connect()
            connect_future.result(timeout=10.0)
            return self.connection_established.result(timeout=10.0)
        except Exception as e:
            self.logger.error(f"Failed to connect: {e}")
            return False

    def publish(self, message: dict) -> bool:
        """Publish message to AWS IoT Core."""
        if not self.connection:
            return False

        try:
            message_json = json.dumps(message)
            publish_future, packet_id = self.connection.publish(
                topic=self.config.mqtt_topic,
                payload=message_json,
                qos=mqtt.QoS.AT_LEAST_ONCE
            )

            publish_future.result(timeout=5.0)
            self.logger.debug(f"Published message with packet ID {packet_id}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to publish message: {e}")
            return False

    def disconnect(self):
        """Disconnect from AWS IoT Core."""
        if self.connection:
            self.logger.info("Disconnecting from AWS IoT Core...")
            try:
                disconnect_future = self.connection.disconnect()
                disconnect_future.result(timeout=5.0)
                self.logger.info("Disconnected successfully")
            except Exception as e:
                self.logger.error(f"Error during disconnect: {e}")


class AnalogPinDataPublisher(Node):
    """ROS2 node that subscribes to analog data and publishes to AWS IoT Core."""

    def __init__(self, aws_manager: AWSIoTManager, config: Config):
        super().__init__('analog_pin_data_publisher')
        self.aws_manager = aws_manager
        self.config = config
        self.rate_limiter = RateLimiter(config.publish_rate_hz)
        self.message_count = 0
        self.last_message_time = None

        # Create subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/processed_analog',
            self.sensor_callback,
            10
        )

        # Create timer for health monitoring
        self.health_timer = self.create_timer(30.0, self.health_check)

        self.get_logger().info(
            f'Analog pin data publisher initialized with {config.publish_rate_hz} Hz rate limit'
            )

    def sensor_callback(self, msg):
        """Handle incoming sensor data."""
        if not self.rate_limiter.can_publish():
            self.get_logger().debug("Rate limited - skipping message")
            return

        try:
            utc_datetime = datetime.datetime.now()
            message = {
                "sensor_readings": list(msg.data),
                "device_id": self.config.device_id,
                "timestamp": utc_datetime.isoformat(),
                "sensor_type": self.config.sensor_type,
                "message_id": self.message_count
            }

            if self.aws_manager.publish(message):
                self.message_count += 1
                self.last_message_time = utc_datetime
                self.get_logger().info(
                    f"Published message {self.message_count} with "
                    f"{len(msg.data)} sensor readings"
                    )
            else:
                self.get_logger().error("Failed to publish message to AWS IoT Core")

        except Exception as e:
            self.get_logger().error(f"Error processing sensor data: {e}")

    def health_check(self):
        """Periodic health check."""
        if self.last_message_time:
            time_since_last = (datetime.datetime.now() - self.last_message_time).total_seconds()
            if time_since_last > 60:  # No messages in the last minute
                self.get_logger().warning(
                    f"No messages published in {time_since_last:.1f} seconds"
                    )

        self.get_logger().info(f"Health check: {self.message_count} messages published")


class Application:
    """Main application class."""

    def __init__(self):
        self.config = None
        self.aws_manager = None
        self.node = None
        self.running = False
        self.logger = None

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        if self.logger:
            self.logger.info(f"Received signal {signum}, shutting down...")
        self.running = False

    def initialize(self) -> bool:
        """Initialize the application."""
        try:
            self.config = Config()
            self.aws_manager = AWSIoTManager(self.config)
            self.logger = logging.getLogger(__name__)

            if not self.aws_manager.build_connection():
                return False

            if not self.aws_manager.connect():
                return False

            rclpy.init()
            self.node = AnalogPinDataPublisher(self.aws_manager, self.config)

            return True
        except Exception as e:
            logging.error(f"Failed to initialize application: {e}")
            return False

    def run(self):
        """Run the main application loop."""
        if not self.initialize():
            self.logger.error("Failed to initialize application")
            return

        self.logger.info("Starting analog pin data publisher...")
        self.running = True

        try:
            while self.running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=1.0)
        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt received")
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources."""
        self.logger.info("Cleaning up...")

        if self.node:
            self.node.destroy_node()

        rclpy.shutdown()

        if self.aws_manager:
            self.aws_manager.disconnect()

        self.logger.info("Cleanup complete")


def main():
    """Run the main application."""
    app = Application()
    app.run()


if __name__ == '__main__':
    main()
