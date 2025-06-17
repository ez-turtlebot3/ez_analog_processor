# Overview

This ROS 2 package sits as the cherry on top of two other ROS 2 packages:
1. The [ez-turtlebot3 fork of OpenCR](https://github.com/ez-turtlebot3/OpenCR), which extends the TurtleBot3 OpenCR firmware to read data from its analog pins, A0-A5.
2. The [ez-turtlebot3 fork of turtlebot3](https://github.com/ez-turtlebot3/turtlebot3), which extends the turtlebot3_node to publish that analog pin data to the ROS 2 topic /analog_pins.

ez_analog_processor then takes /analog_pins data as input, processes that data, and publishes that processed data to several new topics.

## Installation

### Prerequisites

- ROS 2 Humble
- Python 3
- [ez-turtlebot3's modified OpenCR firmware](https://github.com/ez-turtlebot3/OpenCR)
    - Find these installation instructions in the repo's README.
- [ez-turtlebot3's modified TurtleBot3 packages](https://github.com/ez-turtlebot3/turtlebot3)
    - Find these installation instructions in the repo's README.

### Building from Source

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/yourusername/ez_analog_processor.git
```

2. Install dependencies:
```bash
cd ez_analog_processor
./install_data_deps.sh
```

3. Build the package:
```bash
cd ~/turtlebot3_ws
colcon build --packages-select ez_tb3_streamer
```

### Configuration

The package uses two configuration files that need to be set up for your specific environment:

1. `config/sensors.yaml`: Contains sensor-specific configuration
   - Pin mappings for your sensors
   - Calibration offsets
   - Update rates
   - Streaming settings
   - To set up:
     ```bash
     cp config/sensors.example.yaml config/sensors.yaml
     # Edit config/sensors.yaml with your sensor settings
     ```

2. `config/aws_iot.yaml`: Contains AWS IoT Core configuration
   - AWS IoT Core endpoint
   - Certificate paths
   - MQTT settings
   - To set up:
     ```bash
     cp config/aws_iot.example.yaml config/aws_iot.yaml
     # Edit config/aws_iot.yaml with your AWS IoT Core settings
     ```

Both configuration files are gitignored to prevent accidental commits of user-specific settings. The example files provide templates with default values that you can modify for your setup.

### Setting up Pre-commit Hooks

This repository uses pre-commit hooks to ensure code quality and prevent accidental commits of sensitive information. To set up the pre-commit hooks:

1. Install pre-commit:
```bash
pip3 install pre-commit
```

2. Install the git hooks:
```bash
pre-commit install
```

The hooks will now run automatically on each commit. They will:
- Check for accidental commits of secrets using gitleaks
- Format and lint Python code using ruff
- Check for common issues like trailing whitespace and merge conflicts

To manually run the hooks on all files:
```bash
pre-commit run --all-files
```

## Usage

[Add usage instructions here]

## Dependencies

- rclpy
- std_msgs
- diagnostic_msgs
- python3-numpy
- python3-yaml
- turtlebot3_bringup

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
