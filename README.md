# Overview

This ROS 2 package sits as the cherry on top of two other ROS 2 packages:
1. The [ez-turtlebot3 fork of OpenCR](https://github.com/ez-turtlebot3/OpenCR), which extends the TurtleBot3 OpenCR firmware to read data from its analog pins, A0-A5.
2. The [ez-turtlebot3 fork of turtlebot3](https://github.com/ez-turtlebot3/turtlebot3), which extends the turtlebot3_node to publish that analog pin data to the ROS 2 topic /analog_pins.

ez_analog_processor then takes /analog_pins data as input, processes that data, and publishes that processed data to several new topics.

# ROS Setup

## ROS package prerequisites

- ROS 2 Humble
- Python 3.10 or later

This is a standalone ROS 2 package, so it can be installed on the TurtleBot's Raspberry Pi or the remote PC. That said, the /analog_pins topic is published when the TurtleBot3 is running:
- [ez-turtlebot3's modified OpenCR firmware](https://github.com/ez-turtlebot3/OpenCR)
    - Find these installation instructions in the repo's README.
- [ez-turtlebot3's modified TurtleBot3 packages](https://github.com/ez-turtlebot3/turtlebot3)
    - Find these installation instructions in the repo's README.

## Building from Source

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/ez-turtlebot3/ez_analog_processor.git
```

2. Build the package:
```bash
cd ~/turtlebot3_ws
colcon build --packages-select ez_analog_processor
```

3. Source the workspace:
```bash
source ~/turtlebot3_ws/install/setup.bash
```

## ROS Configuration

Set up `config/sensors.yaml` for your hardware. This file is gitignored to prevent accidental commits of user-specific settings. The example files provide templates with default values that you can modify for your setup.

## ROS use

1. To start the node: `ros2 launch ez_analog_processor analog_processor.launch.py`
2. That's it! Maybe set up a live plot with [PlotJuggler](https://github.com/facontidavide/PlotJuggler).

# AWS IoT Setup

## AWS Prerequisites

* Someone on your team will have to set up your device on AWS IoT, on AWS.
* That someone will need to give you your
  - AWS IoT endpoint
  - AWS IoT certificates
  - AWS IoT device private key
  - Names for the MQTT topic and sensor type
* Include these things, or filepaths to them, in your bashrc file. Example export statements can be found in [bashrc_exports.example](aws_publisher/bashrc_exports.example)

## Create the environment for the AWS publisher

Publishing to AWS IoT requires the `awsiotsdk` Python package, which I could only install to my Raspberry Pi when I installed it in a virtual environment. The AWS IoT functionality and its virtual environment are kept separate from the rest of the ROS package. Follow these steps to set them up:

1. Complete the ROS Setup.
2. Create the virtual environment and install Python requirements inside.
```
cd ~/turtlebot3_ws
python -m venv .venv
source .venv/bin/activate
pip3 install -r src/ez_analog_processor/requirements.txt
```
3. You might have to source the workspace again, I'm not sure about this one.
`source ~/turtlebot3_ws/install/setup.bash`

## AWS use
1. Source the virtual environment if you don't have it active already
`source ~/turtlebot3_ws/.venv/bin/activate`
2. Run the script
`python3 ~/turtlebot3_ws/src/ez_analog_processor/aws_publisher/publish_analog_pins_data_to_AWS.py`

# Dependencies

This package has dependencies managed through multiple systems:

* ROS 2 dependencies
  * Managed through `package.xml`
  * Installed automatically by the ROS 2 build system

* Python dependencies
  * Managed through `requirements.txt`
  * Must be installed manually with: `pip3 install -r requirements.txt`

* Development dependencies for testing and code quality
  - `pytest` - Python testing framework
  - `pre-commit` - Git hooks for code quality
  - Install with: `pip3 install pytest pre-commit`

See `package.xml` and `requirements.txt` for complete dependency specifications.

# Setting up Pre-commit Hooks

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
# License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

# Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
