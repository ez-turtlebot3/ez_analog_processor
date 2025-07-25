# Agent Instructions for ROS 2 Analog Data Processor

## Build/Test Commands
These commands must be entered from the top-level directory of the workspace, which is `$TB3_WS`
- `colcon build` - Build all packages in workspace. Can be time-intensive if executed on the Raspberry Pi.
- `colcon build --packages-select <ez_analog_processor>` - Build just this package
- `colcon test` - Run all tests
- `colcon test --packages-select <ez_analog_processor>` - Run tests for just this package
- `python3 -m pytest test/` - Run pytest tests in a package directory
- `ament_flake8 .` - Check PEP8 style compliance
- `ament_pep257 .` - Check docstring compliance

## Architecture
- **ROS 2 Humble package** for processing analog data
- **aws_publisher** exists outside of the ROS 2 package architecture in its own directory. Once the ROS 2 package is running, the aws publisher can push this data to AWS through MQTT

## Code Style
- **Python packages** using ament_python build type
- Standard ROS 2 package structure: package.xml, setup.py, resource/, test/
- Launch files in `launch/` directory with `.launch.py` extension
- Import order: stdlib, third-party, ROS packages, local imports
- Use snake_case for variables, PascalCase for classes
- Follow PEP8 and PEP257 standards (enforced by ament_flake8/ament_pep257)
- Entry points defined in setup.py console_scripts section
