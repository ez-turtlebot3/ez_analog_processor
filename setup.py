from setuptools import find_packages, setup
import os
from glob import glob

package_name = "ez_analog_processor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    zip_safe=True,
    maintainer="trav",
    maintainer_email="travis.mendoza@eyezense.com",
    description="A package for processing and publishing TurtleBot3 sensor data",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "analog_processor = ez_analog_processor.analog_processor:main",
            "publish_to_AWS = ez_analog_processor.publish_analog_pin_data_to_AWS:main",
        ],
    },
    python_requires=">=3.8",
)
