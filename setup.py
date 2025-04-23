from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ez_tb3_streamer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trav',
    maintainer_email='travis.mendoza@eyezense.com',
    description='A package for processing and streaming TurtleBot3 sensor data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stream_bits = ez_tb3_streamer.stream_bits:main',
            'analog_processor = ez_tb3_streamer.analog_processor:main',
            'stream_processed = ez_tb3_streamer.stream_processed_analog:main',
        ],
    },
)