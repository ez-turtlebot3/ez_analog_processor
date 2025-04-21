from setuptools import find_packages, setup

package_name = 'ez_tb3_streamer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trav',
    maintainer_email='travis.mendoza@eyezense.com',
    description='A package to process and stream data collected by a TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stream_topic = ez_tb3_streamer.stream_ros2_topic_to_LAN:main',
            'analog_processor = ez_tb3_streamer.analog_processor:main',
        ],
    },
)
