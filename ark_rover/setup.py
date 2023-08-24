import os
from glob import glob
from setuptools import setup

package_name = 'ark_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz')),
        # (os.path.join('share', package_name), glob('description/'))
        (os.path.join('share', package_name, 'description'), glob('description/*'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='Braden',
    maintainer_email='braden@arkelectron.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'rplidar_checker = ark_rover.rplidar_checker:main',
                'rplidar_composition = rplidar_ros.rplidar_composition:main',
                'stream_compress = ark_rover.stream_compress:main',
                'battery_node = ark_rover.battery_node:main',
                'marker_node = ark_rover.marker_node:main',
                'image_publisher = ark_rover.image_publisher:main',
                'battery_publisher = ark_rover.battery_publisher:main',
                'offboard_control = ark_rover.offboard_control:main',
                'visualizer = ark_rover.visualizer:main',
                'velocity_control = ark_rover.velocity_control:main',
                'control = ark_rover.control:main',
                'processes = ark_rover.processes:main',
                'rover_control = ark_rover.rover_control:main',
                'joystick = ark_rover.joystick:main'
        ],
    },
)
