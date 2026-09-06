from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'bunker_hardware'
package_root = os.path.dirname(__file__)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob(os.path.join(package_root, 'config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bidya',
    maintainer_email='bidhyashree.dahal@dcmail.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_node = bunker_hardware.gps_node:main',
            'IMU_node = bunker_hardware.IMU_node:main',
            'can_feedback_node = bunker_hardware.can_feedback_node:main',
            'robot_controller_node = bunker_hardware.robot_controller_node:main',
            'realsense_publisher_node = bunker_hardware.realsense_publisher_node:main',
            'realsense_right_publisher_node = bunker_hardware.realsense_right_publisher_node:main',
            'realsense_left_publisher_node = bunker_hardware.realsense_left_publisher_node:main',
        ],
    },
)