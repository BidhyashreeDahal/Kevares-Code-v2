from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'bunker_perception'
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
            'object_detection_node = bunker_perception.object_detection_node:main',
            'collision_avoidance_node = bunker_perception.collision_avoidance_node:main',
        ],
    },
)
