from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'bunker_navigation'
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
            'path_perimeter_node = bunker_navigation.path_perimeter_node:main',
            'path_generator_node = bunker_navigation.path_generator_node:main',
            'stanlynavigation_node = bunker_navigation.stanlynavigation_node:main',
            'dodging_node = bunker_navigation.dodging_node:main',
            'cargo_path_perimeter_node = bunker_navigation.cargo_path_perimeter_node:main',
            'cargohauling_navigation_node = bunker_navigation.cargohauling_navigation_node:main',
        ],
    },
)
