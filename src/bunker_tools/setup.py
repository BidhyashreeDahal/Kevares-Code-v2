from setuptools import find_packages, setup

package_name = 'bunker_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'discord_zigzag_test_node = bunker_tools.discord_zigzag_test_node:main',
        ],
    },
)
