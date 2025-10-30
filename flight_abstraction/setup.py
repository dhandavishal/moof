from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'flight_abstraction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhandavishal',
    maintainer_email='dhandavishal@yahoo.co.in',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fal_node = flight_abstraction.fal_node:main',
            'primitive_command_handler = flight_abstraction.primitive_command_handler:main',
            'test_single_drone = flight_abstraction.test_single_drone:main',
        ],
    },
)
