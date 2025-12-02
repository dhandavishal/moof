from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'task_execution'

setup(
    name=package_name,  # Must be 'task_execution' (underscore, not hyphen)
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MOOFS Team',
    maintainer_email='moofs@example.com',
    description='Task Execution Engine for MOOFS multi-drone control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tee_node = task_execution.tee_node:main',
            'performance_monitor = task_execution.performance_monitor:main',
        ],
    },
)