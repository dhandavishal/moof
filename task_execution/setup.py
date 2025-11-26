from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_execution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dhandavishal',
    maintainer_email='dhandavishal@yahoo.co.in',
    description='Task Execution Engine for multi-drone coordination',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'monitor_demo = task_execution.monitor_demo:main',
            'tee_node = task_execution.tee_node:main',
            'test_mission_execution = scripts.test_mission_execution:main',
            'monitor_tee_status = scripts.monitor_tee_status:main',
            'performance_monitor = task_execution.performance_monitor:main',
        ],
    },
)
