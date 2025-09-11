from setuptools import setup
import os
from glob import glob

package_name = 'as2_ardu_msn'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ArduPilot Survey Mission for Aerostack2',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'survey_mission = as2_ardu_msn.survey_mission:main',
            'simple_mission = scripts.simple_mission:main',
            'clear_mission = scripts.clear_mission:main',
        ],
    },
)