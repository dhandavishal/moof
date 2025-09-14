from setuptools import setup
import os
from glob import glob

package_name = 'as2_ardu_msn'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],  # No Python packages since we use scripts
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
        # Install scripts
        (os.path.join('share', package_name, 'scripts'), 
            glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vishal Dhanda',
    maintainer_email='dhandavishal@example.com',
    description='Modular ArduPilot Survey Mission Package for Aerostack2',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Scripts are run directly, no console entry points needed
        ],
    },
)