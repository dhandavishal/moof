from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'squadron_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Include test scripts
        (os.path.join('share', package_name, 'test'),
            glob('test/*.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
    ],
    zip_safe=True,
    maintainer='dhandavishal',
    maintainer_email='dhandavishal@yahoo.co.in',
    description='Squadron Manager - Multi-drone coordination and task allocation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'squadron_manager_node = squadron_manager.squadron_manager_node:main',
            'test_squadron = squadron_manager.test_squadron:main',
            'test_moofs_survey_mission = squadron_manager.test_moofs_survey_mission:main',
        ],
    },
)