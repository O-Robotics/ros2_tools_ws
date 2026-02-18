from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'localization_calib'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/config.yaml',
            'config/topics.yaml',
        ]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='liziye725@gmail.com',
    description='ROS2 package for calibrating sensor covariances for localization systems (IMU and Odometry)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_calib = localization_calib.record_calib:main',
            'calculate_cov = localization_calib.calculate_cov:main',
        ],
    },
)
