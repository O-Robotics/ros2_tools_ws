from setuptools import setup, find_packages

package_name = 'covariance_calculator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/analyze_covariance.launch.py']),
        ('share/' + package_name + '/config', ['config/analysis_config.yaml']),
        ('share/' + package_name + '/docs', ['docs/covariance_formulas.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='O-Robotics Developer',
    maintainer_email='dev@o-robotics.com',
    description='ROS2 package for calculating IMU and Odometry measurement noise covariance matrices from bag data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'covariance_calculator_node = covariance_calculator.covariance_calculator_node:main',
            'batch_analyze = covariance_calculator.batch_analyze:main',
        ],
    },
)
