from setuptools import setup

package_name = 'iot_c2d_receiver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/c2d_receiver.launch.py']),
        ('share/' + package_name + '/config', ['config/default.yaml']),
    ],
    install_requires=['setuptools', 'azure-iot-device', 'python-dotenv'],
    zip_safe=True,
    author='ZiyeLi',
    author_email='ziyli@orobotics.com',
    maintainer='ORobotics',
    maintainer_email='ziyli@example.com',
    description='ROS 2 node that receives Azure IoT Hub C2D messages and stores waypoint YAMLs.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'c2d_receiver = iot_c2d_receiver.c2d_receiver_node:main',
        ],
    },
)
