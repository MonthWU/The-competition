from setuptools import setup, find_packages
import os
from glob import glob
package_name = 'imu_node'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@d-robotics.cc',
    description='Optimized ROS2 node package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = imu_node.node:main',
        ],
    },
)
