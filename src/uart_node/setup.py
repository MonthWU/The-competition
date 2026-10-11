from setuptools import setup, find_packages

package_name = 'uart_node'

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
    description='UART sender node - 200Hz real-time, 24B protocol',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_sender = uart_node.node:main',
            'uart_node = uart_node.node:main',
        ],
    },
)
