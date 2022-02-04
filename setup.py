from setuptools import setup
import os
from glob import glob
package_name = 'grpc_ros_adapter'
submodules = [f'{package_name}/protobuf', f'{package_name}/services', f'{package_name}/utils']
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, *submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*_launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mandicluka',
    maintainer_email='luka.mandic311@gmail.com',
    description='Bridges and translates messages between ROS and Unity using gRPC',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'server = {package_name}.server:main',
        ],
    },
)

