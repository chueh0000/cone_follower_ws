from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cone_follower_vehicle_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['*.so'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='howardchueh',
    maintainer_email='howardchueh@todo.todo',
    description='Vehicle interface package for low-level control communication.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_interface_node = cone_follower_vehicle_interface.vehicle_interface_node:main'
        ],
    },
)
