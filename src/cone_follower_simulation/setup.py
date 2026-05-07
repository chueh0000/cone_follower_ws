from setuptools import find_packages, setup

package_name = 'cone_follower_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fsds_simulation.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/fsds_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='howardchueh',
    maintainer_email='howardchueh@todo.todo',
    description='Simulation package for generating mock track data.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_track_publisher = cone_follower_simulation.mock_track_publisher:main',
            'cone_visualization_node = cone_follower_simulation.cone_visualization_node:main',
            'fsds_track_bridge = cone_follower_simulation.fsds_track_bridge:main'
        ],
    },
)
