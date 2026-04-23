from setuptools import find_packages, setup

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'mock_track_publisher = simulation.mock_track_publisher:main'
        ],
    },
)
