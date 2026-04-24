from setuptools import find_packages, setup
package_name = 'cone_follower_planning'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='howardchueh',
    maintainer_email='howardchueh@todo.todo',
    description='Path planning package for centerline generation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
