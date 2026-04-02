from setuptools import find_packages, setup

package_name = 'flood_boat_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/flood_mission.launch.py']),
        ('share/' + package_name + '/params', ['params/mission.yaml']),
        ('share/' + package_name + '/worlds', ['worlds/flood_city.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='project-owner',
    maintainer_email='you@example.com',
    description='Starter ROS 2 package for autonomous flood-response boat simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = flood_boat_sim.mission_node:main',
        ],
    },
)
