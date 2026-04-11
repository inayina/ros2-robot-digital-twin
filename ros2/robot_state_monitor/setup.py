from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_state_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml', 'README.md', 'Debug.md', 'setup.cfg']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*.sdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ina',
    maintainer_email='ina@example.com',
    description='Robot State Monitor package for ROS 2 with Gazebo integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_bridge = robot_state_monitor.robot_gazebo_bridge:main',
        ],
    },
)
