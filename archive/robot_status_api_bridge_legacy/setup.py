from setuptools import find_packages, setup


package_name = 'robot_status_api_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt',
    ],
    zip_safe=True,
    maintainer='ina',
    maintainer_email='ina@example.com',
    description='ROS 2 to dashboard backend robot status bridge.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_status_bridge = '
            'robot_status_api_bridge.status_bridge_node:main',
        ],
    },
)
