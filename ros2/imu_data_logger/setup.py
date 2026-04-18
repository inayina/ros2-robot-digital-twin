from setuptools import find_packages, setup

package_name = 'imu_data_logger'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ina',
    maintainer_email='yinawl107@gmail.com',
    description='ROS 2 IMU telemetry logger and live plot analysis tools.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_logger = imu_data_logger.logger_node:main',
            'imu_live_plot = imu_data_logger.live_plot_node:main',
        ],
    },
)
