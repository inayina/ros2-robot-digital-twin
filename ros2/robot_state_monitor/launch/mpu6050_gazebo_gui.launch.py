import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('robot_state_monitor')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    default_world = os.path.join(package_share, 'worlds', 'empty.sdf')

    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    imu_topic = LaunchConfiguration('imu_topic')
    model_name = LaunchConfiguration('model_name')
    update_rate = LaunchConfiguration('update_rate')
    use_imu_estimator = LaunchConfiguration('use_imu_estimator')
    accel_correction = LaunchConfiguration('accel_correction')
    gyro_deadband = LaunchConfiguration('gyro_deadband')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 --render-engine-gui ogre ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_service_bridge',
        output='screen',
        arguments=[
            ['/world/', world_name, '/create@ros_gz_interfaces/srv/SpawnEntity'],
            ['/world/', world_name, '/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        ],
    )

    gazebo_bridge = Node(
        package='robot_state_monitor',
        executable='gazebo_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{
            'world_name': world_name,
            'imu_topic': imu_topic,
            'model_name': model_name,
            'update_rate': ParameterValue(update_rate, value_type=float),
            'use_imu_estimator': ParameterValue(
                use_imu_estimator,
                value_type=bool
            ),
            'accel_correction': ParameterValue(
                accel_correction,
                value_type=float
            ),
            'gyro_deadband': ParameterValue(gyro_deadband, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('world_name', default_value='default'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/filtered'),
        DeclareLaunchArgument('model_name', default_value='mpu6050'),
        DeclareLaunchArgument('update_rate', default_value='15.0'),
        DeclareLaunchArgument('use_imu_estimator', default_value='true'),
        DeclareLaunchArgument('accel_correction', default_value='0.04'),
        DeclareLaunchArgument('gyro_deadband', default_value='0.015'),
        gazebo,
        service_bridge,
        gazebo_bridge,
    ])
