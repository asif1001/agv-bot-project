from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_params = LaunchConfiguration('lidar_params')
    dashboard_host = LaunchConfiguration('dashboard_host')
    dashboard_port = LaunchConfiguration('dashboard_port')

    ydlidar_launch = PathJoinSubstitution([
        FindPackageShare('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py',
    ])
    default_params = PathJoinSubstitution([
        FindPackageShare('ydlidar_ros2_driver'),
        'params',
        'X3.yaml',
    ])

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch),
        launch_arguments={
            'params_file': lidar_params,
            'port': lidar_port,
        }.items(),
    )

    dashboard = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'agv_test_pkg', 'odrive_dashboard',
        ],
        additional_env={
            'ODRIVE_DASHBOARD_HOST': dashboard_host,
            'ODRIVE_DASHBOARD_PORT': dashboard_port,
        },
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0', description='Serial port for the YDLIDAR device.'),
        DeclareLaunchArgument('lidar_params', default_value=default_params, description='Path to the active YDLIDAR parameter file.'),
        DeclareLaunchArgument('dashboard_host', default_value='0.0.0.0', description='Host interface for the ODrive dashboard HTTP server.'),
        DeclareLaunchArgument('dashboard_port', default_value='8081', description='Port for the ODrive dashboard HTTP server.'),
        lidar,
        dashboard,
    ])
