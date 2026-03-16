from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('agv_description'))
    nav2_bringup_share = Path(get_package_share_directory('nav2_bringup'))
    ros_gz_sim_share = Path(get_package_share_directory('ros_gz_sim'))

    world_file = package_share / 'worlds' / 'agv_room.world'
    bridge_config = package_share / 'params' / 'agv_bridge.yaml'
    nav2_params_file = package_share / 'params' / 'agv_nav2_params.yaml'
    urdf_file = package_share / 'urdf' / 'agv_robot.urdf'
    rviz_config_file = package_share / 'rviz' / 'agv_display.rviz'

    robot_description = urdf_file.read_text()
    use_rviz = LaunchConfiguration('use_rviz')
    use_nav2 = LaunchConfiguration('use_nav2')

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_sim_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_sim_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ],
    )

    spawn_agv = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'agv',
            '-string', robot_description,
            '-x', '-3.8',
            '-y', '0.0',
            '-z', '0.05',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}',
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_file)],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_bringup_share / 'launch' / 'bringup_launch.py')),
        launch_arguments={
            'slam': 'True',
            'use_localization': 'True',
            'use_sim_time': 'True',
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
            'params_file': str(nav2_params_file),
        }.items(),
        condition=IfCondition(use_nav2),
    )

    nav_goal_bridge = Node(
        package='agv_test_pkg',
        executable='agv_nav_goal_bridge',
        name='agv_nav_goal_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_nav2),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz alongside Gazebo using the simulated robot state.',
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Launch SLAM Toolbox, Nav2, and the RViz goal bridge.',
        ),
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_agv,
        bridge,
        rviz,
        nav2_bringup,
        nav_goal_bridge,
    ])
