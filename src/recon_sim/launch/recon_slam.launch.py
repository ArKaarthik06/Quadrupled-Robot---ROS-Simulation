import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('recon_sim')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'recon_gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'recon_office.world'),
            'gui': LaunchConfiguration('gui'),
            'demo': 'false',
            'period_ms': LaunchConfiguration('period_ms'),
        }.items())

    odom = Node(package='recon_sim', executable='recon_odom.py', name='recon_odom', output='screen',
                parameters=[{'use_sim_time': True, 'noise': LaunchConfiguration('odom_noise')}])

    slam = Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                output='screen',
                parameters=[os.path.join(pkg, 'config', 'slam_params.yaml'), {'use_sim_time': True}])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', os.path.join(pkg, 'config', 'recon_slam.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz window (map, scan, camera)'),
        DeclareLaunchArgument('period_ms', default_value='500.0'),
        DeclareLaunchArgument('odom_noise', default_value='0.0',
                              description='drift of the odometry, e.g. 0.03 = 3 percent'),
        sim, odom, slam, rviz,
    ])
