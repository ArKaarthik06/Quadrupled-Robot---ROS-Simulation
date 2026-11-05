import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('recon_sim')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    gui = LaunchConfiguration('gui')
    demo = LaunchConfiguration('demo')
    world = LaunchConfiguration('world')

    robot_description = xacro.process_file(os.path.join(pkg, 'urdf', 'recon.urdf.xacro')).toxml()
    robot_description = re.sub(r'<!--.*?-->', '', robot_description, flags=re.S)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': LaunchConfiguration('verbose'),
                          'params_file': os.path.join(pkg, 'config', 'gazebo_params.yaml')}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description', '-entity', 'recon',
                            '-x', '0', '-y', '0', '-z', '0.06'],
                 output='screen')

    gait = Node(package='recon_sim', executable='recon_gait.py', name='recon_gait',
                output='screen',
                parameters=[{'use_sim_time': True, 'demo': demo, 'period_ms': LaunchConfiguration('period_ms')}])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='open the Gazebo client window'),
        DeclareLaunchArgument('verbose', default_value='false', description='verbose gzserver output'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='run the built-in walk/turn/sit demo sequence'),
        DeclareLaunchArgument('period_ms', default_value='500.0',
                              description='gait period in ms (Speed in the firmware)'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'recon_arena.world')),
        gzserver, gzclient, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[gait])),
    ])
