import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch.actions import ExecuteProcess, LogInfo, DeclareLaunchArgument
from ament_index_python import get_package_share_directory
from launch.event_handlers import OnShutdown
from time import sleep
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import TextSubstitution, LaunchConfiguration

def generate_launch_description():
    launch_description = LaunchDescription()

    # Current Rover Argument
    launch_description.add_action(
        DeclareLaunchArgument(name = 'current_rover', default_value = 'heimdall', description = 'What Rover are you running?')
    )

    launch_description.add_action(
        DeclareLaunchArgument(name = 'launch_level', default_value = '', description = 'Level at which to display the launch logs')
    )

    can_interface_log_depth   = LaunchConfiguration('launch_level')

    launch_description.add_action(
        LogInfo(condition=LaunchConfigurationEquals('current_rover', 'heimdall'),
        msg=['[can_interface.launch.py]\t\t', can_interface_log_depth,'\033[34m> Launching CAN as \033[93mheimdall\033[0m']
        )
    )

    # If it is heimdall
    launch_description.add_action(
        ExecuteProcess(
            condition=LaunchConfigurationEquals('current_rover', 'heimdall'),
            cmd=[["echo heimdall | sudo -S /sbin/ip link set can0 up type can bitrate 1000000"]],
            shell=True,
            output='screen'
        )
    )
    
    launch_description.add_action(
        LogInfo(condition=LaunchConfigurationEquals('current_rover', 'wanderer2'),
        msg=['[can_interface.launch.py]\t\t', can_interface_log_depth,'\033[34m> Launching CAN as wanderer2\033[0m']
        )
    )

    # If it is wanderer2
    launch_description.add_action(
        ExecuteProcess(
            condition=LaunchConfigurationEquals('current_rover', 'wanderer2'),
            cmd=[["echo wanderer2 | sudo -S /sbin/ip link set can0 up type can bitrate 1000000"]],
            shell=True,
            output='screen'
        )
    )

    launch_description.add_action(
        LogInfo(msg=['[can_interface.launch.py]\t\t', can_interface_log_depth,'\033[34m> Launching CAN Subscriber Node\033[0m']
        )
    )

    launch_description.add_action(
            launch_ros.actions.Node(
            package='can_interface_pkg',
            executable='can_subscriber_node',
            name='can_subscriber_node')
        )
 
    return launch_description
