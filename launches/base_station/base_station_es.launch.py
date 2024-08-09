import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.actions import ExecuteProcess, LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


config_dir = os.path.join(get_package_share_directory('manipulator_control_pkg'),('config'))

config = os.path.join(config_dir, 'green_controller_manipulator_control.yaml')

def generate_launch_description():
    launch_description = LaunchDescription()

    # Name of the Purple Controller --> Look into changing this name
    drive_name = '8BitDo Pro 2 Wired Controller'
    arm_name = '8BitDo Ultimate C 2.4G Wireless Controller'


    # Declare Each Argument
    heartbeat_arg   = DeclareLaunchArgument(name = 'doHeartbeat',       default_value = 'false',    description = 'Are you running teleop with the heartbeat node? [true, false]')
    vel_arg         = DeclareLaunchArgument(name = 'joy_vel',           default_value = 'cmd_vel',  description = 'The output topic for the command velocity [cmd_vel]')
    joy_config_arg  = DeclareLaunchArgument(name = 'joy_config',        default_value = '8bit-p',   description = 'The config file name for the controller [8bit-p, 8bit-g]')
    config_path_arg = DeclareLaunchArgument(name = 'config_filepath',   default_value = [
                                                                        TextSubstitution(text=os.path.join('launches/config', '')),
                                                                        LaunchConfiguration('joy_config'),
                                                                        TextSubstitution(text='.config.yaml')
                                                                        ],                          description='The direct filepath to the controller config file')

    # Add each argument
    launch_description.add_action(heartbeat_arg)
    launch_description.add_action(vel_arg)
    launch_description.add_action(joy_config_arg)
    launch_description.add_action(config_path_arg)

    # Get the important values
    config_filepath     = LaunchConfiguration('config_filepath')
    joy_vel             = LaunchConfiguration('joy_vel')

    # The Heartbeat Node
    heartbeat_transmitter = \
        Node(
            condition=LaunchConfigurationEquals('doHeartbeat', 'true'),
            name='heartbeat_transmitter',
            package='comms_heartbeat_pkg',
            executable='heartbeat_transmitter',
        )
    
    # Joy Control
    drive_joy_node = \
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[
                {
                'device_name' : drive_name,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
                }
            ]
        )
    
    # Arm Control
    arm_joy_node = \
        Node(
            namespace='manipulator',
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                {
                'device_name' : arm_name,
                }
            ]
        )
    
    # Teleop Joy Decypherer
    drive_teleop_node = \
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath],
            remappings={('/cmd_vel', joy_vel)},
        )

    launch_description.add_action(heartbeat_transmitter)
    launch_description.add_action(drive_joy_node)
    launch_description.add_action(drive_teleop_node)
    launch_description.add_action(arm_joy_node)

    return launch_description