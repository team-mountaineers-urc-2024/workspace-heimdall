from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, RosTimer
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import TextSubstitution

def generate_launch_description():

    # Create the Launch Description
    launch_description = LaunchDescription()

    # Declare Each Argument
    localization_arg    = DeclareLaunchArgument(name = 'doLocalization',    default_value = 'true',     description = 'Are you running the rover with teleop? [true, false]')
    heartbeat_arg       = DeclareLaunchArgument(name = 'doHeartbeat',       default_value = 'false',    description = 'Are you running the rover with the heartbeat node? [true, false]')
    px4_config_arg      = DeclareLaunchArgument(name = 'px4_config_file',   default_value = [
                                                                            TextSubstitution(text=os.path.join('launches/config', '')),
                                                                            TextSubstitution(text='urc_px4_config'),
                                                                            TextSubstitution(text='.yaml')
                                                                            ],                          description = 'What is the directory to the pixhawk config file?')

    # Add each argument
    launch_description.add_action(localization_arg)
    launch_description.add_action(heartbeat_arg)
    launch_description.add_action(px4_config_arg)


    # Get the important values
    doHeartbeat     = LaunchConfiguration('doHeartbeat')
    px4Config       = LaunchConfiguration('px4_config_file')

    # Heimdall Drive
    heimdall_drive = \
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                'launches/heimdall/heimdall_teleop.launch.py'
            ),
            launch_arguments={
                'doHeartbeat': doHeartbeat,
                'doCan': 'true',
                'doJoy': 'false'
            }.items(),
        )
    
    safety_node = \
        Node(
            name='motor_protection_node',
            package='motor_protection',
            executable='timeout_check_es',
        )

    # Heimdall Arm Launch
    heimdall_arm = \
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                'launches/heimdall/heimdall_arm.launch.py'
            ),
            launch_arguments={
                'doHeartbeat': doHeartbeat,
                'doCan': 'false',
                'doJoy': 'false'
            }.items(),
        )
    
    # Pixhawk Localization
    heimdall_localization = \
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource('launches/components/urc_px4.launch'),
            launch_arguments={
                'fcu_url': '/dev/urc/mtc/pixhawk',
                'config_yaml': px4Config
            }.items(),
            condition=LaunchConfigurationEquals('doLocalization', 'true'),
        )

    # Set up timers for each additional launch
    heimdall_drive_d        = RosTimer(period = 0.0, actions = [heimdall_drive, safety_node])
    heimdall_arm_d          = RosTimer(period = 5.0, actions = [heimdall_arm])
    heimdall_localization_d = RosTimer(period = 10.0, actions = [heimdall_localization])


    # Launch each individual thing
    launch_description.add_action(heimdall_drive_d)
    launch_description.add_action(heimdall_arm_d)
    launch_description.add_action(heimdall_localization_d)

    return launch_description
