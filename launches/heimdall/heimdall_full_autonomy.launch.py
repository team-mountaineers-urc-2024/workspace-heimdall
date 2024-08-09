import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node, RosTimer
from launch_ros.substitutions import FindPackageShare
import launch_ros
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    ld = LaunchDescription()

    # Launch MAVROS for the Pixhawk
    ld.add_action(
        RosTimer(
            period = 5.0,
            actions = [
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource('./launches/components/urc_px4.launch'),
                    launch_arguments={
                        'fcu_url': '/dev/urc/mtc/pixhawk',
                        'config_yaml': './launches/config/urc_px4_config.yaml',
                    }.items()
                )
            ]
        )
    )

    #Hardware launch files
    ld.add_action(
        RosTimer(
            period = 10.0,
            actions = [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource('launches/heimdall/heimdall_static_tf.launch.py')
                )
            ]
        )
    )
    
    ld.add_action(
        RosTimer(
            period = 15.0,
            actions = [
                Node(
                    name='motor_protection_node',
                    package='motor_protection',
                    executable='timeout_check_drive',
                )
            ]
        )
    )

    # launch the autonomy status LED node
    ld.add_action(
        RosTimer(
            period = 15.0,
            actions = [
                Node(
                    package='autonomy_led_pkg',
                    executable='autonomy_led_subscriber',
                    name='autonomy_led_subscriber')
            ]
        )
    )

    # Software launch files
    ld.add_action(
        RosTimer(
            period = 15.0,
            actions = [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                                FindPackageShare('ros2_aruco'),
                                'launch',
                                'aruco_auto.launch.py'
                            ])
                    )
                )
            ]
        )
    )
    
    ld.add_action(
        RosTimer(
            period = 20.0,
            actions = [
                ExecuteProcess(
                    cmd=['ros2', 'run', 'autonomy_pkg', 'object_chaser_node'],
                    output='log'
                )
            ]
        )
    )

    ld.add_action(
        RosTimer(
            period = 20.0,
            actions = [
                ExecuteProcess(
                    cmd=['ros2', 'run', 'autonomy_pkg', 'waypoint_queue_node'],
                    output='log'
                )
            ]
        )
    )

    ld.add_action(
        RosTimer(
            period = 25.0,
            actions = [
                ExecuteProcess(
                    cmd=['ros2', 'run', 'autonomy_pkg', 'autonomy_ptp_planner_node'],
                    output='log'
                )
            ]
        )
    )

    # launch autonomy cams
    ld.add_action(
        RosTimer(
            period = 25.0,
            actions = [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        'launches/heimdall/heimdall_autonomy_cams.launch.py'
                    ),
                )
            ]
        )
    )

    # launch driving capability
    ld.add_action(
        RosTimer(
            period = 0.0,
            actions = [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        'launches/heimdall/heimdall_teleop.launch.py'
                    ),
                    launch_arguments={
                        'doHeartbeat': 'false',
                        'doCan': 'true',
                        'doJoy': 'false'
                    }.items(),
                )
            ]
        )
    )

    return ld
