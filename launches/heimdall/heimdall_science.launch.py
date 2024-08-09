from launch import LaunchDescription
from launch_ros.actions import Node, RosTimer
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os

def generate_launch_description():

    px4Config       = LaunchConfiguration('px4_config_file')

    launch_description = LaunchDescription()

    # Launches the spectrometer
    spectrometer = \
        Node(
            package='science_control_pkg',
            executable='ocean_optics_spectrometer_node',
            name='ocen_optics_spectrometer_node'
            )

    # Launches the Dynamixels
    dynamixels = \
        Node(
            package='science_control_pkg',
            executable='dynamixel_control_node',
            name='dynamixel_control_node'
            )

    # Launches the MyActuator motor used for the core drill
    core_drill = \
        Node(
            package='science_control_pkg',
            executable='subsurface_motor_node',
            name='subsurface_motor_node'
            )

    safety_node = \
        Node(
            name='motor_protection_node',
            package='motor_protection',
            executable='timeout_check_drive',
        )

    # Launch the drive interface
    drive = \
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                'launches/heimdall/heimdall_teleop.launch.py'
            ),
            launch_arguments={
                'doHeartbeat' : 'false',
                'doJoy' : 'false'
            }.items()
        )
    
    # Launch the pixhawk
    pixhawk = \
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource('launches/components/urc_px4.launch'),
            launch_arguments={
                'fcu_url': '/dev/urc/mtc/pixhawk',
                'config_yaml': px4Config
            }.items(),
        ) 
    
    # Set up timers for each additional launch
    drive_d         = RosTimer(period = 0.0, actions = [drive, safety_node])
    core_drill_d    = RosTimer(period = 5.0, actions = [core_drill])
    dynamixels_d    = RosTimer(period = 10.0, actions = [dynamixels])
    spectrometer_d  = RosTimer(period = 15.0, actions = [spectrometer])
    pixhawk_d       = RosTimer(period = 20.0, action = [pixhawk])


    # Launch each individual thing
    launch_description.add_action(drive_d)
    launch_description.add_action(core_drill_d)
    launch_description.add_action(dynamixels_d)
    launch_description.add_action(spectrometer_d)
    launch_description.add_action(pixhawk_d)

    return launch_description
