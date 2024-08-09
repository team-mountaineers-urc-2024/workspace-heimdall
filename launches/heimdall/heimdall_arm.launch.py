from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.substitutions import LaunchConfiguration
import os 
from ament_index_python.packages import get_package_share_directory


config_dir = os.path.join(get_package_share_directory('manipulator_control_pkg'),('config'))

config = os.path.join(config_dir, 'green_controller_manipulator_control.yaml')


def generate_launch_description():
    launch_description = LaunchDescription()

    # Name of the Green Controller --> Look into changing this name
    arm_name = '8BitDo Ultimate C 2.4G Wireless Controller'

    # Declare Each Argument
    launch_level    = DeclareLaunchArgument(name = 'launch_level',      default_value = '',         description = 'Level at which to display the launch logs')
    heartbeat_arg   = DeclareLaunchArgument(name = 'doHeartbeat',       default_value = 'false',    description = 'Are you running teleop with the heartbeat node? [true, false]')
    joy_arg         = DeclareLaunchArgument(name = 'doJoy',             default_value = 'true',     description = 'Are you running the teleop with the joy control? [true, false]')
    can_arg         = DeclareLaunchArgument(name = 'doCan',             default_value = 'true',     description = 'Should Teleop launch the can interface node? [true, false]')


    # Add each argument
    launch_description.add_action(heartbeat_arg)
    launch_description.add_action(joy_arg)
    launch_description.add_action(can_arg)
    launch_description.add_action(launch_level)

    # Get the important values
    arm_log_depth       = LaunchConfiguration('launch_level')

    # If we want it, launch the heartbeat reciever
    heartbeat_reciever = \
        Node(
            condition=LaunchConfigurationEquals('doHeartbeat', 'true'),
            name='heartbeat_reciever',
            package='comms_heartbeat_pkg',
            executable='heartbeat_reciever',
        )

    # If we want it, launch the can interface
    can_interface = \
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                'launches/components/can_interface.launch.py'
            ),
            condition=LaunchConfigurationEquals('doCan', 'true'),

        )

    # If we want it, launch the game controller
    joy_node = \
        Node(
            condition=LaunchConfigurationEquals('doJoy', 'true'),
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

    # Launch the controller decypherer
    operator_control_node = \
        Node(
            namespace='manipulator',
            package='manipulator_control_pkg',
            executable='operator_controls',
            name='manipulator_operator_controls_node',
            parameters = [config],
            remappings=[
                ('/manipulator/pico_sub', '/pico_sub')
            ]
        )

    # Launch the non-uart motors
    can_motor_node = \
        Node(
            namespace='manipulator',
            package='manipulator_control_pkg',
            executable='manipulator_motors_can_node',
            name='manipulator_motors_can_node',
            parameters = [
                {'do_heartbeat': LaunchConfiguration('doHeartbeat')},
                config
                ],
            remappings=[
                ('/manipulator/outgoing_can_commands', '/outgoing_can_commands'),
            ]

        )

    # Logging Outputs
    heartbeat_true = \
        LogInfo(
            condition=LaunchConfigurationEquals('doHeartbeat', 'true'),
            msg=['[heimdall_arm.launch.py]\t\t', arm_log_depth, '\033[34m> Launching the Heartbeat Node\033[0m']
        )

    heartbeat_false = \
        LogInfo(condition=LaunchConfigurationEquals('doHeartbeat', 'false'),
        msg=['[heimdall_arm.launch.py]\t\t', arm_log_depth, '\033[93m> Not Launching the Heartbeat Node\033[0m']
        )
    
    can_true = \
        LogInfo(condition=LaunchConfigurationEquals('doCan', 'true'),
        msg=['[heimdall_arm.launch.py]\t\t', arm_log_depth, '\033[34m> Launching the CAN Interface Node\033[0m']
        )

    can_false = \
        LogInfo(condition=LaunchConfigurationEquals('doCan', 'false'),
        msg=['[heimdall_arm.launch.py]\t\t', arm_log_depth, '\033[93m> Not Launching the CAN Interface Node\033[0m']
        )
    
    joy_true = \
        LogInfo(condition=LaunchConfigurationEquals('doJoy', 'true'),
        msg=['[heimdall_arm.launch.py]\t\t', arm_log_depth, '\033[34m> Launching the Joystick Node\033[0m']
        )

    joy_false = \
        LogInfo(condition=LaunchConfigurationEquals('doJoy', 'false'),
        msg=['[heimdall_arm.launch.py]\t\t', arm_log_depth, '\033[93m> Not Launching the Joystick Node\033[0m']
        )
    
    operator_true = \
        LogInfo(
        msg=['[heimdall_arm.launch.py]\t\t', arm_log_depth, '\033[34m> Launching the Operator Control Node\033[0m']
        )
    
    can_motors = \
        LogInfo(
        msg=['[heimdall_arm.launch.py]\t\t', arm_log_depth, '\033[34m> Launching the CAN motor Node\033[0m']
        )
    
    controller = \
        LogInfo(
            msg=['[heimdall_teleop.launch.py]\t', arm_log_depth, '\033[34m> Controller #' + arm_name +'\033[0m']
        )

    # Launch each individual thing
    launch_description.add_action(controller)
    launch_description.add_action(heartbeat_true)
    launch_description.add_action(heartbeat_false)
    launch_description.add_action(heartbeat_reciever)
    launch_description.add_action(can_true)
    launch_description.add_action(can_false)
    launch_description.add_action(can_interface)
    launch_description.add_action(joy_true)
    launch_description.add_action(joy_false)
    launch_description.add_action(joy_node)
    launch_description.add_action(operator_true)
    launch_description.add_action(operator_control_node)
    launch_description.add_action(can_motors)
    launch_description.add_action(can_motor_node)

    return launch_description
