import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import launch_ros

def generate_launch_description():
    ld = LaunchDescription()
    
    # Paths to the launch files

    
    ros2_aruco_launch = os.path.join(
        get_package_share_directory('ros2_aruco'), 'launch', 'aruco_auto.launch.py')
    autonomy_pkg_launch = os.path.join(
        get_package_share_directory('autonomy_pkg'), 'launch', 'autonomy.launch.py')

    #Hardware launch files
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource('launches/heimdall/heimdall_static_tf.launch.py')
    ))
    
    ld.add_action(
        Node(
            name='motor_protection_node',
            package='motor_protection',
            executable='timeout_check_drive',
        )
    )


    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource('launches/heimdall/heimdall_autonomy.launch.py')
    # ))


    # launch the autonomy status LED node
    ld.add_action(
            launch_ros.actions.Node(
            package='autonomy_led_pkg',
            executable='autonomy_led_subscriber',
            name='autonomy_led_subscriber')
        )

    # Software launch files
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_aruco_launch)
    ))
    
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'run', 'autonomy_pkg', 'object_chaser_node'],
        output='screen'
    ))

    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'run', 'autonomy_pkg', 'waypoint_queue_node'],
        output='screen'
    ))

    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'run', 'autonomy_pkg', 'autonomy_ptp_planner_node'],
        output='screen'
    ))


    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(autonomy_pkg_launch)
    # ))
    

    return ld

def get_package_share_directory(package_name):
    """Helper function to get the package share directory."""
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package_name)
