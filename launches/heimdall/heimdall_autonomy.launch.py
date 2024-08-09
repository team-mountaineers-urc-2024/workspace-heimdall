import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_ros

def generate_launch_description():

    ld = LaunchDescription()

    # Launch the minimal autonomy interface
    # ld.add_action(
    #         launch_ros.actions.Node(
    #         package='ros2_pid_planner_pkg',
    #         executable='go_to_point_node',
    #         name='go_to_point_node',
    #         parameters=[
    #             {'is_active': True},
    #         ]
    #         )
    #     )
    
    # # Launch the path manager for passing GUI inpit to the autonomy interface
    # ld.add_action(
    #         launch_ros.actions.Node(
    #         package='ros2_pid_planner_pkg',
    #         executable='path_manager_node',
    #         name='path_manager_node',
    #         )
    #     )
    
    # Launch the PID planner
    # ld.add_action(
    #         launch_ros.actions.Node(
    #         package='ros2_pid_planner_pkg',
    #         executable='pid_planner_node',
    #         name='pid_planner_node',
    #         parameters=[
    #           {'angle_threshold': 10.0},
	# 	        {'dist_prop_gain': 0.05},
	# 	        {'theta_prop_gain': 0.2},
	# 	        {'dist_kp': 0.01},
	# 	        {'head_kp': 1.0},
    #         ]
    #         )
    #     )

    # # Do Transform
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             'launches/heimdall/heimdall_static_tf.launch.py'
    #         )
    #     )
    # )

    # # Launch Cameras
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             'launches/heimdall/heimdall_usb_cam.launch.py'
    #         )
    #     )
    # )

    # # Launch Aruco
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             'launches/heimdall/heimdall_aruco.launch.py'
    #         )
    #     )
    # )

    # Launch MAVROS for the Pixhawk
    ld.add_action(
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource('./launches/components/urc_px4.launch'),
            launch_arguments={
                'fcu_url': '/dev/urc/mtc/pixhawk',
                'config_yaml': './launches/config/urc_px4_config.yaml',
            }.items()
        )
    )

    # # launch the pixhawk localization node
    # ld.add_action(
    #         launch_ros.actions.Node(
    #         package='ros2_pid_planner_pkg',
    #         executable='pixhawk_node',
    #         name='pixhawk_node')
    #     )

    # # Launch the drivebase node
    # ld.add_action(
    #      IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             PathJoinSubstitution([
    #                 FindPackageShare('drivebase_control_pkg'),
    #                 'launch',
    #                 'drivebase_control_node.launch.py'
    #             ])
    #         ),
    #         launch_arguments={
    #             'track_width': '0.914',
    #             'wheel_radius': '0.2540',
    #             'isHeimdall': 'True'
    #         }.items()
    #     )
    # )

    # # Launch the can interface
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             'launches/components/can_interface.launch.py'
    #             )
    #     )
    # )

    # # launch the autonomy status LED node
    # ld.add_action(
    #         launch_ros.actions.Node(
    #         package='autonomy_led_pkg',
    #         executable='autonomy_led_subscriber',
    #         name='autonomy_led_subscriber')
    #     )


    return ld
