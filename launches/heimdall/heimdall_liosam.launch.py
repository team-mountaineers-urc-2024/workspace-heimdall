import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch.actions import ExecuteProcess
from ament_index_python import get_package_share_directory


def generate_launch_description():
    launch_description = LaunchDescription()


    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                'launches/components/liosam.launch.py'
                )
        )
    )

    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                'launches/components/lidar.launch.py'
                )
        )
    )
    
    launch_description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                'launches/heimdall/heimdall_static_tf.launch.py'
                )
        )
    )
    
    return launch_description
