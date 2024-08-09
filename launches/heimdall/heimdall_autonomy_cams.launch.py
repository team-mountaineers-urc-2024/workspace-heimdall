# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import os
from pathlib import Path  # noqa: E402
import sys

# Hack to get relative import of .camera_config file working
# dir_path = os.path.__file__
# sys.path.append(dir_path)
# 
from launch import LaunchDescription  # noqa: E402
from launch.actions import DeclareLaunchArgument  # noqa: E402
from launch_ros.actions import Node, RosTimer  # noqa: E402
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

LAUNCH_DIR = os.path.normpath(os.path.join(os.path.abspath(__file__), '../..'))

def generate_launch_description():
    ld = LaunchDescription()

    perception_left = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_01',
            namespace='',
            parameters= [Path(LAUNCH_DIR, 'config', 'insane.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_01'},
                        {'camera_info_url' : './logitech_01_info_640x480.yaml'},
                        {'frame_id' : 'perception_left'},
                        {'camera_name' : 'logitech_01'},
                        {'framerate' : 10.0},
                        {'camera_parameters_url': str(Path(LAUNCH_DIR, 'config', 'insane.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_01/image_raw'),
                ('image_raw/compressed', 'logitech_01/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_01/compressedDepth'),
                ('image_raw/theora', 'logitech_01/image_raw/theora'),
                ('camera_info', 'logitech_01/camera_info'),
            ]
        )
    
    perception_right = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_03',
            namespace='',
            parameters= [Path(LAUNCH_DIR, 'config', 'insane.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_03'},
                        {'camera_info_url' : './logitech_03_info_640x480.yaml'},
                        {'frame_id' : 'perception_right'},
                        {'camera_name' : 'logitech_03'},
                        {'framerate' : 10.0},
                        {'camera_parameters_url': str(Path(LAUNCH_DIR, 'config', 'insane.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_03/image_raw'),
                ('image_raw/compressed', 'logitech_03/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_03/compressedDepth'),
                ('image_raw/theora', 'logitech_03/image_raw/theora'),
                ('camera_info', 'logitech_03/camera_info'),
            ]
        )
    
    front = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_05',
            namespace='',
            parameters= [Path(LAUNCH_DIR, 'config', 'insane.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_05'},
                        {'camera_info_url' : './logitech_05_info_640x480.yaml'},
                        {'frame_id' : 'front'},
                        {'camera_name' : 'logitech_05'},
                        {'framerate' : 10.0},
                        {'camera_parameters_url': str(Path(LAUNCH_DIR, 'config', 'insane.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_05/image_raw'),
                ('image_raw/compressed', 'logitech_05/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_05/compressedDepth'),
                ('image_raw/theora', 'logitech_05/image_raw/theora'),
                ('camera_info', 'logitech_05/camera_info'),
            ]
        )
    
    back = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_10',
            namespace='',
            parameters= [Path(LAUNCH_DIR, 'config', 'insane.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_10'},
                        {'camera_info_url' : './logitech_10_info_640x480.yaml'},
                        {'frame_id' : 'back'},
                        {'camera_name' : 'logitech_10'},
                        {'framerate' : 10.0},
                        {'camera_parameters_url': str(Path(LAUNCH_DIR, 'config', 'insane.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_10/image_raw'),
                ('image_raw/compressed', 'logitech_10/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_10/compressedDepth'),
                ('image_raw/theora', 'logitech_10/image_raw/theora'),
                ('camera_info', 'logitech_10/camera_info'),
            ]
        )
    
    perception_front = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_18',
            namespace='',
            parameters= [Path(LAUNCH_DIR, 'config', 'incomprehensible.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_18'},
                        {'camera_info_url' : './logitech_18_info_1920x1080.yaml'},
                        {'frame_id' : 'perception_front'},
                        {'camera_name' : 'logitech_18'},
                        {'framerate' : 10.0},
                        {'camera_parameters_url': str(Path(LAUNCH_DIR, 'config', 'incomprehensible.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_18/image_raw'),
                ('image_raw/compressed', 'logitech_18/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_18/compressedDepth'),
                ('image_raw/theora', 'logitech_18/image_raw/theora'),
                ('camera_info', 'logitech_18/camera_info'),
            ]
        )
    
    perception_back = \
        Node(
            package='usb_cam', executable='usb_cam_node_exe', output='screen',
            name='logitech_19',
            namespace='',
            parameters= [Path(LAUNCH_DIR, 'config', 'insane.yaml'),
                        {'video_device' : '/dev/urc/cam/logitech_19'},
                        {'camera_info_url' : './logitech_19_info_640x480.yaml'},
                        {'frame_id' : 'perception_back'},
                        {'camera_name' : 'logitech_19'},
                        {'framerate' : 10.0},
                        {'camera_parameters_url': str(Path(LAUNCH_DIR, 'config', 'insane.yaml'))}],
            remappings = [
                ('image_raw', 'logitech_19/image_raw'),
                ('image_raw/compressed', 'logitech_19/image_compressed'),
                ('image_raw/compressedDepth', 'logitech_19/compressedDepth'),
                ('image_raw/theora', 'logitech_19/image_raw/theora'),
                ('camera_info', 'logitech_19/camera_info'),
            ]
        )
    
    ld.add_action(perception_left)
    ld.add_action(perception_right)
    ld.add_action(perception_back)
    ld.add_action(perception_front)
    ld.add_action(front)
    ld.add_action(back)
    return ld
