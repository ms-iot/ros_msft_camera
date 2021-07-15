# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('win_camera')
    rviz_default_view = os.path.join(share_dir, 'rviz', 'default_view.rviz')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='win_camera', executable='win_camera_node', output='screen',
            name=['win_camera'],
            parameters=[
                {'frame_rate': 5.0},
                {'frame_id': 'camera'},
                {'image_width': 1280 },
                {'image_height': 720 },
                {'videoDeviceId': '\\\\?\\USB#VID_045E&PID_097D&MI_00#7&16658bf3&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\\GLOBAL'},
                {'camera_info_url': 'package://win_camera/camera_info/camera.yaml'},
            ],
            arguments=[{'--ros-args', '--log-level', 'INFO'}]
            ),

        #launch_ros.actions.Node(
        #    package='tf2_ros', executable='static_transform_publisher', output='screen',
        #    name=['static_transform_publisher'],
        #    arguments=[
        #        '0.1', '0.2', '0.3', '0.4', '.5', '.6', 'map', 'camera'
        #    ]),
            
#        launch_ros.actions.Node(
#            package='rviz2', executable='rviz2', output='screen',
#            name=['rviz2'],
#            arguments=[
#                '-d', rviz_default_view
#            ]),
    ])