from launch_ros.actions import Node

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_source import LaunchDescriptionSource
import xacro
from ament_index_python import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('eddiebot_gazebo'),
                             'launch/eddiebot_gz_sim.launch.py')
            ),
            launch_arguments={'world': 'maze_marked'}.items()
        ),

        TimerAction(period=8.0, actions=[

            


            Node(
                package='ros_gz_bridge', 
                executable='parameter_bridge', 
                name='parameter_bridge',
                arguments=['/kinect_rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image'],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge', 
                executable='parameter_bridge', 
                name='parameter_bridge',
                arguments=['/model/eddiebot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
                remappings=[
                    ('/model/eddiebot/tf', 'tf')
                ],
                output='screen'
            ),
            Node(
                package='ros_gz_bridge', 
                executable='parameter_bridge', 
                name='parameter_bridge',
                arguments=['/kinect_rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
                output='screen'
            ),
            Node(
                package='depthimage_to_laserscan',
                executable='depthimage_to_laserscan_node',
                name='depthimage_to_laserscan_node',
                remappings=[
                    ('depth_camera_info', '/kinect_rgbd_camera/camera_info'),
                    ('depth', '/kinect_rgbd_camera/depth_image' ),
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=["0", "0", "0", "0", "0", "0", "eddiebot/odom", "base_footprint"],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=["0", "0", "0", "0", "0", "0", "world", "base_footprint"],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=["0", "0", "0", "0", "0", "0", "world", "eddiebot/odom"],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=["0", "0", "0", "0", "0", "0", "eddiebot/base_footprint", "base_footprint"],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=["0", "0", "0", "0", "0", "0", "camera_link", "camera_depth_frame"],
                output='screen'
            )
        ])

    ])
