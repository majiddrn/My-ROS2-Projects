from launch_ros.actions import Node

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
import xacro

def generate_launch_description():

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '/home/majiddrn/Academic/Robotics/rosbox/HW7And8Lab/maze.sdf'],
            output='screen'
        ),

        TimerAction(period=2.0,
            actions=[ 
                Node(
                    package='sensor_controller_py',
                    executable='controller', 
                    output='screen',
                ),
                Node(
                    package='ros_gz_bridge', 
                    executable='parameter_bridge', 
                    name='parameter_bridge',
                    arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
                    output='screen'
                ),
                Node(
                    package='ros_gz_bridge', 
                    executable='parameter_bridge', 
                    name='parameter_bridge',
                    arguments=['/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
                    output='screen'
                )
            ]
        ),

    ])
