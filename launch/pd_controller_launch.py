import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pd_controller_node = Node(
            package='uma_arm_control',
            executable='pd_controller',
            name='pd_controller',
            output='screen'
        )

    return LaunchDescription([pd_controller_node])