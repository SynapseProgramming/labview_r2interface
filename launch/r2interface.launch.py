from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='labview_r2interface',
            executable='odom_convert',
            name='odom_convert'
        )
        ])
