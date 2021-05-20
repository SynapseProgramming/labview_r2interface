from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='labview_r2interface',
            executable='odom_convert',
            name='odom_convert'
        ),
        Node(
            package='labview_r2interface',
            executable='laserscan_convert',
            name='laserscan_convert'
        )

        ])
