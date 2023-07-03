import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # this function gets the absolute shared path of the package
    sick_scan = get_package_share_directory("sick_scan2")
    laser_filters = get_package_share_directory("laser_filters")

    # lidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(sick_scan, "launch", "sick_tim_5xx.launch.py")
    #     )
    # )

    # filter_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(laser_filters, "examples", "speckle_filter_example.launch.py")
    #     )
    # )

    odom_converter = Node(
        package="labview_r2interface", executable="odom_convert", name="odom_convert"
    )

    ld = LaunchDescription()
    ld.add_action(odom_converter)
    # ld.add_action(lidar_launch)
    # ld.add_action(filter_launch)
    return ld
