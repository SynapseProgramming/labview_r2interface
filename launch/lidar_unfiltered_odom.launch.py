import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # this function gets the absolute shared path of the package
    sick_scan = get_package_share_directory("sick_scan2")

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sick_scan, "launch", "sick_tim_5xx.launch.py")
        )
    )

    unfiltered_odom_converter = Node(
        package="labview_r2interface",
        executable="unfiltered_odom_convert",
        name="unfiltered_odom_convert",
    )

    # max_range:  only datapoints with distances smaller
    # than this range are taken into account (m)

    # max_range_difference: the maximum difference in distance between two
    # consecutive points (m)

    # filter_window: maximum number of neighbouring points to compare to

    laser_converter = Node(
        package="labview_r2interface",
        executable="laser_filter",
        name="laser_filter",
        parameters=[
            {"max_range": 2.0},
            {"max_range_difference": 0.04},
            {"filter_window": 2},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(unfiltered_odom_converter)
    ld.add_action(lidar_launch)
    ld.add_action(laser_converter)
    return ld
