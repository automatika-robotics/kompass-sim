#!/usr/bin/env python3

"""Launch GAZEBO TurtleBot3 House Simulation + Map Server + Robot Localization"""

import os
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    this_package_dir = get_package_share_directory("kompass_sim")
    launch_file_dir = os.path.join(this_package_dir, "launch")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    run_rviz = LaunchConfiguration("run_rviz", default=True)

    sim_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gazebo_house_sim.launch.py")
        )
    )

    localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                this_package_dir,
                "params",
                "turtlebot3_gazebo_localization.yaml",
            )
        ],
        remappings=[("/set_pose", "/initialpose")],
    )

    footprint_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "/base_link"],
    )

    rviz_config_dir = os.path.join(
        this_package_dir,
        "rviz",
        "gazebo.rviz",
    )

    # RVIZ
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_dir],
        condition=IfCondition(run_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            footprint_publisher,
            sim_launch_cmd,
            localization_node,
            rviz_node,
        ]
    )
