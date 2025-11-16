#!/usr/bin/env python3

"""Launch GAZEBO TurtleBot3 House Simulation + Map Server + Robot Localization"""

import os
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch.actions import (
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
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

    # Map server
    map_server_config_path = os.path.join(
        this_package_dir, "maps", "turtlebot3_gazebo_house.yaml"
    )

    map_server_node = LifecycleNode(
        name="map_server",
        namespace="",
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_server_config_path}],
    )

    emit_event_to_configure_map_server = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_map_server = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            goal_state="inactive",
            entities=[
                LogInfo(
                    msg="node 'Map Server' reached the 'inactive' state, 'activating'."
                ),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
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
            map_server_node,
            emit_event_to_configure_map_server,
            activate_map_server,
            rviz_node,
        ]
    )
