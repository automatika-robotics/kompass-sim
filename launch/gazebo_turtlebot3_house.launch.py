#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool, Hyungyu Kim

"""Launch GAZEBO TurtleBot3 House Simulation + Map Server + Robot Localization"""

import os
from launch.substitutions import PythonExpression
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
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    this_package_dir = get_package_share_directory("kompass_sim")
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    run_rviz = LaunchConfiguration("run_rviz", default=True)

    TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]

    urdf_file_name = "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"
    frame_prefix = LaunchConfiguration("frame_prefix", default="")

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "urdf", urdf_file_name
    )

    with open(urdf_path, "r") as infp:
        robot_desc = infp.read()

    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])}],
    )

    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    x_pose = LaunchConfiguration("x_pose", default="2.0")
    y_pose = LaunchConfiguration("y_pose", default="0.5")

    world = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "worlds",
        "turtlebot3_house.world",
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s -v2 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g -v2 ", "on_exit_shutdown": "true"}.items(),
    )

    footprint_publisher_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(get_package_share_directory("turtlebot3_gazebo"), "models"),
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
            gzserver_cmd,
            gzclient_cmd,
            spawn_turtlebot_cmd,
            robot_state_publisher_cmd,
            footprint_publisher_cmd,
            set_env_vars_resources,
            localization_node,
            map_server_node,
            emit_event_to_configure_map_server,
            activate_map_server,
            rviz_node,
        ]
    )
