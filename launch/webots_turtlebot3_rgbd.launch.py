#!/usr/bin/env python

"""Launch Webots TurtleBot3 with depth camera + Robot Localization"""

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import (
    get_package_share_directory,
)
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


def generate_launch_description():
    this_package_dir = get_package_share_directory("kompass_sim")
    mode = LaunchConfiguration("mode")
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    run_rviz = LaunchConfiguration("run_rviz", default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution(
            [this_package_dir, "worlds", "turtlebot3_waffle_depth.wbt"]
        ),
        mode=mode,
        ros2_supervisor=True,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": '<robot name=""><link name=""/></robot>'}],
    )

    footprint_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )

    # Static TF for the depth camera frame
    camera_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0.03", "0", "0.05", "0", "0", "0", "base_link", "camera_link",
        ],
    )

    # ROS control spawners
    controller_manager_timeout = ["--controller-manager-timeout", "50"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    ros2_control_params = os.path.join(
        this_package_dir, "resource", "ros2control_waffle.yml"
    )
    diffdrive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["diffdrive_controller", "--param-file", ros2_control_params]
        + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_state_broadcaster"] + controller_manager_timeout,
    )
    ros_control_spawners = [
        diffdrive_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    robot_description_path = os.path.join(
        this_package_dir, "resource", "turtlebot_waffle_webots.urdf"
    )
    use_twist_stamped = "ROS_DISTRO" in os.environ and (
        os.environ["ROS_DISTRO"] in ["rolling", "jazzy"]
    )
    if use_twist_stamped:
        mappings = [
            ("/diffdrive_controller/cmd_vel", "/cmd_vel"),
            ("/diffdrive_controller/odom", "/odom"),
        ]
    else:
        mappings = [
            ("/diffdrive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/diffdrive_controller/odom", "/odom"),
        ]
    turtlebot_driver = WebotsController(
        robot_name="TurtleBot3Burger",
        parameters=[
            {
                "robot_description": robot_description_path,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": True,
            },
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
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
                "turtlebot3_localization.yaml",
            )
        ],
        remappings=[("/set_pose", "/initialpose")],
    )

    # Combine color + depth into realsense2_camera_msgs/RGBD
    rgbd_publisher = Node(
        package="kompass_sim",
        executable="rgbd_publisher",
        name="rgbd_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Wait for the simulation to be ready to start navigation nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=ros_control_spawners + [localization_node],
    )

    rviz_config_dir = os.path.join(
        this_package_dir,
        "rviz",
        "webots.rviz",
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
            DeclareLaunchArgument(
                "mode",
                default_value="realtime",
                description="Webots startup mode",
            ),
            webots,
            webots._supervisor,
            robot_state_publisher,
            footprint_publisher,
            camera_tf_publisher,
            turtlebot_driver,
            waiting_nodes,
            # This action will kill all nodes once the Webots simulation has exited
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
            rviz_node,
            rgbd_publisher,
        ]
    )
