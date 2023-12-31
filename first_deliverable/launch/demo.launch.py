from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    declared_arguments = []

    # Set 'use_sim_time' in all nodes
    declared_arguments.append(LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        ])
    )

    # Include the launch of the deliverable
    declared_arguments.append(
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('pecore_launch') 
                                + '/launch/practicum1.launch.py')
        )
    )

    # Fake global localization (odom in map frame)
    declared_arguments.append(
        Node(package = "tf_adder", 
             executable = "map_adder",
        output="screen"
        )
    )

    # Add tf for aruco_H_descam
    declared_arguments.append(
        Node(package = "tf2_ros",
                      executable = "static_transform_publisher",
                    #   arguments = ["0.0", "0.0", "0.1","-0.7071068", "0.7071068", "0.0", "0.7071068", "aruco_marker_frame", "desired_camera"],
                      arguments = ["0.0", "0.0", "0.1","0.0", "0.7071068", "0.0", "0.7071068", "aruco_marker_frame", "desired_camera"],
                      output = "screen"
           )
    )
    # Add tf for descam_H_desrobot
    declared_arguments.append(
        Node(package = "tf2_ros",
                      executable = "static_transform_publisher",
                      arguments = ["-0.242", "0.0", "-0.216", "0.0", "0.0", "0.0", "1.0", "desired_camera", "desired_robot"],
                      output = "screen"
           )
    )
    # Add tf for cam_H_aruco
    declared_arguments.append(
        Node(package = "first_deliverable",
                      executable = "tf_remap",
                      output = "screen"
           )
    )
    # Apply control
    declared_arguments.append(
        Node(package = "first_deliverable",
                      executable = "pbvs",
                      output = "screen",
                      parameters=[{"lam":0.3}]
           )
    )


    return LaunchDescription(declared_arguments)