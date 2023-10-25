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

    # Rviz config file
    rviz_config_file = PathJoinSubstitution([FindPackageShare("pecore_launch"), "rviz", "tutorial2.rviz"]) 
    world_file = PathJoinSubstitution([FindPackageShare('pecore_launch'), 'worlds', 'boxes.world'])  
    launch_perception = LaunchConfiguration('launch_perception', default=False)
    launch_navigation = LaunchConfiguration('launch_navigation', default=False)

    declared_arguments = []

    declared_arguments.append(LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above
        ])
    )

    # Jackal Robot
    declared_arguments.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('pecore_launch'), 
                             'launch/include/jackal_sim.launch.py')
            ),
            launch_arguments=[('rviz_config_file', rviz_config_file),
                              ('world_file', world_file),
                              ('launch_perception', launch_perception),
                              ('launch_navigation', launch_navigation)]
        )
    )

    # Launch Teleop
    declared_arguments.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('jackal_control'), 
                             'launch/teleop_keyboard.launch.py')
            )
        )
    )

    # # Fake global localization (odom in map frame)
    # declared_arguments.append(
    #     Node(package = "tf2_ros", 
    #          executable = "static_transform_publisher",
    #          arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"])
    # )

    # Fake global localization (odom in map frame)
    declared_arguments.append(
        Node(package = "tf_adder", 
             executable = "map_adder",
        output="both",
        parameters=[{"use_sim_time": True}] )
    )


    return LaunchDescription(declared_arguments)