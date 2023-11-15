from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import GroupAction

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    usingtime = LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        ])

    config_ukf_map = PathJoinSubstitution(
        [FindPackageShare('second_deliverable'),
            'config',
            'ukf_gps.yaml'],
    )

    # Fake global localization (odom in map frame)
    localizer =Node(
            package = "tf_adder", 
            executable = "map_adder",
            name="publisher_odom",  
            output="screen"
        )


    jackalSim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory('pecore_launch') 
                                + '/launch/practicum2.launch.py')
        )

    # Global Localization (NavSat)
    global_navsat = GroupAction([
        # GNSS conversion
        Node(
            package='pecore_launch',
            executable='navsat_odom',
            name='navsat_odom',
            output='screen'
        ),
    ])

    # robot_localization ukf node for the map frame
    robot_localization_ukf_node_map = Node(
                    package="robot_localization",
                    executable="ukf_node",
                    name="ukf_gps",
                    output="screen",
                    parameters=[config_ukf_map],
                    remappings=[('odometry_filtered', 'odometry/filtered_map')]
                )


    # Arguments
    ld =  LaunchDescription()

    ld.add_action(usingtime)
    ld.add_action(jackalSim)
    ld.add_action(localizer)
    ld.add_action(global_navsat)
    ld.add_action(robot_localization_ukf_node_map)

    return ld
