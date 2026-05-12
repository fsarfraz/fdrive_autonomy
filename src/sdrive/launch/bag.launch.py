import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess


def generate_launch_description():

    elevation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("elevation_mapping_cupy"),
                "launch",
                "elevation_mapping.launch.py",
            )
        ),
        launch_arguments={'use_nixgl': 'false'}.items()
    )

    bag_launch = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            "/home/fsarfraz/workspaces/sdrive_autonomy/sdrive_bag/sdrive_bag_0.mcap",
            "--clock",
        ]
    )

    sdrive_dir = get_package_share_directory("sdrive")

    rko_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rko_lio"),
                "launch",
                "odometry.launch.py",
            )
        ),
    )

    loop_closure = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("simple_loop_closure"),
                "launch",
                "simple_loop_closure.launch.py",
            )
        ),
    )


    gridmap_to_costmap = Node(
        package="sdrive",
        executable="gridmap_to_costmap",
        name="gridmap_to_costmap",
        parameters=[{"layer": "slope_traversability"}],
    )

    global_costmap = Node(
        package="sdrive",
        executable="global_costmap",
        name="global_costmap",
        parameters=[{
            "resolution": 0.05,
            "map_size": 6.0,
            "publish_rate": 5.0,
            "log_odds_occ": 0.2,
            "log_odds_free": -1.5,
            "decay_rate": 0.0,
            "unknown_threshold": 0.3,
            "inflation_radius": 0.1,
            "inflation_threshold": 10,
            "expansion_padding": 0.5
        }],
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            bag_launch,
            rko_lio_launch,
            loop_closure,
            elevation_launch,
            gridmap_to_costmap,
            global_costmap,
        ]
    )
