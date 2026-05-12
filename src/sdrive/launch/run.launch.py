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
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file_name = "shredder.xacro"
    urdf_path = os.path.join(
        get_package_share_directory("sdrive"), "urdf", urdf_file_name
    )
    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    nav2_params = os.path.join(
        get_package_share_directory("sdrive"), "params", "nav2_params.yaml"
    )

    ouster_ros_dir = get_package_share_directory("ouster_ros")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    robot_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ouster_ros_dir, "launch", "driver.launch.py")
        ),
    )

    elevation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("elevation_mapping_cupy"),
                "launch",
                "elevation_mapping.launch.py",
            )
        ),
        launch_arguments={"use_nixgl": "false"}.items(),
    )

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
            "resolution": 0.075,
            "map_size": 5.0,
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

    sdrive_controller = Node(
        package="sdrive",
        executable="sdrive_controller",
        name="sdrive_controller",
        output="screen",
    )

    vesc_socketcan_node = Node(
        package="sdrive",
        executable="vesc_socketcan_node",
        name="vesc_socketcan_node",
        output="screen",
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params,
            "use_sim_time": "false",
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            robot_publisher,
            ouster_launch,
            rko_lio_launch,
            loop_closure,
            elevation_launch,
            gridmap_to_costmap,
            global_costmap,
            sdrive_controller,
            vesc_socketcan_node,
            # nav2_launch,
        ]
    )
