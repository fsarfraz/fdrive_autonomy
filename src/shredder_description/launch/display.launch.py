from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_package",
            default_value="shredder_description",
            description="Package where the URDF file is located",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_file",
            default_value="shredder.urdf.xml",
            description="URDF file",
        )
    )
    
    # Initialize Arguments
    urdf_package = LaunchConfiguration("urdf_package")
    urdf_file = LaunchConfiguration("urdf_file")
    
    # Get URDF via xacro
    # This will process shredder_ouster.urdf.xacro, which already includes shredder.urdf.xml
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(urdf_package), "urdf", urdf_file]
            ),
        ]
    )
    
    # Wrap the robot description with ParameterValue to properly encode it as a string parameter
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )
    
    # Ouster driver launch
    ouster_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ouster_ros"),
                "launch",
                "driver.launch.py"
            ])
        ])
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(urdf_package), "rviz", "view.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    return LaunchDescription(
        declared_arguments + 
        [
            robot_state_publisher_node,
            ouster_driver_launch,
            rviz_node,
        ]
    )