from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

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
            default_value="simulated_shredder.urdf.xml",
            description="URDF file",
        )
    )

    # Initialize Arguments
    urdf_package = LaunchConfiguration("urdf_package")
    urdf_file = LaunchConfiguration("urdf_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(urdf_package), "urdf", urdf_file]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Start Gazebo server
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

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

    # Configure the teleop_twist_keyboard node for manual control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix="xterm -e"
    )

    # Spawn the robot with a delay to ensure Gazebo is ready
    spawn_entity = TimerAction(
        period=5.0,  # 5-second delay to ensure Gazebo is fully started
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                arguments=['-entity', 'shredder', '-topic', 'robot_description'],  # removed leading slash
                output='screen'
            )
        ]
    )

    # Configure our custom track controller node
    track_controller_node = Node(
        package='shredder_description',
        executable='track_controller',  # Remove .py extension
        name='track_controller',
        output='screen',
        emulate_tty=True  # Enables proper console output formatting
    )

    # Launch Description
    return LaunchDescription(
        declared_arguments +
        [
            # Start Gazebo first
            # gazebo_launch,
            
            # Robot state publisher
            robot_state_publisher_node,
            joint_state_publisher_node,
            
            # Spawn the robot after a delay
            # spawn_entity,
            
            # Start other nodes
            # rviz_node,
            teleop_node,
            track_controller_node
        ]
    )