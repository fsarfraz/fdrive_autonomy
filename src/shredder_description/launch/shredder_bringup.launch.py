from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    shredder_dir = get_package_share_directory('shredder_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    sdrive_dir = get_package_share_directory('sdrive')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # tb3_dir = get_package_share_directory('turtlebot3_gazebo')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(DeclareLaunchArgument(
        'x_pose', default_value='60.0',
        description='X position to spawn the robot'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'y_pose', default_value='-17.0', 
        description='Y position to spawn the robot'))
        
    # declared_arguments.append(DeclareLaunchArgument(
    #     'world', default_value=os.path.join(shredder_dir, 'worlds', 'turtlebot3_house.world'),
    #     description='Full path to world model file to load'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'world', default_value=os.path.join(shredder_dir, 'worlds', 'simple_baylands.sdf'),
        description='Full path to world model file to load'))
    
    # LaunchConfiguration variables
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    world = LaunchConfiguration('world')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    
    # Load URDF
    urdf_file_name = "shredder_core.xacro"
    urdf_path = os.path.join(shredder_dir, 'urdf', urdf_file_name)
    
    # with open(urdf_path, 'r') as infp:
    #     robot_desc = infp.read()
   
    # Set environment for Gazebo resources
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(shredder_dir, 'models'))
    
    # Launch Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world]}.items()
    )
    
    # Launch Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4'}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_path]),
            'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
        }],
    )
    
    # Spawn robot directly (SINGLE SPAWN POINT)
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_shredder',
        arguments=[
            '-name', 'shredder',
            '-topic', "/robot_description",
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.2'  # Slightly higher to avoid ground collision
        ],
        output='screen',
    )
    
    # ROS-Gazebo Bridge
    bridge_params = os.path.join(shredder_dir, 'params', 'shredder_bridge.yaml')
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    
    # Autonomy stack (mirrors src/sdrive/launch/run.launch.py, minus ouster /
    # robot_state_publisher / sdrive_controller / vesc_socketcan which Gazebo replaces)
    nav2_params = os.path.join(sdrive_dir, 'params', 'nav2_params.yaml')

    elevation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('elevation_mapping_cupy'),
                'launch',
                'elevation_mapping.launch.py',
            )
        ),
        launch_arguments={'use_nixgl': 'false', 'use_sim_time': 'true'}.items(),
    )

    rko_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rko_lio'),
                'launch',
                'odometry.launch.py',
            )
        ),
    )

    loop_closure = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('simple_loop_closure'),
                'launch',
                'simple_loop_closure.launch.py',
            )
        ),
    )

    gridmap_to_costmap = Node(
        package='sdrive',
        executable='gridmap_to_costmap',
        name='gridmap_to_costmap',
        parameters=[{'layer': 'slope_traversability', 'use_sim_time': True}],
    )

    global_costmap = Node(
        package='sdrive',
        executable='global_costmap',
        name='global_costmap',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05,
            'map_size': 4.0,
            'publish_rate': 5.0,
            'log_odds_occ': 0.2,
            'log_odds_free': -1.5,
            'decay_rate': 0.0,
            'unknown_threshold': 0.3,
            'inflation_radius': 0.1,
            'inflation_threshold': 10,
        }],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    # RViz
    rviz_config_file = os.path.join(shredder_dir, "rviz", "view.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(
        declared_arguments + [
            gzserver_cmd,
            gzclient_cmd,
            robot_state_publisher_node,
            spawn_robot_node,  # ONLY ONE SPAWN CALL
            bridge_node,
            set_env_vars_resources,
            rko_lio_launch,
            # loop_closure,
            # elevation_launch,
            # gridmap_to_costmap,
            # global_costmap,
            # nav2_launch,
            rviz_node,
        ]
    )