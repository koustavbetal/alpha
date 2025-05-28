import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package path
    pkg_share = FindPackageShare('sr_bringup')
    
    # Declare launch arguments
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([pkg_share, 'descriptions', 'security_robot.urdf.xacro']),
        description='Path to the URDF file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'rviz_config.rviz']),
        description='Path to the RViz config file'
    )
    
    gz_world_arg = DeclareLaunchArgument(
        'gz_world',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'ionic.sdf']),
        description='Path to the Gazebo world file'
    )
    
    # Set environment variable for Gazebo resource path (Untasted| Requires if URDF contains mesh along with tag "package://" 
    # "file://" seems to mitigate thie  err, although Uaware of the Repurcussions.)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''), 
            ':',
            PathJoinSubstitution([pkg_share, '..'])
        ]
    )
    
    # Robot description
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', LaunchConfiguration('urdf_path')
    ])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_world'),
            'gz_version': '8',
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_spawn_model.launch.py'
            ])
        ]),
        launch_arguments={
            'topic': 'robot_description',
            'entity_name': 'security_robot',
            'x': '-3.5',
            'y': '-0.01', 
            'z': '0.075',
            'R': '0.0',
            'P': '0.0',
            'Y': '1.57',
        }.items()
    )
    
    # Alternative: Direct executable (simpler but less robust)
    # spawn_robot_direct = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=['-topic', 'robot_description', '-z', '0.075'],
    #     output='screen'
    # )


    return LaunchDescription([
        urdf_path_arg,
        rviz_config_arg,
        gz_world_arg,
        # gz_resource_path,
        robot_state_publisher,
        # joint_state_publisher_gui,
        # rviz2,
        gazebo_launch,
        spawn_robot,
    ])