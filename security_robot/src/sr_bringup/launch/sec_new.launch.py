import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,SetEnvironmentVariable,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,FindExecutable,PathJoinSubstitution,LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # Get package path
    pkg_share = FindPackageShare("sr_bringup")

    # Declare launch arguments
    urdf_path_arg = DeclareLaunchArgument("urdf_path",
        default_value=PathJoinSubstitution([pkg_share, "descriptions", "security_robot.urdf.xacro"]
        ),
        description="Path to the URDF file",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([pkg_share, "config", "rviz_config.rviz"]),
        description="Path to the RViz config file",
    )
    gz_world_arg = DeclareLaunchArgument(
        "gz_world",
        default_value="empty.sdf",
        description="Path to the Gazebo world file",
    )

    # Robot description
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", LaunchConfiguration("urdf_path")]
    )

    bridge_description = PathJoinSubstitution([pkg_share, "config", "bridge_config.yaml"])

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
        ],
        output="screen",
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        # parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # RViz2
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        # parameters=[{"use_sim_time": True}],
        output="screen",
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={
            "gz_args": LaunchConfiguration("gz_world"),
            "gz_version": "8",
            "on_exit_shutdown": "true",
        }.items(),
    )
    ros_gazebo_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare("ros_gz_bridge"),
            "launch",
            "ros_gz_bridge.launch.py",])]
        ),
        launch_arguments={
            "bridge_name": "parameter_bridge",
            "config_file": bridge_description,
        }.items(),
    )

    # Spawn robot
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare("ros_gz_sim"),
            "launch",
            "gz_spawn_model.launch.py",])]
        ),
        launch_arguments={
            "topic": "robot_description",
            "entity_name": "security_robot",
            "z": "0.075",
        }.items(),
    )


    return LaunchDescription(
    [
        urdf_path_arg,
        rviz_config_arg,
        gz_world_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        gazebo_launch,
        spawn_robot,
        ros_gazebo_bridge,
        rviz2  # RViz last
    ]
)