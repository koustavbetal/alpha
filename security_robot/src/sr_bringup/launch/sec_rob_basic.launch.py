import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package path
    pkg_share = FindPackageShare("sr_bringup")

    # Declare launch arguments
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=PathJoinSubstitution(
            [pkg_share, "descriptions", "security_robot.urdf.xacro"]
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

    bridge_config_arg = DeclareLaunchArgument(
        "bridge_config",
        default_value=PathJoinSubstitution([pkg_share, "config", "bridge_config.yaml"]),
        description="Path to the bridge configuration file",
    )

    # Set environment variable for Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ":",
            PathJoinSubstitution([pkg_share, ".."]),
        ],
    )

    # Robot description
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", LaunchConfiguration("urdf_path")]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True}
        ],
        output="screen",
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # RViz2
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "gz_args": LaunchConfiguration("gz_world"),
            "gz_version": "8",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_spawn_model.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "topic": "robot_description",
            "entity_name": "security_robot",
            "z": "0.075",
        }.items(),
    )

    # Individual bridges with proper remapping (more reliable)
    
    # Clock bridge (critical for sim time)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )
    
    # Joint states bridge (critical for robot description)
    joint_states_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/security_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
        remappings=[('/model/security_robot/joint_state', '/joint_states')],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )
    
    # Cmd_vel bridge
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/security_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        remappings=[('/model/security_robot/cmd_vel', '/cmd_vel')],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )
    
    # Odometry bridge
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/security_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[('/model/security_robot/odometry', '/odom')],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )
    
    # Camera bridge
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', '-r', '/camera:=/security_robot/camera/image_raw',
            '-r', '/camera_info:=/security_robot/camera/camera_info'
        ],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    # Alternative TF bridge syntax
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    return LaunchDescription(
        [
            urdf_path_arg,
            rviz_config_arg,
            gz_world_arg,
            bridge_config_arg,
            gz_resource_path,
            tf_bridge,
            gazebo_launch,  # Start Gazebo first
            clock_bridge,   # Clock bridge first
            joint_states_bridge,  # Joint states critical for robot description
            cmd_vel_bridge,
            odom_bridge,
            camera_bridge,
            robot_state_publisher,
            joint_state_publisher_gui,
            spawn_robot,
            rviz2,  # RViz last
        ]
    )