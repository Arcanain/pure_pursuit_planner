import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'pure_pursuit_planner'
    simulator_package = 'arcanain_simulator'
    odrive_package = 'odrive_ros2_control'
    rviz_file_name = "pure_pursuit_planner.rviz"
    #lidar_rviz_file_name = "pure_pursuit_planner_lidar.rviz"

    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')

    with open(file_path, 'r') as file:
        robot_description = file.read()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", rviz_file_name]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=[
            '-d', 
            FindPackageShare('gnss_preprocessing').find('gnss_preprocessing') + '/rviz/gnss_preprocessing.rviz'
        ]
    )

    ros_bag_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', 
            os.path.expanduser('~/ドキュメント/ros2_bag_file/tukuba_1729737929'),
            '--rate', '30.0',
            '--remap', '/ublox/fix:=/ublox_gps_node/fix'
        ],
        output='screen'
    )

    # tf static transform from map to odom
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )

    # tf static transform from baselink to navsat (gps)
    baselink_to_navsat = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink_to_navsat',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'gps']
    )

    joy_linux_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        output='screen'
    )
    joy_to_twist_node = Node(
            package='ros2_joy_to_twist',
            executable='joy_to_twist',
            name='joy_to_twist',
            output='screen'
    )

    bwt901cl_pkg_node = Node(
        package='bwt901cl_pkg',
        executable='imu_bwt901cl',
        output="screen",
    )

    odometry_pub_node = Node(
        package=simulator_package,
        executable='odrive_gps_odom_pub',
        output="screen",
    )

    obstacle_pub_node = Node(
        package=simulator_package,
        executable='obstacle_pub',
        output="screen",
    )

    odrive_ros2_control_node = Node(
        package=odrive_package,
        executable='control_odrive_and_odom_pub',
        output="screen",
    )

    path_publisher_node = Node(
        package='path_smoother',
        executable='path_publisher',
        output="screen",
        sigterm_timeout=LaunchConfiguration('timeout_sec')  # タイムアウトを10秒に設定
    )
    
    path_smoother_node = Node(
        package='path_smoother',
        executable='save_path_gnss',
        output="screen",
    )

    pure_pursuit_planner_node = Node(
        package=package_name,
        executable='pure_pursuit_planner',
        output="screen",
    )

    gnss_node = Node(
        package='gnss_preprocessing',
        executable='gnss_preprocessing',
        output='screen'
    )

    arg_node = DeclareLaunchArgument(
        'timeout_sec', default_value='30',
        description='Time to wait for the process to terminate gracefully'
    )

    nodes = [
        ros_bag_node,
        gnss_node,
        map_to_odom,
        baselink_to_navsat,
        rviz_node,
        path_smoother_node,
    ]

    return LaunchDescription(nodes)
