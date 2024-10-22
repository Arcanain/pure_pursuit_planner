import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'pure_pursuit_planner'
    simulator_package = 'arcanain_simulator'
    odrive_package = 'odrive_ros2_control'
    #rviz_file_name = "pure_pursuit_planner.rviz"
    lidar_rviz_file_name = "pure_pursuit_planner_lidar.rviz"

    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')

    with open(file_path, 'r') as file:
        robot_description = file.read()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", lidar_rviz_file_name]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_description_rviz_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_rviz_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='both',
        parameters=[{'joint_state_publisher': robot_description}]
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
        executable='odrive_imu_odom_pub',
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
    )
    
    path_smoother_node = Node(
        package='path_smoother',
        executable='save_path',
        output="screen",
    )

    pure_pursuit_planner_node = Node(
        package=package_name,
        executable='pure_pursuit_planner',
        output="screen",
    )

    nodes = [
        rviz_node,
        robot_description_rviz_node,
        joint_state_publisher_rviz_node,
        joy_linux_node,
        joy_to_twist_node,
        bwt901cl_pkg_node,
        odrive_ros2_control_node,
        odometry_pub_node,
        path_smoother_node,
    ]

    return LaunchDescription(nodes)
