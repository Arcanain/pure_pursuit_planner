import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'pure_pursuit_planner'
    simulator_package = 'arcanain_simulator'
    rviz_file_name = "pure_pursuit_planner.rviz"

    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')

    with open(file_path, 'r') as file:
        robot_description = file.read()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", rviz_file_name]
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

    odometry_pub_node = Node(
        package=simulator_package,
        executable='odometry_pub',
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
        odometry_pub_node,
        pure_pursuit_planner_node,
    ]

    return LaunchDescription(nodes)
