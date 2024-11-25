import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'pure_pursuit_planner'
    simulator_package = 'arcanain_simulator'
    odrive_package = 'odrive_ros2_control'
    judge_package = 'rtk_judge'
    rviz_file_name = "pure_pursuit_planner.rviz"
    lidar_dir = get_package_share_directory('sllidar_ros2')
    emcl_dir = get_package_share_directory('emcl2')

    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')

    # シミュレーション時間の設定
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    with open(file_path, 'r') as file:
        robot_description = file.read()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", rviz_file_name]
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    emcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(emcl_dir, 'launch', 'emcl2_origin.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]  # シミュレーション時間を適用
    )

    robot_description_rviz_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': use_sim_time}
        ]
    )

    joint_state_publisher_rviz_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    odometry_pub_node = Node(
        package=simulator_package,
        executable='odrive_gps_switch_pub',
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    obstacle_pub_node = Node(
        package=simulator_package,
        executable='obstacle_pub',
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    odrive_ros2_control_node = Node(
        package=odrive_package,
        executable='control_odrive_and_odom_pub',
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    path_publisher_node = Node(
        package='path_smoother',
        executable='path_publisher',
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    path_smoother_node = Node(
        package='path_smoother',
        executable='cubic_spline_node',
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    pure_pursuit_planner_node = Node(
        package=package_name,
        executable='pure_pursuit_planner',
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )
    gnss_node = Node(
        package='gnss_preprocessing',
        executable='gnss_preprocessing',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ros_bag_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', 
            os.path.expanduser('~/ドキュメント/ros2_bag_file/03test_run_1123_1732344777'),
            '--rate','1.0',
            '--clock'
        ],
        output='screen'
    )

    rtk_judge_node = Node(
        package=judge_package,
        executable='judge_rtk_status',
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    nodes = [
        ros_bag_node,
        gnss_node,
        robot_description_rviz_node,
        joint_state_publisher_rviz_node,
        emcl_launch,
        lidar_launch,
        rtk_judge_node,
        path_publisher_node,
        odometry_pub_node,
        pure_pursuit_planner_node,
    ]

    return LaunchDescription(nodes)
