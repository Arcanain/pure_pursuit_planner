import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'pure_pursuit_planner'
    simulator_package = 'arcanain_simulator'
    odrive_package = 'odrive_ros2_control'
    rviz_file_name = "pure_pursuit_planner.rviz"
    lidar_dir = get_package_share_directory('sllidar_ros2')

    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    with open(file_path, 'r') as file:
        robot_description = file.read()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", rviz_file_name]
    )

    dummy_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    output='screen',
    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'dummy_link']
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    imu_node = Node(
        package='adi_imu_tr_driver_ros2',
        executable='adis_rcv_csv_node',
        output="screen",
        parameters=[
            {"mode": "Attitude"},
            {"device": "/dev/ttyACM_IMU"},
        ],
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

    bwt901cl_pkg_node = Node(
        package='bwt901cl_pkg',
        executable='imu_bwt901cl',
        output="screen",
    )

    odometry_pub_node = Node(
        package=simulator_package,
        executable='odrive_odometry_pub',
        output="screen",
    )

    obstacle_pub_node = Node(
        package=simulator_package,
        executable='obstacle_pub',
        output="screen",
    )

    odrive_ros2_control_node = Node(
        package=odrive_package,
        executable='control_odrive_use_imu',
        output="screen",
    )

    path_publisher_node = Node(
        package='path_smoother',
        executable='path_publisher_gps',
        output="screen",
    )
    
    path_smoother_node = Node(
        package='path_smoother',
        executable='cubic_spline_node',
        output="screen",
    )

    pure_pursuit_planner_node = Node(
        package=package_name,
        executable='pure_pursuit_planner',
        output="screen",
    )

    laser_frame_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    output='screen',
    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser_frame']
    )
    perception_obstacle_node = Node(
           package='sllidar_ros2',
           executable='perception_obstacle',
           name='perception_obstacle',
           output='screen'
    )

    nodes = [
        rviz_node,
        dummy_node,
        imu_node,
        perception_obstacle_node,
        laser_frame_node,
        lidar_launch,
        robot_description_rviz_node,
        joint_state_publisher_rviz_node,
        odrive_ros2_control_node,
        odometry_pub_node,
        path_publisher_node,
        pure_pursuit_planner_node,
    ]

    return LaunchDescription(nodes)
