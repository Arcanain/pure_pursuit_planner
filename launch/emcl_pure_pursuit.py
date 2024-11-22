import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, TextSubstitution

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 引数の宣言（コマンドライン引数で指定可能にする）
    odometry_init_x_arg = DeclareLaunchArgument(
        "init_x", default_value="0.0", description="Initial x position for odometry_pub"
    )
    odometry_init_y_arg = DeclareLaunchArgument(
        "init_y", default_value="0.0", description="Initial y position for odometry_pub"
    )
    odometry_init_th_arg = DeclareLaunchArgument(
        "init_th", default_value="0.0", description="Initial theta for odometry_pub"
    )


    package_name = 'pure_pursuit_planner'
    simulator_package = 'arcanain_simulator'
    odrive_package = 'odrive_ros2_control'
    judge_package = 'rtk_judge'
    rviz_file_name = "pure_pursuit_planner.rviz"
    lidar_dir = get_package_share_directory('sllidar_ros2')
    emcl_dir = get_package_share_directory('emcl2')

    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')

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

    perception_obstacle_node = Node(
           package='sllidar_ros2',
           executable='perception_obstacle',
           name='perception_obstacle',
           output='screen'
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
        executable='emcl_odom_pub',
        output="screen",
        parameters=[
            {"init_x": LaunchConfiguration("init_x")},
            {"init_y": LaunchConfiguration("init_y")},
            {"init_th": LaunchConfiguration("init_th")}
        ]
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
        executable='cubic_spline_node',
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

    ros_bag_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', 
            os.path.expanduser('~/Documents/ros2_bag_file/tukuba_1729737929'),
            '--rate','2.0',
            '--remap', '/ublox/fix:=/ublox_gps_node/fix'
        ],
        output='screen'
    )

    rtk_judge_node = Node(
        package=judge_package,
        executable='judge_rtk_status',
        output="screen",
    )

    nodes = [
        odometry_init_x_arg,
        odometry_init_y_arg,
        odometry_init_th_arg,
        lidar_launch,
        perception_obstacle_node,
        robot_description_rviz_node,
        joint_state_publisher_rviz_node,
        odrive_ros2_control_node,
        emcl_launch,
        rtk_judge_node,
        odometry_pub_node ,
        path_publisher_node,
        pure_pursuit_planner_node,
    ]

    return LaunchDescription(nodes)
