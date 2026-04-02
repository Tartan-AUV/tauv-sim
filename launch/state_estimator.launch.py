from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share_dir = Path(get_package_share_directory("tauv_sim"))
    sim_param_file = sim_share_dir / "config" / "params.yaml"
    trajectory_file = sim_share_dir / "config" / "trajectories" / "osprey_square.yaml"

    common_share_dir = Path(get_package_share_directory("tauv_core"))
    common_ekf_file = common_share_dir / "config" / "ekfFUNNY.yaml"

    # Timestamped bag name
    timestamp = datetime.now().strftime('%Y.%m.%d_%H.%M.%S')
    bag_name = f"sim_{timestamp}"
    bag_name_latest = "latest"
    common_ekf_record_file = (
        Path("/tauv-mono/ros_ws") / "bags" / bag_name
    )
    common_ekf_record_file_latest = (
        Path("/tauv-mono/ros_ws") / "bags" / bag_name_latest
    )
    print(f"Recording EKF data to: {common_ekf_record_file}")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'record', default_value='false', description='Enable rosbag recording'
            ),
            Node(
                package="tauv_sim",
                executable="tauv_sim",
                name="tauv_sim",
                parameters=[str(sim_param_file)],
                # arguments=["--kinematic", str(trajectory_file)],
                output="screen",
            ),
            TimerAction(
                period=8.0,
                actions=[
                    LogInfo(msg="Starting EKF filter node!!!!!!"),
                    Node(
                        package="robot_localization",
                        executable="ekf_node",
                        name="ekf_filter_node",
                        parameters=[str(common_ekf_file)],
                        output="screen",
                    ),
                ],
            ),
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration('record')),
                cmd=[
                    'ros2', 'bag', 'record',
                    '-a',
                    '-s', 'mcap',
                    '--polling-interval', '1',
                    '-x', '/os/sensors/cam.*',
                    '-o', str(common_ekf_record_file),
                ],
                output='screen',
            ),
            # ExecuteProcess(
            #     condition=IfCondition(LaunchConfiguration('record')),
            #     cmd=[
            #         'ros2', 'bag', 'record',
            #         '-a',
            #         '-s', 'mcap',
            #         '-o', str(common_ekf_record_file_latest),
            #     ],
            #     output='screen',
            # ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_imu',
                arguments=['0', '0', '0', '0', '0', '0', 'os/base_link', 'imu_xsens_link'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_depth',
                arguments=['0', '0', '0', '0', '0', '0', 'os/base_link', 'depth_link'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_dvl',
                arguments=['0', '0', '0', '0', '0', '0', 'os/base_link', 'dvl_link'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='tauv_autonomy',
                executable='controller',
                name='controller',
                output='screen',
            ),
            Node(
                package='tauv_autonomy',
                executable='thruster_forces',
                name='thruster_forces',
                output='screen',
            ),
            Node(
                package='tauv_autonomy',
                executable='thruster_rpms',
                name='thruster_rpms',
                output='screen',
            )
        ]
    )
