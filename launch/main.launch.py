"""Launches dynamic tauv_sim with robot_localization EKF, controller stack, and optional
rosbag recording.

Updated to follow the same architecture as main.launch.py (confirmed working on the robot):
- state estimation config now lives under tauv_state_estimation (not tauv_core)
- EKF runs as a composable node inside a component container for intra-process comms
- tune/record launch args and timestamped bag output preserved from the original sim launch
"""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Builds a launch description for state-estimator / controller integration testing in sim."""
    sim_share_dir = Path(get_package_share_directory("tauv_sim"))
    sim_param_file = sim_share_dir / "config" / "params.yaml"
    trajectory_file = sim_share_dir / "config" / "trajectories" / "osprey_square.yaml"

    # NOTE: main.launch.py sources the EKF config from tauv_state_estimation, not tauv_core.
    # If this package hasn't been renamed on your branch, swap this back to tauv_core.
    common_share_dir = Path(get_package_share_directory("tauv_state_estimation"))
    common_ekf_file = common_share_dir / "config" / "ekfFUNNY.yaml"

    # Timestamped bag output so each sim run gets an isolated recording.
    timestamp = datetime.now().strftime('%Y.%m.%d_%H.%M.%S')
    bag_name = f"sim_{timestamp}"
    common_ekf_record_file = Path("/tauv-mono/ros_ws") / "bags" / bag_name
    print(f"Recording EKF data to: {common_ekf_record_file}")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'record', default_value='false', description='Enable rosbag recording'
            ),
            DeclareLaunchArgument(
                'tune', default_value='false', description='Enable autotuning'
            ),

            # tauv_sim is assumed to be the /clock source itself, so it's left off use_sim_time
            # below — if it isn't publishing /clock, this needs use_sim_time: True too.
            Node(
                package="tauv_sim",
                executable="tauv_sim",
                name="tauv_sim",
                parameters=[str(sim_param_file)],
                # arguments=["--kinematic", str(trajectory_file)],
                output="screen",
            ),

            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                parameters=[{'port': 8765, 'address': '0.0.0.0', 'use_sim_time': False}],
                output='screen',
            ),

            # Component container for the EKF, matching main.launch.py's sensor_fusion_container
            # pattern. Sim has no FOG node to co-locate, but keeping the same container/component
            # structure means this launch file stays a drop-in match for the real-robot config.
            ComposableNodeContainer(
                name='sensor_fusion_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[],
                output='screen',
            ),
            TimerAction(
                period=8.0,
                actions=[
                    LogInfo(msg="Loading EKF component into container!!!!!!"),
                    LoadComposableNodes(
                        target_container='sensor_fusion_container',
                        composable_node_descriptions=[
                            ComposableNode(
                                package="robot_localization",
                                plugin="robot_localization::EkfComponent",
                                name="ekf_filter_node",
                                parameters=[str(common_ekf_file), {'use_sim_time': False}],
                                extra_arguments=[{'use_intra_process_comms': True}],
                            )
                        ],
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

            # Simplified sim-frame static transforms (identity offsets), unlike main.launch.py's
            # measured real-robot extrinsics. Kept from the original sim launch on purpose.
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_imu',
                arguments=['0', '0', '0', '0', '0', '0', 'os/base_link', 'imu_xsens_link'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_depth',
                arguments=['0', '0', '0', '0', '0', '0', 'os/base_link', 'depth_link'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_dvl',
                arguments=['0', '0', '0', '0', '0', '0', 'os/base_link', 'dvl_link'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            ),

            Node(
                package='tauv_controller',
                executable='controller',
                name='controller',
                parameters=[{'tune': LaunchConfiguration('tune'), 'use_sim_time': False}],
                output='screen',
            ),
            Node(
                package='tauv_controller',
                executable='thruster_forces',
                name='thruster_forces',
                parameters=[{'use_sim_time': False}],
                output='screen',
            ),
            Node(
                package='tauv_controller',
                executable='thruster_rpms',
                name='thruster_rpms',
                parameters=[{'use_sim_time': False}],
                output='screen',
            ),

            # Pulled in from main.launch.py since they're hardware-agnostic and useful for
            # exercising the full nav stack against the sim's kinematic trajectories.
            Node(
                package='tauv_trajectory',
                executable='trajectory_planner',
                name='trajectory_planner',
                parameters=[{'use_sim_time': False}],
                output='screen',
            ),
        ]
    )