"""
thruster_test.launch.py

Launch the Osprey sim in dynamic (physics) mode for manual thruster command testing.
Unlike state_estimator.launch.py, this:
  - Runs Stonefish without --kinematic (so thruster RPM commands affect the vehicle)
  - Starts the EKF immediately (no 8s delay)
  - Starts the ThrusterController from tauv_autonomy (via osprey.launch.py)

Usage:
  ros2 launch tauv_sim thruster_test.launch.py [record:=true] [teleop:=true]

Joystick/teleop via Foxglove (requires teleop:=true):
  1. In Foxglove, add a Teleop panel
  2. Set topic = cmd_vel, message type = geometry_msgs/Twist
  3. Use the on-screen joystick or keyboard arrows to drive

Manual wrench (without teleop):
  ros2 topic pub /cmd_wrench geometry_msgs/msg/WrenchStamped \\
    "{wrench: {force: {x: 1.0, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}}"

Key Foxglove topics:
  /cmd_vel             - Twist from Foxglove Teleop panel (teleop mode)
  /cmd_wrench          - WrenchStamped sent to ThrusterController
  /thruster_rpm        - per-thruster RPM commands
  /esc_telemetry       - simulated ESC feedback (RPM + voltage)
  /odometry/filtered   - EKF fused pose/velocity
  /tf                  - vehicle pose transform
"""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share_dir = Path(get_package_share_directory('tauv_sim'))
    sim_param_file = sim_share_dir / 'config' / 'params.yaml'

    common_share_dir = Path(get_package_share_directory('tauv_core'))
    common_ekf_file = common_share_dir / 'config' / 'ekfFUNNY.yaml'

    autonomy_share_dir = Path(get_package_share_directory('tauv_autonomy'))
    osprey_launch_file = autonomy_share_dir / 'Osprey' / 'launch' / 'osprey.launch.py'

    timestamp = datetime.now().strftime('%Y.%m.%d_%H.%M.%S')
    bag_name = f'thruster_test_{timestamp}'
    bag_output = Path('/tauv-mono/ros_ws') / 'bags' / bag_name

    return LaunchDescription([
        DeclareLaunchArgument(
            'record', default_value='false', description='Enable rosbag recording'
        ),
        DeclareLaunchArgument(
            'teleop', default_value='false',
            description='Start twist_to_wrench bridge for Foxglove Teleop panel (cmd_vel -> cmd_wrench)'
        ),
        DeclareLaunchArgument(
            'play', default_value='false',
            description='Run play_commands sequence automatically after startup (cmd_wrench scripted steps)'
        ),

        # Stonefish in dynamic (physics) mode — thrusters actually move the vehicle.
        # No --kinematic flag, no trajectory file.
        Node(
            package='tauv_sim',
            executable='tauv_sim',
            name='tauv_sim',
            parameters=[str(sim_param_file)],
            arguments=['--no-cameras'],
            output='screen',
        ),

        # robot_state_publisher + ThrusterController (from tauv_autonomy)
        # sim:=true converts NED cmd_wrench -> ENU before TAM (controller outputs NED)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(osprey_launch_file)),
            launch_arguments={'sim': 'true'}.items(),
        ),

        # EKF starts immediately (no delay needed for dynamic mode testing)
        LogInfo(msg='Starting EKF filter node for thruster test...'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[str(common_ekf_file)],
            output='screen',
        ),

        # Optional: scripted wrench command sequence (play_commands.py)
        # Enable with: play:=true
        Node(
            condition=IfCondition(LaunchConfiguration('play')),
            package='tauv_autonomy',
            executable='play_commands.py',
            name='play_commands',
            parameters=[{'startup_delay': 3.0}],
            output='screen',
        ),

        # Optional: Foxglove Teleop bridge (cmd_vel Twist -> cmd_wrench WrenchStamped)
        # Enable with: teleop:=true
        # In Foxglove: add Teleop panel, topic=cmd_vel, type=geometry_msgs/Twist
        Node(
            condition=IfCondition(LaunchConfiguration('teleop')),
            package='tauv_autonomy',
            executable='twist_to_wrench.py',
            name='twist_to_wrench',
            parameters=[{
                'max_force_xy':   5.0,   # N  — forward / strafe
                'max_force_z':    3.0,   # N  — heave
                'max_torque_rp':  2.0,   # N·m — roll / pitch
                'max_torque_yaw': 2.0,   # N·m — yaw
            }],
            output='screen',
        ),

        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record')),
            cmd=[
                'ros2', 'bag', 'record',
                '-a',
                '-s', 'mcap',
                '--polling-interval', '1',
                '-x', '/os/sensors/cam.*',
                '-o', str(bag_output),
            ],
            output='screen',
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
            }],
        ),
    ])
