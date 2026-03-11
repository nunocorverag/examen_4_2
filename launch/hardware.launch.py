"""
PuzzleBot Hardware Launch
──────────────────────────
Lanza los controladores + motor_interface + dashboard
SIN Gazebo — para usar con el ESP32 real.

Usage:
  ros2 launch puzzlebot_control hardware.launch.py
  ros2 launch puzzlebot_control hardware.launch.py mode:=position
  ros2 launch puzzlebot_control hardware.launch.py wheel:=right_wheel_joint

Luego en otra terminal:
  ros2 run puzzlebot_control teleop_keyboard

O manda un goal directo:
  ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 0.0}}}" --once
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Argumentos ────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='velocity',
        description='Modo del motor: velocity o position')

    wheel_arg = DeclareLaunchArgument(
        'wheel', default_value='left_wheel_joint',
        description='Nombre del joint: left_wheel_joint o right_wheel_joint')

    # ── Motor Interface ───────────────────────────────────────────────
    motor_interface_node = Node(
        package='puzzlebot_control',
        executable='motor_interface',
        name='motor_interface',
        output='screen',
        parameters=[{
            'wheel_name': LaunchConfiguration('wheel'),
            'omega_max':  9.15,
            'mode':       LaunchConfiguration('mode'),
            'filter_alpha': 0.85,
        }],
    )

    # ── micro-ROS Agent (serial) ──────────────────────────────────────
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
    )

    # ── Controladores (mismos que en Gazebo) ──────────────────────────
    common = {
        'goal_x': 2.0, 'goal_y': 0.0,
        'max_linear_vel': 0.5, 'max_angular_vel': 3.0,
        'control_rate': 50.0, 'goal_tolerance': 0.05,
    }

    pid_node = Node(
        package='puzzlebot_control',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[common],
    )

    smc_node = Node(
        package='puzzlebot_control',
        executable='smc_controller',
        name='smc_controller',
        output='screen',
        parameters=[{**common, 'default_active': False}],
    )

    ismc_node = Node(
        package='puzzlebot_control',
        executable='ismc_controller',
        name='ismc_controller',
        output='screen',
        parameters=[{**common, 'default_active': False}],
    )

    ctc_node = Node(
        package='puzzlebot_control',
        executable='ctc_controller',
        name='ctc_controller',
        output='screen',
        parameters=[{**common, 'default_active': False}],
    )

    ph_node = Node(
        package='puzzlebot_control',
        executable='ph_controller',
        name='ph_controller',
        output='screen',
        parameters=[{**common, 'default_active': False}],
    )

    # ── Terrain perturbation (opcional en hardware) ───────────────────
    terrain_node = Node(
        package='puzzlebot_control',
        executable='terrain_perturb',
        name='terrain_perturbation',
        output='screen',
        parameters=[{
            'enabled': False,   # desactivado por default en hardware
            'type': 'mixed',
            'amplitude_v': 0.03,
            'amplitude_w': 0.06,
        }],
    )

    # ── Dashboard ─────────────────────────────────────────────────────
    dashboard_node = Node(
        package='puzzlebot_control',
        executable='dashboard',
        name='dashboard',
        output='screen',
        parameters=[{'port': 8080}],
    )

    # ── Launch description ────────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(mode_arg)
    ld.add_action(wheel_arg)

    ld.add_action(motor_interface_node)
    ld.add_action(microros_agent)

    ld.add_action(pid_node)
    ld.add_action(smc_node)
    ld.add_action(ismc_node)
    ld.add_action(ctc_node)
    ld.add_action(ph_node)

    ld.add_action(terrain_node)
    ld.add_action(dashboard_node)

    return ld
