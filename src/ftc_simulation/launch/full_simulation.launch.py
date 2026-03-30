"""Full FTC simulation launch — Stonefish simulator + control stack with delayed start."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='BLUEROV2',
        description='Name of the AUV robot (used for topic namespacing)'
    )

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='bluerov2',
        description='Robot type: girona500 or bluerov2 (affects dynamics and allocation)'
    )

    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='ftsmc',
        description='Controller type: ftsmc, i_ftsmc'
    )

    trajectory_type_arg = DeclareLaunchArgument(
        'trajectory_type',
        default_value='hover',
        description='Trajectory type: hover, eight, lawnmower, waypoint'
    )

    window_width_arg = DeclareLaunchArgument(
        'window_width',
        default_value='1920',
        description='Simulator window width in pixels (0 for headless)'
    )

    window_height_arg = DeclareLaunchArgument(
        'window_height',
        default_value='1080',
        description='Simulator window height in pixels (0 for headless)'
    )

    simulation_data_arg = DeclareLaunchArgument(
        'simulation_data',
        default_value='',
        description='Path to simulation data directory (auto-detected if empty)'
    )

    robot_name = LaunchConfiguration('robot_name')
    robot_type = LaunchConfiguration('robot_type')
    controller_type = LaunchConfiguration('controller_type')
    trajectory_type = LaunchConfiguration('trajectory_type')
    window_width = LaunchConfiguration('window_width')
    window_height = LaunchConfiguration('window_height')

    ftc_sim_share = FindPackageShare('ftc_simulation')
    ftc_control_share = FindPackageShare('ftc_control')
    stonefish_ros2_share = FindPackageShare('stonefish_ros2')

    simulation_data = PathJoinSubstitution([ftc_sim_share, 'data'])
    scenario_file = PathJoinSubstitution([ftc_sim_share, 'scenarios', 'experiment_world.scn'])

    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([stonefish_ros2_share, 'launch', 'stonefish_simulator.launch.py'])
        ]),
        launch_arguments={
            'simulation_data': simulation_data,
            'scenario_desc': scenario_file,
            'simulation_rate': '300',
            'window_res_x': window_width,
            'window_res_y': window_height,
        }.items()
    )

    # Delay control stack 3s to let simulator initialize
    control_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([ftc_control_share, 'launch', 'control_stack.launch.py'])
                ]),
                launch_arguments={
                    'robot_name': robot_name,
                    'robot_type': robot_type,
                    'controller_type': controller_type,
                    'trajectory_type': trajectory_type,
                    'use_ground_truth': 'true',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        robot_type_arg,
        controller_type_arg,
        trajectory_type_arg,
        window_width_arg,
        window_height_arg,
        simulation_data_arg,
        simulator_launch,
        control_launch,
    ])
