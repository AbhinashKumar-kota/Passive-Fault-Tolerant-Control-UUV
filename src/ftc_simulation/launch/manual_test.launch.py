"""Manual test launch — simulator, estimator, trajectory, allocator, logger, and manual test node."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='BLUEROV2', description='Name of the AUV robot'
    )

    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='bluerov2', description='Robot type: girona500 or bluerov2'
    )

    controller_type_arg = DeclareLaunchArgument(
        'controller_type', default_value='backstepping', description='Controller: backstepping, ftsmc, or i_ftsmc'
    )

    traj_type_arg = DeclareLaunchArgument(
        'trajectory_type', default_value='eight', description='hover, eight, waypoint'
    )

    log_file_arg = DeclareLaunchArgument(
        'log_file_name', default_value='ftc_mission_log.csv', description='Name of the output CSV log'
    )

    robot_name = LaunchConfiguration('robot_name')
    robot_type = LaunchConfiguration('robot_type')
    controller_type = LaunchConfiguration('controller_type')
    trajectory_type = LaunchConfiguration('trajectory_type')
    log_file_name = LaunchConfiguration('log_file_name')

    ftc_sim_share = FindPackageShare('ftc_simulation')
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
        }.items()
    )

    state_estimator_node = Node(
        package='ftc_control',
        executable='state_estimator_node',
        name='state_estimator_node',
        parameters=[{
            'robot_name': robot_name,
            'robot_type': robot_type,
            'use_ground_truth': True
        }]
    )

    trajectory_node = Node(
        package='ftc_control',
        executable='trajectory_node',
        name='trajectory_node',
        parameters=[{
            'trajectory_type': trajectory_type,
            'rate': 20.0
        }]
    )

    thruster_allocator_node = Node(
        package='ftc_control',
        executable='thruster_allocator_node',
        name='thruster_allocator_node',
        parameters=[{
            'robot_name': robot_name,
            'robot_type': robot_type,
            'force_to_rpm_gain': 50.0
        }]
    )

    data_logger_node = Node(
        package='ftc_control',
        executable='data_logger_node',
        name='data_logger_node',
        parameters=[{
            'log_file_name': log_file_name
        }]
    )

    # Delay manual test node 4s for simulator to settle
    manual_test_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ftc_control',
                executable='manual_test_node',
                name='manual_test_node',
                output='screen',
                parameters=[{
                    'vehicle_name': robot_name,
                    'controller_type': controller_type,
                }]
            )
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        robot_type_arg,
        controller_type_arg,
        traj_type_arg,
        log_file_arg,
        simulator_launch,
        state_estimator_node,
        trajectory_node,
        thruster_allocator_node,
        data_logger_node,
        manual_test_node,
    ])
