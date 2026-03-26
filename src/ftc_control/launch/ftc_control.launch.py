"""FTC controller launch — state estimator, trajectory generator, and controller."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    vehicle_name_arg = DeclareLaunchArgument('vehicle_name', default_value='BLUEROV2')
    vehicle_type_arg = DeclareLaunchArgument('vehicle_type', default_value='bluerov2')
    trajectory_type_arg = DeclareLaunchArgument('trajectory_type', default_value='eight')
    controller_type_arg = DeclareLaunchArgument('controller_type', default_value='backstepping')

    state_estimator_node = Node(
        package='ftc_control',
        executable='state_estimator_node',
        name='state_estimator',
        parameters=[{
            'robot_name': LaunchConfiguration('vehicle_name'),
            'robot_type': LaunchConfiguration('vehicle_type'),
            'use_ground_truth': True,
        }]
    )

    trajectory_node = Node(
        package='ftc_control',
        executable='trajectory_node',
        name='trajectory_generator',
        parameters=[{
            'trajectory_type': LaunchConfiguration('trajectory_type'),
            'rate': 20.0,
        }]
    )

    controller_node = Node(
        package='ftc_control',
        executable='controller_node',
        name='controller',
        parameters=[{
            'k1': 1.5,
            'k2': 2.5,
            'pwm_gain': 25.0,
            'controller_type': LaunchConfiguration('controller_type'),
            'vehicle_type': LaunchConfiguration('vehicle_type'),
            'vehicle_name': LaunchConfiguration('vehicle_name'),
        }],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(vehicle_name_arg)
    ld.add_action(vehicle_type_arg)
    ld.add_action(trajectory_type_arg)
    ld.add_action(controller_type_arg)

    ld.add_action(state_estimator_node)
    ld.add_action(trajectory_node)
    ld.add_action(controller_node)

    return ld
