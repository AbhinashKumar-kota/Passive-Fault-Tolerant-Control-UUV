"""FTC control stack launch — trajectory, estimator, controller, allocator, and logger."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='BLUEROV2',
        description='Name of the AUV robot (used for topic namespacing)'
    )

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='bluerov2',
        description='Robot type: girona500 or bluerov2 (affects dynamics model and allocation)'
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

    use_ground_truth_arg = DeclareLaunchArgument(
        'use_ground_truth',
        default_value='true',
        description='Use ground truth state from simulator (true) or sensor fusion (false)'
    )

    robot_name = LaunchConfiguration('robot_name')
    robot_type = LaunchConfiguration('robot_type')
    controller_type = LaunchConfiguration('controller_type')
    trajectory_type = LaunchConfiguration('trajectory_type')
    use_ground_truth = LaunchConfiguration('use_ground_truth')

    pkg_share = FindPackageShare('ftc_control')

    smc_config = PathJoinSubstitution([pkg_share, 'config', 'smc.yaml'])
    faults_config = PathJoinSubstitution([pkg_share, 'config', 'faults.yaml'])

    trajectory_node = Node(
        package='ftc_control',
        executable='trajectory_node',
        name='trajectory_node',
        output='screen',
        parameters=[{
            'rate': 20.0,
            'trajectory_type': trajectory_type,
        }]
    )

    state_estimator_node = Node(
        package='ftc_control',
        executable='state_estimator_node',
        name='state_estimator_node',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'robot_type': robot_type,
            'rate': 50.0,
            'use_ground_truth': use_ground_truth,
        }]
    )

    controller_node = Node(
        package='ftc_control',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[
            {
                'rate': 50.0,
                'controller_type': controller_type,
            },
            smc_config,
        ]
    )

    thruster_allocator_node = Node(
        package='ftc_control',
        executable='thruster_allocator_node',
        name='thruster_allocator_node',
        output='screen',
        parameters=[
            {
                'robot_name': robot_name,
                'robot_type': robot_type,
                'rate': 50.0,
                'max_thrust': 100.0,
                'min_thrust': -100.0,
            },
            faults_config,
        ]
    )

    data_logger_node = Node(
        package='ftc_control',
        executable='data_logger_node',
        name='data_logger_node',
        output='screen',
        parameters=[{
            'log_directory': '/tmp/ftc_logs',
            'log_prefix': 'ftc_experiment',
            'log_rate': 10.0,
        }]
    )

    control_stack = GroupAction([
        trajectory_node,
        state_estimator_node,
        controller_node,
        thruster_allocator_node,
        data_logger_node,
    ])

    return LaunchDescription([
        robot_name_arg,
        robot_type_arg,
        controller_type_arg,
        trajectory_type_arg,
        use_ground_truth_arg,
        control_stack,
    ])
