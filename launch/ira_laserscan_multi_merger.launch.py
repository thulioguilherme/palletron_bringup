from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, GroupAction

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='palletron1',
            description='Top-level namespace.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare('palletron_bringup'),
                                                'params', 'default.yaml']),
            description='Full path to the file with the all localization parameters.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Whether use simulation (Gazebo) clock.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description=('Whether to respawn if a node crashes.')))

    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level.'))

    # Initialize arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    # Replace '<robot_namespace>' tag with the real robot namespace
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': namespace})

    # Rewrite the file to have the robot namespace
    configured_params_file = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)

    laserscan_multi_merger_group_action = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='ira_laser_tools',
                executable='laserscan_multi_merger',
                name='laserscan_multi_merger',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params_file],
                arguments=['--ros-args', '--log-level', log_level])
        ]
    )

    return LaunchDescription(declared_arguments + [laserscan_multi_merger_group_action])
