from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource

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
            description='Whether to respawn if a node crashes.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Automatically start the navigation stack.'))

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
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_saver']

    # Replace '<robot_namespace>' tag with the real robot namespace
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': namespace})

    # Rewrite the file to have the robot namespace
    param_substitutions = {'use_sim_time': use_sim_time}

    configured_params_file = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    map_server_group_action = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params_file],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[{'autostart': autostart,
                             'use_sim_time': use_sim_time,
                             'node_names': lifecycle_nodes}],
                arguments=['--ros-args', '--log-level', log_level])
        ]
    )

    slam_toolbox_launcher = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('slam_toolbox'), 'launch', 'online_sync_launch.py'])),
                launch_arguments={'slam_params_file': configured_params_file}.items())
        ]
    )

    return LaunchDescription(declared_arguments + [map_server_group_action, slam_toolbox_launcher])
