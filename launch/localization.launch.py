from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, GroupAction

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml, ReplaceString

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value = 'palletron1',
            description = 'Top-level namespace.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value = PathJoinSubstitution([FindPackageShare('palletron_bringup'),
                'params', 'default_params.yaml']),
            description = 'Full path to the file with the all localization parameters.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value = PathJoinSubstitution([FindPackageShare('aws_robomaker_small_warehouse_world'),
                'maps', '005', 'map.yaml']),
            description = 'Full path to the map yaml file.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'True',
            description = 'Whether use simulation (Gazebo) clock.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_respawn',
            default_value = 'False', 
            description = 'Whether to respawn if a node crashes.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'autostart',
            default_value = 'True',
            description = 'Automatically start the localization stack.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level',
            default_value = 'info',
            description = 'Log level.'
        )
    )
    
    # Initialize arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server', 'amcl']

    # Replace '<robot_namespace>' tag with the real robot namespace
    params_file = ReplaceString(
        source_file = params_file,
        replacements = {'<robot_namespace>': namespace}
    )

    # Rewrite the file to have the robot namespace
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    configured_params = RewrittenYaml(
        source_file = params_file,
        root_key = namespace,
        param_rewrites = param_substitutions,
        convert_types = True
    )

    localization_group_action = GroupAction(
        actions = [
            Node(
                package = 'nav2_map_server',
                executable = 'map_server',
                name = 'map_server',
                output = 'screen',
                respawn = use_respawn,
                respawn_delay = 2.0,
                parameters = [configured_params],
                arguments = ['--ros-args', '--log-level', log_level]),
            Node(
                package = 'nav2_amcl',
                executable = 'amcl',
                name = 'amcl',
                output = 'screen',
                respawn = use_respawn,
                respawn_delay = 2.0,
                parameters = [configured_params],
                arguments = ['--ros-args', '--log-level', log_level]),
            Node(
                package = 'nav2_lifecycle_manager',
                executable = 'lifecycle_manager',
                name = 'lifecycle_manager_localization',
                output = 'screen',
                parameters = [{'autostart': autostart,
                               'use_sim_time': use_sim_time,
                               'node_names': lifecycle_nodes}],
                arguments = ['--ros-args', '--log-level', log_level])
        ]
    )

    return LaunchDescription(declared_arguments + [localization_group_action])