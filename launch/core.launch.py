from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction

from launch.conditions import IfCondition

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='ugv1',
            description='Top-level namespace.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare('palletron_bringup'),
                                                'params', 'default.yaml']),
            description='Full path to the file with the all parameters.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('aws_robomaker_small_warehouse_world'),
                 'maps', '005', 'map.yaml']),
            description='Full path to the map yaml file.'))

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
            'use_slam',
            default_value='False',
            description='Whether to map and localization instead of just localization.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Automatically start the localization stack.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level.'))

    # Initialize arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')
    use_slam = LaunchConfiguration('use_slam')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    bringup_group_action = GroupAction(
        actions=[
            PushRosNamespace(namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('palletron_bringup'),
                         'launch', 'ira_laserscan_multi_merger.launch.py']
                    )
                ),
                launch_arguments={'namespace': namespace,
                                  'params_file': params_file,
                                  'use_sim_time': use_sim_time,
                                  'use_respawn': use_respawn,
                                  'autostart': autostart,
                                  'log_level': log_level}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('palletron_bringup'), 'launch', 'slam.launch.py']
                    )
                ),
                condition=IfCondition(use_slam),
                launch_arguments={'namespace': namespace,
                                  'params_file': params_file,
                                  'use_sim_time': use_sim_time,
                                  'use_respawn': use_respawn,
                                  'autostart': autostart,
                                  'log_level': log_level}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('palletron_bringup'), 'launch', 'localization.launch.py']
                    )
                ),
                condition=IfCondition(PythonExpression(['not ', use_slam])),
                launch_arguments={'namespace': namespace,
                                  'params_file': params_file,
                                  'map_yaml_file': map_yaml_file,
                                  'use_sim_time': use_sim_time,
                                  'use_respawn': use_respawn,
                                  'autostart': autostart,
                                  'log_level': log_level}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('palletron_bringup'), 'launch', 'navigation.launch.py']
                    )
                ),
                condition=IfCondition(PythonExpression(['not ', use_slam])),
                launch_arguments={'namespace': namespace,
                                  'params_file': params_file,
                                  'use_sim_time': use_sim_time,
                                  'use_respawn': use_respawn,
                                  'autostart': autostart,
                                  'log_level': log_level}.items())
        ]
    )

    return LaunchDescription(declared_arguments + [bringup_group_action])
