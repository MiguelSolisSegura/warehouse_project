import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    map_file = context.launch_configurations['map_file']

    # Default values for map and RViz config directories
    map_path = os.path.join(get_package_share_directory('map_server'), 'config', map_file)

    if map_file == 'warehouse_map_real.yaml':
        rviz_config_dir = os.path.join(get_package_share_directory('map_server'), 'config', 'map_server.rviz')
    else:
        rviz_config_dir = os.path.join(get_package_share_directory('map_server'), 'config', 'sim_map_server.rviz')

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_path} 
                       ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='warehouse_map_real.yaml',
            description='Type of map to use.'),
        OpaqueFunction(function=launch_setup)
    ])