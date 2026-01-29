import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    #pkg_share = get_package_share_directory('map_server')
    #default_sim_map = os.path.join(pkg_share, 'config', 'warehouse_map_sim.yaml')
    #default_real_map = os.path.join(pkg_share, 'config', 'warehouse_map_real.yaml')

    # Get values from launch arguments
    map_file_name = LaunchConfiguration('map_file').perform(context)

    map_file_name = os.path.join(
        get_package_share_directory('map_server'), 'config', map_file_name
    )

    #print(map_file_name)

    use_sim_time = map_file_name == '/home/user/ros2_ws/install/map_server/share/map_server/config/warehouse_map_sim.yaml'

    #print(use_sim_time)

    rviz_config_file = '/home/user/ros2_ws/src/warehouse_project/map_server/rviz/map_display.rviz' if use_sim_time else '/home/user/ros2_ws/src/warehouse_project/map_server/rviz/map_display_real.rviz'

    #print(rviz_config_file)

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file_name
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value= 'warehouse_map_sim.yaml',
            description='Map YAML file name (sim or real)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
