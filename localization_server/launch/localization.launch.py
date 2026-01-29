import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    map_file_name = LaunchConfiguration('map_file').perform(context)
    
    # Construct full map file path
    map_file = os.path.join(
        get_package_share_directory('map_server'), 'config', map_file_name
    )

    # Decide sim or real
    use_sim_time = (map_file_name == 'sim_warehouse_map_save.yaml') or (map_file_name == 'warehouse_map_keepout_sim.yaml')
    amcl_yaml_file = 'amcl_config_sim.yaml' if use_sim_time else 'amcl_config_real.yaml'
    rviz_config_file = '/home/user/ros2_ws/src/warehouse_project/localization_server/rviz/localization_display.rviz' if use_sim_time else '/home/user/ros2_ws/src/warehouse_project/localization_server/rviz/localization_display_real.rviz'
    # Construct full nav2_yaml path
    nav2_yaml = os.path.join(
        get_package_share_directory('localization_server'), 'config', amcl_yaml_file
    )

    #rviz_config_file = '/home/user/ros2_ws/src/warehouse_project/localization_server/rviz/localization_display.rviz'

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_file}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
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
            default_value='warehouse_map_sim.yaml',
            description='Map YAML file name (sim or real)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
