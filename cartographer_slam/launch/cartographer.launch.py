import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_config_dir = os.path.join(
        get_package_share_directory('cartographer_slam'), 'config')

    rviz_config_sim = '/home/user/ros2_ws/src/warehouse_project/cartographer_slam/rviz/mapping.rviz'
    rviz_config_real = '/home/user/ros2_ws/src/warehouse_project/cartographer_slam/rviz/mapping_real.rviz'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Cartographer node for simulation
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', 'cartographer_sim.lua'],
            remappings=[
            ('scan', '/scan_fixed'),
            ('odom', '/odom_fixed'),
            ],
            condition=IfCondition(use_sim_time)
        ),

        # Cartographer node for real robot
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', 'cartographer_real.lua'],
            condition=UnlessCondition(use_sim_time)
        ),

        # Occupancy grid node (common to both)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

        # RViz with simulation config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_sim',
            output='screen',
            arguments=['-d', rviz_config_sim],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_sim_time)
        ),

        # RViz with real config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_real',
            output='screen',
            arguments=['-d', rviz_config_real],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(use_sim_time)
        ),
    ])
