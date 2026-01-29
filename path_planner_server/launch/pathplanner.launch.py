import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    config_suffix = 'sim.yaml' if use_sim_time else 'real.yaml'
    pkg_dir = get_package_share_directory('path_planner_server')
    
    controller_yaml = os.path.join(pkg_dir, 'config', f'controller_{config_suffix}')
    #print(controller_yaml)
    bt_navigator_yaml = os.path.join(pkg_dir, 'config', f'bt_navigator_{config_suffix}')
    #print(bt_navigator_yaml)
    planner_yaml = os.path.join(pkg_dir, 'config', f'planner_{config_suffix}')
    #print(planner_yaml)
    recovery_yaml = os.path.join(pkg_dir, 'config', f'recoveries_{config_suffix}')
    #print(recovery_yaml)
    robot_odom = 'odom' if use_sim_time else 'robot_odom'
    cmd_vel_topic = '/diff_controller/cmd_vel' if use_sim_time else '/cmd_vel'
    rviz_config_file = '/home/user/ros2_ws/src/warehouse_project/path_planner_server/rviz/pathplanning.rviz' if use_sim_time else '/home/user/ros2_ws/src/warehouse_project/path_planner_server/rviz/pathplanning_real.rviz'
    #server_node = 'approach_service_server_node' if use_sim_time else 'approach_service_server_node_real'
    #server_node_v2 = 'approach_service_server_node_v2' if use_sim_time else 'approach_service_server_node_real_v2'
    #server_node_2 = 'approach_server_2.py'  if use_sim_time else 'approach_server_2_real.py'
    #client_node = 'move_shelf_to_ship.py'  if use_sim_time else 'move_shelf_to_ship_real.py'
    #detect_cart_action_server = 'detect_cart_action_server' if use_sim_time else 'detect_cart_action_server_real'
    #dock_to_cart_server =  'dock_to_cart_action_server' if use_sim_time else 'dock_to_cart_action_server_real'

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', cmd_vel_topic)]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': use_sim_time}]
            #remappings=[('/cmd_vel', cmd_vel_topic)]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml, {'use_sim_time': use_sim_time, 'robot_base_frame': 'base_link','global_frame': robot_odom,'global_frame': 'map'}],
            remappings=[('/cmd_vel', cmd_vel_topic)],
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', cmd_vel_topic)]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        #Node(
        #    package='path_planner_server',
        #    executable=server_node,
        #    name=server_node,
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}]),
        #Node(
        #    package="nav2_apps",
        #    executable=server_node_2, 
        #    output="screen",
        #    emulate_tty=True
        #),

        #Node(
        #    package="nav2_apps",
        #    executable=client_node, 
        #    output="screen",
        #    emulate_tty=True
        #)
        #Node(
        #    package='action_servers_pkg',
        #    executable=detect_cart_action_server,
        #    name=detect_cart_action_server,
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}]),

        #Node(
        #    package='action_servers_pkg',
        #    executable=dock_to_cart_server,
        #    name=dock_to_cart_server,
        #    output='screen',
        #    parameters=[{'use_sim_time': use_sim_time}]),
        
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),
        OpaqueFunction(function=launch_setup)
    ])
