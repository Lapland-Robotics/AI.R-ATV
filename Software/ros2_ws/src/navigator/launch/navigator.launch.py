import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav_param_file_name = 'snower.yaml'
    slam_param_file_name = 'slamtoolbox_online_async_params.yaml.yaml'
    nav_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    slam_toolbox_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')
    map_dir = LaunchConfiguration(
        'map', default=os.path.join('ros2_ws/src/navigator', 'maps', 'construction_lab.yaml'))
    nav_param_dir = LaunchConfiguration(
        'params_file', default=os.path.join('ros2_ws/src/navigator','param', nav_param_file_name))
    slam_param_dir = LaunchConfiguration(
        'params_file', default=os.path.join('ros2_ws/src/navigator','param', slam_param_file_name))


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=nav_param_dir,
            description='Full path to param file to load'),
    
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time true if using a simulator (Gazebo)'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_toolbox_file_dir, '/online_async_launch.py']),
            launch_arguments={
                'params_file': slam_param_dir,
            }.items(),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav_param_dir,
            }.items(),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav_launch_file_dir, '/bringup_launch.py']),
        #     launch_arguments={
        #         'use_sim_time': use_sim_time,
        #         'map': map_dir,
        #         'params_file': nav_param_dir,
        #   }.items(),
        # ),
    ])
