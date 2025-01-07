import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file robot.urdf.xacro 
    xacro_file = os.path.join('ros2_ws/src/atv', 'urdf','snower.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        Node(
            package='atv',
            executable='atv',
            name='atv',
            output='screen'
        ),
        # Node(
        #     package='atv',
        #     executable='data_collector',
        #     name='data_collector',
        #     output='screen'
        # ),

        # Node(
        #     package='atv',
        #     executable='odom_publisher',
        #     name='odom_publisher',
        #     output='screen',
        #     parameters=[
        #         {'odom_frame': 'odom'},
        #         {'base_frame': 'base_link'}
        #     ]
        # ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        node_robot_state_publisher,

        # Node(
        #     package='atv',
        #     executable='odom_to_baselink_tf',
        #     name='odom_to_baselink_tf',
        #     output='screen'
        # ),
        # Node(
        #     package='atv',
        #     executable='map_to_odom_tf',
        #     name='map_to_odom_tf',
        #     output='screen'
        # ),
    ])
