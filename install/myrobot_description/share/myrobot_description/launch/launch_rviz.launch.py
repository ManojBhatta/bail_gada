from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to your xacro file
    pkg_share_path = get_package_share_directory('myrobot_description')
    xacro_path = os.path.join(pkg_share_path, 'urdf', 'myrobot.urdf.xacro')

    rviz_config_path = os.path.join(pkg_share_path, 'config', 'robot_display.rviz')
    
    # Process the URDF via xacro and store the .urdf as a string in parameter robot_description
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    # Create a robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path, '--ros-args', '-p', 'use_sim_time:=True']
    )
    # Return launch description
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
