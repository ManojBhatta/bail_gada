from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="livox_to_pointcloud2",
            executable="pointcloud2_to_livox_node",
            name="pointcloud2_to_livox_node",
            remappings=[
                ("input_pointcloud2", "/livox/lidar/pcd2"),
                ("livox_custom_msg", "/livox/lidar"),
            ]
        )
    ])
