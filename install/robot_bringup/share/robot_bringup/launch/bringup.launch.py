import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # LIDAR MSG Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')
            ])
        ),

        # Fast LIO Mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
            ]),
            launch_arguments={'config_file': 'mid360.yaml'}.items()
        ),

        # Custom Message to PointCloud
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('livox_to_pointcloud2'), 'launch', 'livox_to_pointcloud2.launch.py')
            ])
        ),

        # PointCloud to Laser Scan
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'launch', 'pointcloud_to_laserscan_node.launch.py')
            ]),
            launch_arguments={'input_topic':'livox/lidar/pcd2', 
                              'angle_min':'-3.14159',
                              'angle_max':'3.14159',
                              'range_min':'1.0'}.items() 
        ),

        # Transformations
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('transform_mid360'), 'launch', 'tf_setup_mid360_launch.py')
            ])
        ),
        
        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'scan_topic': '/scan',
                'odom_topic': '/Odometry',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'map_frame': 'map',
                'use_sim_time': 'false'
            }.items()
        ),

        # RViz2 Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/home/manoj/ros/robot_ws/src/robot_bringup/rviz/mapping.rviz'],
            output='screen'
        ),
    ])
