import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
                              'angle_max':'3.14159'}.items() 
        ),

        # Transformations
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('transform_mid360'), 'launch', 'tf_setup_mid360_launch.py')
            ])
        ),
        #map to odom tf
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_tf_map_to_odom',
            arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
            output='screen'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'params_file': '/home/manoj/ros/robot_ws/src/robot_bringup/params/nav2_params.yaml',
                'use_sim_time': 'false',
                'autostart': 'true',
                'log_level': 'info',
                'map': '/home/manoj/testing/completemap1.yaml'
            }.items()
        ),
        
        # RViz2 Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',    
            arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
        )
    ])
