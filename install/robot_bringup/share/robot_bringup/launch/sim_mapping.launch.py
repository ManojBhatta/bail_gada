from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os

def generate_launch_description():
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('myrobot_description'),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    gazebo = IncludeLaunchDescription(        
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': os.path.join(
            FindPackageShare('robot_bringup').find('robot_bringup'),
            'worlds',
            'sim_test_world.world'
        )}.items()
    )
    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', "robot_description",
                                   '-entity', 'my_bot'],
                        output='screen')

    
    pc_to_ls = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'launch', 'pointcloud_to_laserscan_node.launch.py')  # Changethe launch file name
            ]),
            launch_arguments={'input_topic':'livox/lidar/pcd2', 
                              'angle_min':'-3.14159',
                              'angle_max':'3.14159',
                              'range_min':'1.0'}.items()        
        )
    
    pc_to_livox = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('livox_to_pointcloud2'), 'launch', 'pointcloud2_to_livox.launch.py')  # Changethe launch file name
            ])
        )
    fast_lio = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
            ]),
            launch_arguments={'config_file': 'mid360.yaml',
                              'use_sim_time':'true'}.items()
        )
    odom_to_camera = Node(
                package='tf2_ros', executable='static_transform_publisher',
                name='static_tf_odom_to_camera_init',
                arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'odom', 'camera_init'],
                output='screen'
    )
    body_to_base = Node(
                package='tf2_ros', executable='static_transform_publisher',
                name='static_tf_body_to_baselink',
                arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'body', 'base_link'],
                output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('robot_bringup'), 'rviz', 'robot_disp.rviz'), '--ros-args', '-p', 'use_sim_time:=True']
    )
    
    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'scan_topic': '/scan',
                'odom_topic': '/Odometry',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map', 
                'use_sim_time': 'true'
            }.items()
        )
    
    
    return LaunchDescription([
        gazebo,
        rsp,
        spawn_entity,
        pc_to_ls,
        pc_to_livox,
        fast_lio,
        odom_to_camera,
        body_to_base,
        rviz_node,
        slam
    ])
