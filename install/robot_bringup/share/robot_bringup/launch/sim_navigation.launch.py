# from launch import LaunchDescription
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import IncludeLaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node

# import os

# def generate_launch_description():
#     rsp = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory('myrobot_description'),'launch','rsp.launch.py'
#                 )]), launch_arguments={'use_sim_time': 'true'}.items()
#     )
#     gazebo = IncludeLaunchDescription(        
#         PythonLaunchDescriptionSource([
#             os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
#         ]),
#         launch_arguments={'world': os.path.join(
#             FindPackageShare('robot_bringup').find('robot_bringup'),
#             'worlds',
#             'sim_test_world.world'
#         )}.items()
#     )
    
#     spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
#                         arguments=['-topic', "robot_description",
#                                    '-entity', 'my_bot'],
#                         output='screen')

#     pc_to_ls = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'launch', 'pointcloud_to_laserscan_node.launch.py')  # Changethe launch file name
#             ]),
#             launch_arguments={'input_topic':'livox/lidar/pcd2'}.items()
#         )
    
#     pc_to_livox = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 os.path.join(get_package_share_directory('livox_to_pointcloud2'), 'launch', 'pointcloud2_to_livox.launch.py')  # Changethe launch file name
#             ])
#         )
#     fast_lio = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
#             ]),
#             launch_arguments={'config_file': 'mid360.yaml',
#                               'use_sim_time':'true'}.items()
#         )
#     odom_to_camera = Node(
#                 package='tf2_ros', executable='static_transform_publisher',
#                 name='static_tf_odom_to_camera_init',
#                 arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'odom', 'camera_init'],
#                 output='screen'
#     )
#     body_to_base = Node(
#                 package='tf2_ros', executable='static_transform_publisher',
#                 name='static_tf_body_to_baselink',
#                 arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'body', 'base_link'],
#                 output='screen'
#     )
#     map_to_odom = Node(
#                 package='tf2_ros', executable='static_transform_publisher',
#                 name='static_tf_map_to_odom',
#                 arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
#                 output='screen'
#     )
    
#     nav = IncludeLaunchDescription(
#             PythonLaunchDescriptionSource([
#                 os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
#             ]),
#             launch_arguments={
#                 'params_file': '/home/manoj/ros/robot_ws/src/robot_bringup/params/nav2_params.yaml',
#                 'use_sim_time': 'true',
#                 'autostart': 'false',
#                 'log_level': 'info',
#                 'map': '/home/manoj/sim_test.yaml'
#             }.items()
#         )
#     rviz_nav = Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
#         )
#     return LaunchDescription([
#         gazebo,
#         rsp,
#         spawn_entity,
#         pc_to_ls,
#         pc_to_livox,
#         fast_lio,
#         odom_to_camera,
#         body_to_base,
#         map_to_odom,
#         nav,
#         rviz_nav
        
        
#     ])








from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Start Gazebo
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

    # Start Robot State Publisher after Gazebo
    rsp = TimerAction(
        period=2.0,  # Delay of 2 seconds
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('myrobot_description'),'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
        )]
    )

    # Spawn entity after Gazebo
    spawn_entity = TimerAction(
        period=3.0,  # Delay of 3 seconds
        actions=[Node(package='gazebo_ros', executable='spawn_entity.py',
                      arguments=['-topic', "robot_description",
                                 '-entity', 'my_bot'],
                      output='screen')]
    )

    # Point cloud to LaserScan & Livox converter
    pc_to_ls = TimerAction(
        period=5.0,  # Delay of 5 seconds
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'launch', 'pointcloud_to_laserscan_node.launch.py')
            ]),
            launch_arguments={'input_topic':'livox/lidar/pcd2',
                             'angle_min':'-3.14159',
                              'angle_max':'3.14159',
                              'range_min':'1.0'}.items()
        )]
    )

    pc_to_livox = TimerAction(
        period=5.0,  # Delay of 5 seconds
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('livox_to_pointcloud2'), 'launch', 'pointcloud2_to_livox.launch.py')
            ])
        )]
    )

    fast_lio = TimerAction(
        period=5.0,  # Delay of 5 seconds
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
            ]),
            launch_arguments={'config_file': 'mid360.yaml',
                              'use_sim_time':'true'}.items()
        )]
    )

    # TF Transformations (after point cloud processing)
    odom_to_camera = TimerAction(
        period=7.0,  # Delay of 7 seconds
        actions=[Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_tf_odom_to_camera_init',
            arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'odom', 'camera_init'],
            output='screen'
        )]
    )

    body_to_base = TimerAction(
        period=7.0,  # Delay of 7 seconds
        actions=[Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_tf_body_to_baselink',
            arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'body', 'base_link'],
            output='screen'
        )]
    )

    map_to_odom = TimerAction(
        period=7.0,  # Delay of 7 seconds
        actions=[Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='static_tf_map_to_odom',
            arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
            output='screen'
        )]
    )

    # Navigation stack after TF is published
    nav = TimerAction(
        period=10.0,  # Delay of 10 seconds
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'params_file': os.path.join(get_package_share_directory('robot_bringup'), 'params', 'sim_nav2_params.yaml'),
                'use_sim_time': 'true',
                'autostart': 'true',
                'log_level': 'info',
                'map': '/home/manoj/sim_test3.yaml'
            }.items()
        )]
    )

    # RViz after Nav2
    rviz_nav = TimerAction(
        period=12.0,  # Delay of 12 seconds
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
        )]
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
        map_to_odom,
        nav,
        rviz_nav
    ])
