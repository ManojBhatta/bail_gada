# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():
#     M_PI	=	3.14159265358979323846
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'input_topic',
#             default_value='/livox/lidar/pcd2',
#             description='PointCloud input topic'
#         ),

#         Node(
#             package='pointcloud_to_laserscan',
#             executable='pointcloud_to_laserscan_node',
#             parameters=[{
#                 # 'target_frame': 'laser',
#                 'input_topic': LaunchConfiguration('input_topic'),  # Referencing launch argument
#                 'transform_tolerance': 0.01,
#                 'min_height': -0.20,
#                 'max_height': 0.2,
#                 'angle_min': -M_PI/2,
#                 'angle_max':  M_PI/2,  # M_PI/2
#                 'angle_increment': 0.0087,  # M_PI/360.0
#                 'scan_time': 0.1,
#                 'range_min': 0.2,
#                 'range_max': 30.0,
#                 'use_inf': True,
#                 'inf_epsilon': 1.0
#             }],
#             name='pointcloud_to_laserscan'
#         )
#     ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pi = 3.141592653589793
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', 
            default_value='/livox/lidar/pcd2', 
            description='PointCloud input topic'),
        DeclareLaunchArgument('angle_min', 
            default_value='-1.570796327',  # -pi/2 as double
            description='Minimum scan angle'),
        DeclareLaunchArgument('angle_max', 
            default_value='1.570796327',   # pi/2 as double
            description='Maximum scan angle'),
        DeclareLaunchArgument('range_min', 
            default_value='0.3',   # pi/2 as double
            description='Minimum laser range'),
        
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'transform_tolerance': 0.01,
                'min_height': -0.1,
                'max_height': 0.20,
                'angle_min': LaunchConfiguration('angle_min'),
                'angle_max': LaunchConfiguration('angle_max'),
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': LaunchConfiguration('range_min'),
                'range_max': 40.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])