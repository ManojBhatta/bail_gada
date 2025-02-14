from launch import LaunchDescription
from launch_ros.actions import Node
## Somewhere higher-up...


#### tf2 static transforms
def generate_launch_description():
    return LaunchDescription([

    Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name='static_tf_pub_odom_2_camera_init',
        arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'odom', 'camera_init'],
        output='screen'
    ),

    Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_pub_body_to_odom',
        arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'body', 'base_footprint'],
        output='screen'

    ),
    Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name='static_tf_pub_base_footprint_to_livox_frame',
        arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'livox_frame'],
        output='screen'
    ),
    Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_pub_body_to_odom',
        arguments=['0.0', '0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link'],
        output='screen'
    )
    ]
)
