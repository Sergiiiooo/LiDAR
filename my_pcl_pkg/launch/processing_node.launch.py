from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',  
            parameters=[{'frame_id': 'world_frame', 'child_frame_id': 'world'}], 
            arguments=['0', '0', '0', '0', '0', '0', 'world_frame', 'world'],  
        ),
#        Node(
#            package='tf2_ros',
#            executable='static_transform_publisher',
#            name='static_transform_publisher',
#            output='screen',  
#            parameters=[{'frame_id': 'map', 'child_frame_id': 'world_frame'}], 
#            arguments=['1', '1', '0', '0', '0', '0', 'map', 'world_frame'],  
#        ),
        Node(
            package='my_pcl_pkg',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[
                {"cloud_topic": "/cloud_fullframe"},
                {"world_frame": "world_frame"},
                {"camera_frame": "world"},
            ]
        )
     ])
