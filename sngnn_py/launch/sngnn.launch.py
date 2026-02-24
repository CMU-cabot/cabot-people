from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_name = 'sngnn_py'
    
    goal_topic_arg = DeclareLaunchArgument(
        'goal_topic', default_value='/plan',
        description='Goal topic'
    )
    map_topic_arg = DeclareLaunchArgument(
        'map_topic', default_value='/map',
        description='Map topic'
    )
    people_topic_arg = DeclareLaunchArgument(
        'people_topic', default_value='/people',
        description='People topic'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Odom topic'
    )

    return LaunchDescription([
        goal_topic_arg,
        map_topic_arg,
        people_topic_arg,
        odom_topic_arg,
        
        Node(
            package=pkg_name,
            executable='sngnn_node.py',
            name='sngnn_node',
            output='screen',
            parameters=[{
                'goal_topic': LaunchConfiguration('goal_topic'),
                'map_topic': LaunchConfiguration('map_topic'),
                'save_image': False
            }],
            remappings=[
                ('/people', LaunchConfiguration('people_topic')),
                ('/odom', LaunchConfiguration('odom_topic'))
            ]
        )
    ])
