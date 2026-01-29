from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    target_camera_id = LaunchConfiguration('target_camera_id')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    return LaunchDescription([
        DeclareLaunchArgument('target_camera_id', description='Target camera ID (Serial Number) to filter'),
        DeclareLaunchArgument('input_topic', default_value='/people/detected_boxes', description='Input topic to subscribe'),
        DeclareLaunchArgument('output_topic', default_value='/people/detected_boxes_filtered', description='Output topic to publish'),

        Node(
            package='track_people_py',
            executable='filter_tracked_boxes.py',
            name='filter_tracked_boxes',
            output='screen',
            parameters=[{
                'target_camera_id': target_camera_id,
                'input_topic': input_topic,
                'output_topic': output_topic
            }]
        )
    ])