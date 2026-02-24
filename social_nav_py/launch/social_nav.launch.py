from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Check if simulation
    is_sim = os.environ.get('CABOT_GAZEBO', '0') == '1'
    
    default_image_topic = '/rs1/color/image_raw'
    if is_sim:
        default_image_topic = '/camera/color/image_raw'

    input_image_topic_arg = DeclareLaunchArgument(
        'input_image_topic', default_value=default_image_topic
    )
    people_topic_arg = DeclareLaunchArgument(
        'people_topic', default_value='/people'
    )
    view_image_arg = DeclareLaunchArgument(
        'view_image', default_value='True'
    )

    # Social Nav GPT Node
    social_nav = Node(
        package='social_nav_py',
        executable='social_nav.py',
        name='social_nav',
        parameters=[{
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'people_topic': LaunchConfiguration('people_topic'),
            'view_image': LaunchConfiguration('view_image'),
        }],
        output='screen'
    )

    return LaunchDescription([
        input_image_topic_arg,
        people_topic_arg,
        view_image_arg,
        social_nav
    ])
