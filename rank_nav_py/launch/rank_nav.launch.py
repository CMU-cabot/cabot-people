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
    
    default_camera_info_topic = '/rs1/color/camera_info'
    if is_sim:
        default_camera_info_topic = '/camera/color/camera_info'

    # Declare arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value=default_image_topic,
        description='Topic for RGB image'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value=default_camera_info_topic,
        description='Topic for camera info'
    )
    people_topic_arg = DeclareLaunchArgument(
        'people_topic', default_value='/people',
        description='Topic for people detection'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Topic for odometry'
    )
    camera_serial_arg = DeclareLaunchArgument(
        'camera_serial', default_value='',
        description='Camera serial number for filtering'
    )
    min_depth_arg = DeclareLaunchArgument(
        'min_depth', default_value='0.3',
        description='Minimum depth'
    )
    max_depth_arg = DeclareLaunchArgument(
        'max_depth', default_value='40.0',
        description='Maximum depth'
    )
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda:0',
        description='Compute device'
    )
    publish_rank_nav_image_arg = DeclareLaunchArgument(
        'publish_rank_nav_image', default_value='False',
        description='Publish debug image with bounding boxes'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Node
    rank_nav_node = Node(
        package='rank_nav_py',
        executable='rank_nav.py',
        name='rank_nav',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'people_topic': LaunchConfiguration('people_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'camera_serial': LaunchConfiguration('camera_serial'),
            'min_depth': LaunchConfiguration('min_depth'),
            'max_depth': LaunchConfiguration('max_depth'),
            'device': LaunchConfiguration('device'),
            'publish_rank_nav_image': LaunchConfiguration('publish_rank_nav_image'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        people_topic_arg,
        odom_topic_arg,
        camera_serial_arg,
        min_depth_arg,
        max_depth_arg,
        device_arg,
        publish_rank_nav_image_arg,
        use_sim_time_arg,
        rank_nav_node
    ])
