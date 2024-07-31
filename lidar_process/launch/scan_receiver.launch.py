from launch.logging import launch_config

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

from ament_index_python.packages import get_package_share_directory
try:
    from cabot_common.launch import AppendLogDirPrefix
    workaround = False
except ImportError:
    workaround = True

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    ring_limit = LaunchConfiguration('ring_limit')
    scan_max_range = LaunchConfiguration('scan_max_range')
    history_window = LaunchConfiguration('history_window')
    future_window = LaunchConfiguration('future_window')
    low_level_pos_threshold = LaunchConfiguration('low_level_pos_threshold')
    high_level_pos_threshold = LaunchConfiguration('high_level_pos_threshold')
    high_level_vel_threshold = LaunchConfiguration('high_level_vel_threshold')
    high_level_ori_threshold = LaunchConfiguration('high_level_ori_threshold')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        LogInfo(msg=["no cabot_common"]) if workaround else RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("lidar_process")])),

        DeclareLaunchArgument('namespace', default_value='lidar'),
        DeclareLaunchArgument('ring_limit', default_value='7'),
        DeclareLaunchArgument('scan_max_range', default_value='100'),
        DeclareLaunchArgument('history_window', default_value='8'),
        DeclareLaunchArgument('future_window', default_value='8'),
        DeclareLaunchArgument('low_level_pos_threshold', default_value='0.25'),
        DeclareLaunchArgument('high_level_pos_threshold', default_value='1'),
        DeclareLaunchArgument('high_level_vel_threshold', default_value='1'),
        DeclareLaunchArgument('high_level_ori_threshold', default_value='1'),

        # overwrite parameters
        SetParameter(name='ring_limit', value=ring_limit),
        SetParameter(name='scan_max_range', value=scan_max_range),
        SetParameter(name='history_window', value=history_window),
        SetParameter(name='future_window', value=future_window),
        SetParameter('low_level_pos_threshold', value=low_level_pos_threshold),
        SetParameter('high_level_pos_threshold', value=high_level_pos_threshold),
        SetParameter('high_level_vel_threshold', value=high_level_vel_threshold),
        SetParameter('high_level_ori_threshold', value=high_level_ori_threshold),

        Node(
            package="lidar_process",
            executable="scan_receiver",
            name="scan_receiver",
            namespace=namespace,
            output="both",
            emulate_tty=True,
            #arguments=["--ros-args", "--log-level", "debug"]
        ),

        Node(
            package="cabot_common",
            executable="lookup_transform_service_node",
            name="lookup_transform_service_node",
        ),
    ])
