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
    from cabot_common.launch import AppendLogDirPrefix # type: ignore
    workaround = False
except ImportError:
    workaround = True

def generate_launch_description():
    output = {'stdout': 'log', 'stderr': 'log'}
    namespace = LaunchConfiguration('namespace')
    ring_lower_limit = LaunchConfiguration('ring_lower_limit')
    ring_upper_limit = LaunchConfiguration('ring_upper_limit')
    scan_max_range = LaunchConfiguration('scan_max_range')
    history_window = LaunchConfiguration('history_window')
    future_window = LaunchConfiguration('future_window')
    low_level_pos_threshold = LaunchConfiguration('low_level_pos_threshold')
    low_level_core_samples = LaunchConfiguration('low_level_core_samples')
    high_level_pos_threshold = LaunchConfiguration('high_level_pos_threshold')
    high_level_vel_threshold = LaunchConfiguration('high_level_vel_threshold')
    high_level_ori_threshold = LaunchConfiguration('high_level_ori_threshold')
    smooth_window = LaunchConfiguration('smooth_window')
    ignore_window = LaunchConfiguration('ignore_window')
    static_threshold = LaunchConfiguration('static_threshold')
    max_tracking_time = LaunchConfiguration('max_tracking_time')
    max_tracking_dist = LaunchConfiguration('max_tracking_dist')
    large_obs_size = LaunchConfiguration('large_obs_size')
    max_queue_size = LaunchConfiguration('max_queue_size')
    history_dt = LaunchConfiguration('history_dt')
    shape_increments = LaunchConfiguration('shape_increments')
    shape_scale = LaunchConfiguration('shape_scale')
    shape_offset = LaunchConfiguration('shape_offset')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        LogInfo(msg=["no cabot_common"]) if workaround else RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("lidar_process")])),

        DeclareLaunchArgument('namespace', default_value='lidar'),
        DeclareLaunchArgument('ring_lower_limit', default_value='7'),
        DeclareLaunchArgument('ring_upper_limit', default_value='8'),
        DeclareLaunchArgument('scan_max_range', default_value='15'),
        DeclareLaunchArgument('history_window', default_value='8'),
        DeclareLaunchArgument('future_window', default_value='12'),
        DeclareLaunchArgument('history_dt', default_value='0.4'),
        DeclareLaunchArgument('low_level_pos_threshold', default_value='0.5'),
        DeclareLaunchArgument('low_level_core_samples', default_value='5'),
        DeclareLaunchArgument('high_level_pos_threshold', default_value='2.0'),
        DeclareLaunchArgument('high_level_vel_threshold', default_value='1.0'),
        DeclareLaunchArgument('high_level_ori_threshold', default_value='45.0'),
        DeclareLaunchArgument('smooth_window', default_value='5'),
        DeclareLaunchArgument('ignore_window', default_value='0'),
        DeclareLaunchArgument('static_threshold', default_value='0.25'),
        DeclareLaunchArgument('max_tracking_time', default_value='0.4'),
        DeclareLaunchArgument('max_tracking_dist', default_value='1.0'),
        DeclareLaunchArgument('large_obs_size', default_value='2.0'),
        DeclareLaunchArgument('max_queue_size', default_value='50'),
        DeclareLaunchArgument('shape_increments', default_value='16'),
        DeclareLaunchArgument('shape_scale', default_value='0.354163'),
        DeclareLaunchArgument('shape_offset', default_value='1.0'),

        # overwrite parameters
        SetParameter(name='namespace', value=namespace),
        SetParameter(name='ring_lower_limit', value=ring_lower_limit),
        SetParameter(name='ring_upper_limit', value=ring_upper_limit),
        SetParameter(name='scan_max_range', value=scan_max_range),
        SetParameter(name='history_window', value=history_window),
        SetParameter(name='future_window', value=future_window),
        SetParameter(name='low_level_pos_threshold', value=low_level_pos_threshold),
        SetParameter(name='low_level_core_samples', value=low_level_core_samples),
        SetParameter(name='high_level_pos_threshold', value=high_level_pos_threshold),
        SetParameter(name='high_level_vel_threshold', value=high_level_vel_threshold),
        SetParameter(name='high_level_ori_threshold', value=high_level_ori_threshold),
        SetParameter(name='smooth_window', value=smooth_window),
        SetParameter(name='ignore_window', value=ignore_window),
        SetParameter(name='static_threshold', value=static_threshold),
        SetParameter(name='max_tracking_time', value=max_tracking_time),
        SetParameter(name='max_tracking_dist', value=max_tracking_dist),
        SetParameter(name='large_obs_size', value=large_obs_size),
        SetParameter(name='max_queue_size', value=max_queue_size),
        SetParameter(name='history_dt', value=history_dt),
        SetParameter(name='shape_increments', value=shape_increments),
        SetParameter(name='shape_scale', value=shape_scale),
        SetParameter(name='shape_offset', value=shape_offset),

        # Node(
        #     package="lidar_process",
        #     executable="scan_receiver",
        #     name="scan_receiver",
        #     namespace=namespace,
        #     output=output,
        #     emulate_tty=True,
        #     #arguments=["--ros-args", "--log-level", "debug"]
        # ),

        Node(
            package="lidar_process",
            executable="rl_server",
            name="rl_server",
            namespace=namespace,
            output=output,
            #arguments=["--ros-args", "--log-level", "debug"]
        ),

        Node(
            package="cabot_common",
            executable="lookup_transform_service_node",
            name="lookup_transform_service_node",
        ),
    ])
