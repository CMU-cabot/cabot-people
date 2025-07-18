# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

"""Launch realsense2_camera node."""
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.logging import launch_config
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from cabot_common.launch import AppendLogDirPrefix


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'unite_imu_method',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'depth_module.depth_profile',         'default': '0,0,0', 'description': 'depth stream profile'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.color_profile',           'default': '0,0,0', 'description': 'color stream profile'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'tracking_module.profile',      'default': '0,0,0', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1',              'default': 'true', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',              'default': 'true', 'description': 'enable fisheye2 stream'},
                           {'name': 'enable_confidence',            'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps',                     'default': '0', 'description': "''"},
                           {'name': 'accel_fps',                    'default': '0', 'description': "''"},
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "''"},
                           {'name': 'enable_accel',                 'default': 'false', 'description': "''"},
                           {'name': 'enable_pose',                  'default': 'true', 'description': "''"},
                           {'name': 'pose_fps',                     'default': '200', 'description': "''"},
                           {'name': 'pointcloud.enable',            'default': 'false', 'description': ''},
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': "''"},
                           {'name': 'align_depth.enable',           'default': 'false', 'description': "''"},
                           {'name': 'colorizer.enable',             'default': 'false', 'description': "''"},
                           {'name': 'clip_distance',                'default': '-2.', 'description': "''"},
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},
                           {'name': 'initial_reset',                'default': 'false', 'description': "''"},
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': "''"},
                           {'name': 'ordered_pc',                   'default': 'false', 'description': ''},
                           {'name': 'calib_odom_file',              'default': "''", 'description': "''"},
                           {'name': 'topic_odom_in',                'default': "''", 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'decimation_filter.enable',     'default': 'false', 'description': 'Rate of publishing static_tf'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'depth_module.exposure.1',     'default': '7500', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.gain.1',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.exposure.2',     'default': '1', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.gain.2',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    output = {'stderr': {'log'}}
    log_level = 'info'
    use_intra_process_comms = LaunchConfiguration("use_intra_process_comms")

    jetpack5_workaround = LaunchConfiguration('jetpack5_workaround')

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("realsense")])),
        # Realsense
        DeclareLaunchArgument("use_intra_process_comms", default_value="false"),
        DeclareLaunchArgument('jetpack5_workaround', default_value='false'),
        SetEnvironmentVariable(name='LD_PRELOAD', value='/usr/local/lib/libOpen3D.so', condition=IfCondition(jetpack5_workaround)),
        ComposableNodeContainer(
            name="camera_manager",
            namespace=LaunchConfiguration("camera_name"),
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense2_camera',
                    namespace='',
                    plugin='realsense2_camera::RealSenseNodeFactory',
                    name=LaunchConfiguration("camera_name"),
                    parameters=[set_configurable_parameters(configurable_parameters)],
                    extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                )
            ],
            output=output,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
        )
    ])
