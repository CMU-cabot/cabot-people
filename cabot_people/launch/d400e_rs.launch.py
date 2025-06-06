#  Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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
from launch_ros.actions import Node

from cabot_common.launch import AppendLogDirPrefix


configurable_parameters = [
    {'name': 'camera_name',                             'default': 'camera',    'description': 'camera unique name'},
    {'name': 'serial_no',                               'default': "''",        'description': 'choose device by serial number'},
    {'name': 'ip_address',                              'default': "''",        'description': 'choose device by ip address'},
    {'name': 'usb_port_id',                             'default': "''",        'description': 'choose device by usb port id'},
    {'name': 'device_type',                             'default': "''",        'description': 'choose device by type'},
    {'name': 'config_file',                             'default': "''",        'description': 'yaml config file'},
    {'name': 'json_file_path',                          'default': "''",        'description': 'allows advanced configuration'},
    {'name': 'initial_reset',                           'default': 'false',     'description': "''"},
    {'name': 'accelerate_gpu_with_glsl',                'default': "false",     'description': 'enable GPU acceleration with GLSL'},
    {'name': 'rosbag_filename',                         'default': "''",        'description': 'A realsense bagfile to run from as a device'},
    {'name': 'log_level',                               'default': 'info',      'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
    {'name': 'output',                                  'default': 'screen',    'description': 'pipe node output [screen|log]'},
    {'name': 'enable_color',                            'default': 'true',      'description': 'enable color stream'},
    {'name': 'rgb_camera.packet_size',                  'default': '1500.0',    'description': '[FRAMOS] color stream packet size'},
    {'name': 'rgb_camera.inter_packet_delay',           'default': '13',        'description': '[FRAMOS] color stream inter packet delay'},
    {'name': 'rgb_camera.color_profile',                'default': '0,0,0',     'description': 'color stream profile'},
    {'name': 'rgb_camera.color_format',                 'default': 'RGB8',      'description': 'color stream format'},
    {'name': 'rgb_camera.enable_auto_exposure',         'default': 'true',      'description': 'enable/disable auto exposure for color image'},
    {'name': 'enable_depth',                            'default': 'true',      'description': 'enable depth stream'},
    {'name': 'enable_infra',                            'default': 'false',     'description': 'enable infra0 stream'},
    {'name': 'enable_infra1',                           'default': 'false',     'description': 'enable infra1 stream'},
    {'name': 'enable_infra2',                           'default': 'false',     'description': 'enable infra2 stream'},
    {'name': 'depth_module.packet_size',                'default': '1500.0',    'description': '[FRAMOS] depth stream packet size'},
    {'name': 'depth_module.inter_packet_delay',         'default': '13',        'description': '[FRAMOS] depth stream packet size'},
    {'name': 'depth_module.depth_profile',              'default': '0,0,0',     'description': 'depth stream profile'},
    {'name': 'depth_module.depth_format',               'default': 'Z16',       'description': 'depth stream format'},
    {'name': 'depth_module.infra_profile',              'default': '0,0,0',     'description': 'infra streams (0/1/2) profile'},
    {'name': 'depth_module.infra_format',               'default': 'RGB8',      'description': 'infra0 stream format'},
    {'name': 'depth_module.infra1_format',              'default': 'Y8',        'description': 'infra1 stream format'},
    {'name': 'depth_module.infra2_format',              'default': 'Y8',        'description': 'infra2 stream format'},
    {'name': 'depth_module.exposure',                   'default': '8500',      'description': 'Depth module manual exposure value'},
    {'name': 'depth_module.gain',                       'default': '16',        'description': 'Depth module manual gain value'},
    {'name': 'depth_module.hdr_enabled',                'default': 'false',     'description': 'Depth module hdr enablement flag. Used for hdr_merge filter'},
    {'name': 'depth_module.enable_auto_exposure',       'default': 'true',      'description': 'enable/disable auto exposure for depth image'},
    {'name': 'depth_module.exposure.1',                 'default': '7500',      'description': 'Depth module first exposure value. Used for hdr_merge filter'},
    {'name': 'depth_module.gain.1',                     'default': '16',        'description': 'Depth module first gain value. Used for hdr_merge filter'},
    {'name': 'depth_module.exposure.2',                 'default': '1',         'description': 'Depth module second exposure value. Used for hdr_merge filter'},
    {'name': 'depth_module.gain.2',                     'default': '16',        'description': 'Depth module second gain value. Used for hdr_merge filter'},
    {'name': 'enable_sync',                             'default': 'false',     'description': "'enable sync mode'"},
    {'name': 'enable_rgbd',                             'default': 'false',     'description': "'enable rgbd topic'"},
    {'name': 'enable_gyro',                             'default': 'false',     'description': "'enable gyro stream'"},
    {'name': 'enable_accel',                            'default': 'false',     'description': "'enable accel stream'"},
    {'name': 'gyro_fps',                                'default': '0',         'description': "''"},
    {'name': 'accel_fps',                               'default': '0',         'description': "''"},
    {'name': 'unite_imu_method',                        'default': "0",         'description': '[0-None, 1-copy, 2-linear_interpolation]'},
    {'name': 'clip_distance',                           'default': '-2.',       'description': "''"},
    {'name': 'angular_velocity_cov',                    'default': '0.01',      'description': "''"},
    {'name': 'linear_accel_cov',                        'default': '0.01',      'description': "''"},
    {'name': 'diagnostics_period',                      'default': '0.0',       'description': 'Rate of publishing diagnostics. 0=Disabled'},
    {'name': 'publish_tf',                              'default': 'true',      'description': '[bool] enable/disable publishing static & dynamic TF'},
    {'name': 'tf_publish_rate',                         'default': '0.0',       'description': '[double] rate in Hz for publishing dynamic TF'},
    {'name': 'pointcloud.enable',                       'default': 'false',     'description': ''},
    {'name': 'pointcloud.stream_filter',                'default': '2',         'description': 'texture stream for pointcloud'},
    {'name': 'pointcloud.stream_index_filter',          'default': '0',         'description': 'texture stream index for pointcloud'},
    {'name': 'pointcloud.ordered_pc',                   'default': 'false',     'description': ''},
    {'name': 'pointcloud.allow_no_texture_points',      'default': 'false',     'description': "''"},
    {'name': 'align_depth.enable',                      'default': 'false',     'description': 'enable align depth filter'},
    {'name': 'colorizer.enable',                        'default': 'false',     'description': 'enable colorizer filter'},
    {'name': 'decimation_filter.enable',                'default': 'false',     'description': 'enable_decimation_filter'},
    {'name': 'spatial_filter.enable',                   'default': 'false',     'description': 'enable_spatial_filter'},
    {'name': 'temporal_filter.enable',                  'default': 'false',     'description': 'enable_temporal_filter'},
    {'name': 'disparity_filter.enable',                 'default': 'false',     'description': 'enable_disparity_filter'},
    {'name': 'hole_filling_filter.enable',              'default': 'false',     'description': 'enable_hole_filling_filter'},
    {'name': 'hdr_merge.enable',                        'default': 'false',     'description': 'hdr_merge filter enablement flag'},
    {'name': 'wait_for_device_timeout',                 'default': '30.',       'description': 'Timeout for waiting for device to connect (Seconds)'},
    {'name': 'reconnect_timeout',                       'default': '6.',        'description': 'Timeout(seconds) between consequtive reconnection attempt'}
]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    output = {'stderr': {'log'}}
    log_level = 'info'
    camera_link_frame = LaunchConfiguration("camera_link_frame")

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("framos")])),
        # Realsense
        DeclareLaunchArgument("camera_link_frame", default_value="camera_link"),
        Node(
            package="cabot_people",
            executable="framos_initialize_node",
            name="framos_initialize_node",
            namespace=LaunchConfiguration("camera_name"),
            output=output,
            parameters=[{
                'camera_link_frame': camera_link_frame
            }]
        ),
        Node(
            package='realsense2_camera', 
            namespace='',
            name=LaunchConfiguration("camera_name"),
            executable='framos_realsense2_camera_node',
            parameters = [set_configurable_parameters(configurable_parameters)],
            output=output,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
            respawn=True,
            respawn_delay=5.0
            )
    ])
