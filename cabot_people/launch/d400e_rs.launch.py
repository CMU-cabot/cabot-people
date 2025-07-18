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


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'dev_filter',                   'default': "''", 'description': 'device filter file'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "'src/realsense2_camera/config/d400e.yaml'", 'description': 'yaml config file'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'enable_pointcloud',            'default': 'false', 'description': 'enable pointcloud'},
                           {'name': 'unite_imu_method',             'default': "''", 'description': '[copy|linear_interpolation]'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'depth_width',                  'default': '-1', 'description': 'depth image width'},
                           {'name': 'depth_height',                 'default': '-1', 'description': 'depth image height'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'color_width',                  'default': '-1', 'description': 'color image width'},
                           {'name': 'color_height',                 'default': '-1', 'description': 'color image height'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'infra_width',                  'default': '-1', 'description': 'infra width'},
                           {'name': 'infra_height',                 'default': '-1', 'description': 'infra width'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'fisheye_width',                'default': '-1', 'description': 'fisheye width'},
                           {'name': 'fisheye_height',               'default': '-1', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1',              'default': 'true', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',              'default': 'true', 'description': 'enable fisheye2 stream'},
                           {'name': 'confidence_width',             'default': '-1', 'description': 'depth image width'},
                           {'name': 'confidence_height',            'default': '-1', 'description': 'depth image height'},
                           {'name': 'enable_confidence',            'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'fisheye_fps',                  'default': '-1.', 'description': ''},
                           {'name': 'depth_fps',                    'default': '-1.', 'description': ''},
                           {'name': 'confidence_fps',               'default': '-1.', 'description': ''},
                           {'name': 'infra_fps',                    'default': '-1.', 'description': ''},
                           {'name': 'color_fps',                    'default': '-1.', 'description': ''},
                           {'name': 'gyro_fps',                     'default': '-1.', 'description': ''},
                           {'name': 'accel_fps',                    'default': '-1.', 'description': ''},
                           {'name': 'color_qos',                    'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'confidence_qos',               'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'depth_qos',                    'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'fisheye_qos',                  'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'infra_qos',                    'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'pointcloud_qos',               'default': 'SYSTEM_DEFAULT', 'description': 'QoS profile name'},
                           {'name': 'enable_gyro',                  'default': 'false', 'description': ''},
                           {'name': 'enable_accel',                 'default': 'false', 'description': ''},
                           {'name': 'pointcloud_texture_stream',    'default': 'RS2_STREAM_COLOR', 'description': 'testure stream for pointcloud'},
                           {'name': 'pointcloud_texture_index',     'default': '0', 'description': 'testure stream index for pointcloud'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': ''},
                           {'name': 'align_depth',                  'default': 'false', 'description': ''},
                           {'name': 'filters',                      'default': "''", 'description': ''},
                           {'name': 'clip_distance',                'default': '-2.', 'description': ''},
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': ''},
                           {'name': 'initial_reset',                'default': 'false', 'description': ''},
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': ''},
                           {'name': 'ordered_pc',                   'default': 'false', 'description': ''},
                           {'name': 'calib_odom_file',              'default': "''", 'description': "''"},
                           {'name': 'topic_odom_in',                'default': "''", 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'temporal.holes_fill',          'default': '0', 'description': 'Persistency mode'},
                           {'name': 'stereo_module.exposure.1',     'default': '7500', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.gain.1',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.exposure.2',     'default': '1', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.gain.2',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'wait_for_device_timeout',      'default': '30.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
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
            namespace=LaunchConfiguration("camera_name"),
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
