# Copyright (c) 2022  Carnegie Mellon University
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

from launch.logging import launch_config

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

try:
    from cabot_common.launch import AppendLogDirPrefix
    workaround = False
except ImportError:
    workaround = True


def generate_launch_description():
    output = {'stderr': {'log'}}
    sensor_id = LaunchConfiguration('sensor_id')
    scan_topic = LaunchConfiguration('scan_topic')
    map_frame = LaunchConfiguration('map_frame')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        LogInfo(msg=["no cabot_common"]) if workaround else RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("track_people_cpp-detect_obstacles")])),

        DeclareLaunchArgument('sensor_id', default_value='velodyne'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('map_frame', default_value='map'),

        # overwrite parameters
        SetParameter(name='sensor_id', value=sensor_id),
        SetParameter(name='scan_topic', value=scan_topic),
        SetParameter(name='map_frame', value=map_frame),

        Node(
            package='track_people_cpp',
            executable='detect_obstacle_on_path_node',
            name='detect_obstacle_on_path',
            namespace='obstacle',
            output=output,
        )
    ])
