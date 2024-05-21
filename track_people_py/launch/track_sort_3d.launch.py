#  Copyright (c) 2023  Carnegie Mellon University
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
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

try:
    from cabot_common.launch import AppendLogDirPrefix
    workaround = False
except ImportError:
    workaround = True


def generate_launch_description():
    iou_circle_size = LaunchConfiguration('iou_circle_size')
    kf_init_var = LaunchConfiguration('kf_init_var')
    kf_process_var = LaunchConfiguration('kf_process_var')
    kf_measure_var = LaunchConfiguration('kf_measure_var')
    target_fps = LaunchConfiguration('target_fps')

    # ToDo: workaround https://github.com/CMU-cabot/cabot/issues/86
    jetpack5_workaround = LaunchConfiguration('jetpack5_workaround')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),

        # append prefix name to the log directory for convenience
        LogInfo(msg=["no cabot_common"]) if workaround else RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("track_people_cpp-detect_darknet")])),

        DeclareLaunchArgument('iou_circle_size', default_value='0.5'),
        DeclareLaunchArgument('kf_init_var', default_value='1.0'),
        DeclareLaunchArgument('kf_process_var', default_value='1000.0'),
        DeclareLaunchArgument('kf_measure_var', default_value='1.0'),
        DeclareLaunchArgument('target_fps', default_value=EnvironmentVariable('CABOT_PEOPLE_TRACK_FPS', default_value='15.0')),

        DeclareLaunchArgument('jetpack5_workaround', default_value='false'),

        # overwrite parameters
        SetParameter(name='iou_circle_size', value=iou_circle_size),
        SetParameter(name='kf_init_var', value=kf_init_var),
        SetParameter(name='kf_process_var', value=kf_process_var),
        SetParameter(name='kf_measure_var', value=kf_measure_var),
        SetParameter(name='target_fps', value=target_fps),

        SetEnvironmentVariable(name='LD_PRELOAD', value='/usr/local/lib/libOpen3D.so', condition=IfCondition(jetpack5_workaround)),
        Node(
            package="track_people_py",
            executable="track_sort_3d_people.py",
            name="track_sort_3d_people_py",
        ),
    ])
