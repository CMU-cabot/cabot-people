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
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

try:
    from cabot_common.launch import AppendLogDirPrefix
    workaround = False
except ImportError:
    workaround = True


def generate_launch_description():
    output = {'stderr': {'log'}}
    # ToDo: workaround https://github.com/CMU-cabot/cabot/issues/86
    jetpack5_workaround = LaunchConfiguration('jetpack5_workaround')
    use_sim_time = LaunchConfiguration('use_sim_time')
    target_fps = LaunchConfiguration('target_fps')
    remap_obstacles_topic = LaunchConfiguration('remap_obstacles_topic')

    return LaunchDescription([
        # save all log file in the directory where the launch.log file is saved
        SetEnvironmentVariable('ROS_LOG_DIR', launch_config.log_dir),
        # append prefix name to the log directory for convenience
        LogInfo(msg=["no cabot_common"]) if workaround else RegisterEventHandler(OnShutdown(on_shutdown=[AppendLogDirPrefix("track_people_cpp-track_obstacles")])),

        DeclareLaunchArgument('jetpack5_workaround', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('target_fps', default_value='10.0'),
        DeclareLaunchArgument('remap_obstacles_topic', default_value=''),

        SetEnvironmentVariable(name='LD_PRELOAD', value='/usr/local/lib/libOpen3D.so', condition=IfCondition(jetpack5_workaround)),

        Node(
            package='track_people_py',
            executable='track_sort_3d_people.py',
            name='track_obstacle',
            namespace='obstacle',
            output=output,
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_fps': target_fps,
                'diagnostic_name': 'ObstacleTrack'
            }]
        ),

        Node(
            package="track_people_py",
            executable="predict_kf_obstacle.py",
            name="predict_obstacle",
            namespace='obstacle',
            output=output,
            parameters=[{
                'duration_inactive_to_stop_publish': 0.2,
                'stationary_detect_threshold_duration': 1.0,
                'target_fps': target_fps,
                'diagnostic_name': 'ObstaclePredict'
            }],
            remappings=[('/obstacle/people', '/obstacles')],
            condition=IfCondition(
                PythonExpression(["'", remap_obstacles_topic, "' == ''"])
            )
        ),
        Node(
            package="track_people_py",
            executable="predict_kf_obstacle.py",
            name="predict_obstacle",
            namespace='obstacle',
            output=output,
            parameters=[{
                'duration_inactive_to_stop_publish': 0.2,
                'stationary_detect_threshold_duration': 1.0,
                'target_fps': target_fps,
                'diagnostic_name': 'ObstaclePredict'
            }],
            remappings=[('/obstacle/people', remap_obstacles_topic)],
            condition=IfCondition(
                PythonExpression(["'", remap_obstacles_topic, "' != ''"])
            )
        )
    ])
