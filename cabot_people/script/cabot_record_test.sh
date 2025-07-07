#!/bin/bash

# Copyright (c) 2025  Carnegie Mellon University, IBM Corporation, and others
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

set -m

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
pid=

trap signal INT TERM

function signal() {
    echo "trap cabot_record_test.sh"
    kill -2 $pid

    while kill -0 $pid 2> /dev/null; do
        if [[ $count -eq 15 ]]; then
            blue "escalate to SIGTERM $pid"
            com="kill -TERM $pid"
            eval $com
        fi
        if [[ $count -eq 30 ]]; then
            blue "escalate to SIGKILL $pid"
            com="kill -KILL $pid"
            eval $com
        fi
        echo "waiting $0 $pid"
        snore 1
        count=$((count+1))
    done

    exit
}
function err {
    >&2 red "[ERROR] "$@
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

echo "recording to $ROS_LOG_DIR"
source $scriptdir/../../../install/setup.bash

gazebo=${CABOT_GAZEBO:=0}
backend=${CABOT_ROSBAG_BACKEND:=sqlite3}
compression=${CABOT_ROSBAG_COMPRESSION:=message}

use_sim_time=
if [ $gazebo -eq 1 ]; then
    use_sim_time="--use-sim-time"
fi

if [[ $compression == "none" ]]; then
    compression=""
else
    compression="--compression-mode $compression --compression-format zstd"
fi

interval=1000

if [[ -z $ROS_LOG_DIR ]]; then
    # for debug only
    com="ros2 bag record ${use_sim_time} -s ${backend} ${compression} -p ${interval} /clock /tf /tf_static /people /obstacles /test/people /test/obstacles 2>&1 &"
else
    com="ros2 bag record ${use_sim_time} -s ${backend} ${compression} -p ${interval} -o $ROS_LOG_DIR/ros2_topics /clock /tf /tf_static /people /obstacles /test/people /test/obstacles 2>&1 &"
fi
echo $com
eval $com
pid=$!

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    # blue "snore"
    snore 1
done
