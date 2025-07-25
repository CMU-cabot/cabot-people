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

start=`date +%s.%N`

trap ctrl_c INT QUIT TERM

terminating=0
launched=0

function stop_people_test() {
    terminating=1
    cd $scriptdir
    if [[ ! -z $dccom ]]; then
        while [[ $launched -lt 5 ]]; do
            snore 1
            launched=$((launched+1))
        done

        red "kill -INT $dcpid"
        kill -INT $dcpid
        while kill -0 $dcpid 2> /dev/null; do
            snore 1
        done
        red "$dccom down"
        if [ $verbose -eq 1 ]; then
            $dccom down
        else
            $dccom down > /dev/null 2>&1
        fi
    fi
}

function ctrl_c() {
    red "catch the signal"
    user=$1
    stop_people_test
    exit $user
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

function help()
{
    echo "Usage:"
    echo "-h                show this help"
    echo "-N <dir>          cabot-navigation directory path"
    echo "-S <site>         override CABOT_SITE"
    echo "-T <module>       specify test module CABOT_SITE.<module> (default=tests)"
    echo "-f <test_regex>   run test matched with <test_regex> in CABOT_SITE.<module>"
    echo "-O <dir>          output directory path"
    echo "-a                plot with animation"
    echo "-d                development"
    echo "-n <name>         set log name prefix"
    echo "-v                verbose option"
}


cabot_navigation=
cabot_site=
module=tests
test_regex=
output_dir=
log_prefix=cabot
anim=0
verbose=0
profile=prod

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir/../
scriptdir=`pwd`
source $scriptdir/.env

if [ "$CABOT_LAUNCH_DEV_PROFILE" == "1" ]; then
    profile=dev
fi
if [ -n "$CABOT_LAUNCH_LOG_PREFIX" ]; then
    log_prefix=$CABOT_LAUNCH_LOG_PREFIX
fi

while getopts "hN:S:T:f:O:adn:v" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        N)
            cabot_navigation=$OPTARG
            ;;
        S)
            cabot_site=$OPTARG
            ;;
        T)
            module=$OPTARG
            ;;
        f)
            test_regex=$OPTARG
            ;;
        O)
            output_dir=$OPTARG
            ;;
        a)
            anim=1
            ;;
        d)
            profile=dev
            ;;
        n)
            log_prefix=$OPTARG
            ;;
        v)
            verbose=1
            ;;
    esac
done
shift $((OPTIND-1))

if [ ! -d "$cabot_navigation" ] || [ ! -f "$cabot_navigation/launch.sh" ]; then
    red "Specify cabot-navigation directory path"
    help
    exit 1
fi
if [ -z $cabot_site ]; then
    red "Specify cabot site name"
    help
    exit 1
fi
if [ -z $test_regex ]; then
    red "Specify test case name pattern"
    help
    exit 1
fi
if [ -z $output_dir ]; then
    red "Specify output directory path"
    help
    exit 1
fi

launch_prefix=$(basename $scriptdir)

log_name=${log_prefix}_`date +%Y-%m-%d-%H-%M-%S`
export ROS_LOG_DIR="/home/developer/.ros/log/${log_name}"
export ROS_LOG_DIR_ROOT="/root/.ros/log/${log_name}"
export CABOT_LOG_NAME=$log_name
host_ros_log=$scriptdir/docker/home/.ros/log
host_ros_log_dir=$host_ros_log/$log_name
mkdir -p $host_ros_log_dir
ln -snf $host_ros_log_dir $host_ros_log/latest
blue "log dir is : $host_ros_log_dir"
mkdir -p $host_ros_log_dir
cp $scriptdir/.env $host_ros_log_dir/env-file

## if network interface name for Cyclone DDS is not specified, set autoselect as true
if [ ! -z $CYCLONEDDS_URI ]; then
    if [ ! -z $CYCLONEDDS_NETWORK_INTERFACE_NAME ]; then
        export CYCLONEDDS_NETWORK_INTERFACE_AUTODETERMINE="false"
    else
        export CYCLONEDDS_NETWORK_INTERFACE_AUTODETERMINE="true"
    fi
fi

## launch cabot-people test
cd $scriptdir

echo "Launch cabot-peopole test"
dcfile=docker-compose-test-rs3.yaml
dccom="docker compose -f $dcfile -p $launch_prefix"
if [[ $profile == "dev" ]]; then
    dcservice="rs1-test-dev rs2-test-dev rs3-test-dev track-test-dev record-test-dev"
else
    dcservice="rs1-test-prod rs2-test-prod rs3-test-prod track-test-prod record-test-prod"
fi

if [ $verbose -eq 0 ]; then
    com="$dccom --ansi never up --no-build --abort-on-container-exit $dcservice > $host_ros_log_dir/docker-compose.log &"
else
    com="$dccom up --no-build --abort-on-container-exit $dcservice | tee $host_ros_log_dir/docker-compose.log &"
fi
if [ $verbose -eq 1 ]; then
    blue "$com"
fi
eval $com
dcpid=($!)
blue "[$dcpid] $dccom up $( echo "$(date +%s.%N) - $start" | bc -l )"

## wait to launch cabot-people test
sleep 5

## launch cabot-navigation test and wait finish test
echo "Launch cabot-navigation test"
com="$cabot_navigation/launch.sh -s -t -S $cabot_site -T $module -f $test_regex"
if [ $verbose -eq 1 ]; then
    blue "$com"
fi
eval $com
echo "Stopped cabot-navigation test"

stop_people_test
echo "Stopped cabot-people test"

## check bag file
if [ ! -d "$host_ros_log_dir/ros2_topics" ]; then
    red "cabot-people bag file does not exist"
    exit 1
elif [ ! -f "$host_ros_log_dir/ros2_topics/metadata.yaml" ]; then
    red "Fix cabot-people bag file"

    echo "Backup original bag file"
    cp -r $host_ros_log_dir/ros2_topics $host_ros_log_dir/ros2_topics.bak

    echo "Run reindex bag file"
    ros2 bag reindex $host_ros_log_dir/ros2_topics

    echo "Fix metadata"
    sed -i -E 's/^([[:space:]]*compression_format:)[[:space:]]*""/\1 zstd/' "$host_ros_log_dir/ros2_topics/metadata.yaml"
    sed -i -E 's/^([[:space:]]*compression_mode:)[[:space:]]*""/\1 MESSAGE/' "$host_ros_log_dir/ros2_topics/metadata.yaml"
fi

## evaluate tracking error
echo "Launch tracking evaluation"
anim_option=
if [ $anim -eq 1 ]; then
    anim_option=" -a"
fi
if [[ $profile == "dev" ]]; then
    dcservice="people-dev"
else
    dcservice="people-prod"
fi
com="docker compose -f docker-compose.yaml run --rm $dcservice python3 src/cabot_people/script/eval_track_error.py -i $ROS_LOG_DIR/ros2_topics -o $ROS_LOG_DIR/eval_track_error $anim_option"
if [ $verbose -eq 1 ]; then
    blue "$com"
fi
eval $com

echo "Copy test log to output directory"
cp -r $host_ros_log_dir $output_dir/
