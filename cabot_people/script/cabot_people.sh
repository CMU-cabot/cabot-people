#!/bin/bash

# Copyright (c) 2021, 2023  IBM Corporation and Carnegie Mellon University
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


## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    echo "trap cabot_people.sh "

    kill -INT -1

#    for pid in ${pids[@]}; do
#       echo "send SIGINT to $pid"
#        com="kill -INT $pid"
#        eval $com
#    done
    for pid in ${pids[@]}; do
        count=0
         while kill -0 $pid 2> /dev/null; do
            if [[ $count -eq 10 ]]; then
                echo "escalate to SIGTERM $pid"
                com="kill -TERM $pid"
                eval $com
            fi
            if [[ $count -eq 20 ]]; then
                echo "escalate to SIGKILL $pid"
                com="kill -KILL $pid"
                eval $com
            fi
             echo "waiting $0 $pid"
             snore 1
            count=$((count+1))
         done
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

## private variables
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

### default variables

## debug
minimum=0
debug=0
command=''
commandpost='&'

: ${CABOT_GAZEBO:=0}
: ${CABOT_USE_REALSENSE:=0}
: ${CABOT_SHOW_PEOPLE_RVIZ:=0}
: ${CABOT_REALSENSE_SERIAL:=}
: ${CABOT_CAMERA_NAME:=camera}
: ${CABOT_CAMERA_RGB_FPS:=30}
: ${CABOT_CAMERA_DEPTH_FPS:=15}
: ${CABOT_CAMERA_RESOLUTION:=1280}
: ${CABOT_DETECT_VERSION:=3}
: ${CABOT_DETECT_PEOPLE_CONF_THRES:=0.6}
: ${CABOT_DETECT_PEOPLE_CLEAR_TIME:=0.2}
: ${CABOT_HEADLESS:=0}
if [[ $CABOT_HEADLESS -eq 1 ]]; then
    CABOT_SHOW_PEOPLE_RVIZ=0
fi

gazebo=$CABOT_GAZEBO
show_rviz=$CABOT_SHOW_PEOPLE_RVIZ
realsense_camera=$CABOT_USE_REALSENSE
serial_no=$CABOT_REALSENSE_SERIAL

namespace=$CABOT_CAMERA_NAME
camera_link_frame="${CABOT_CAMERA_NAME}_link"

rgb_fps=$CABOT_CAMERA_RGB_FPS
depth_fps=$CABOT_CAMERA_DEPTH_FPS
resolution=$CABOT_CAMERA_RESOLUTION

opencv_dnn_ver=$CABOT_DETECT_VERSION

camera_type=1
check_required=0
publish_tf=0
publish_sim_people=0
wait_roscore=0
roll=0
tracking=0
detection=0
obstacle=0
noreset=0

processor=$(uname -m)

### usage print function
function usage {
    echo "Usage"
    echo "    run this script after running cabot.sh in another terminal"
    echo "ex)"
    echo $0 "-r -D -K"
    echo ""
    echo "-h                       show this help"
    echo "-d                       debug"
    echo "-V                       show rviz"
    echo "-m <map file>            specify a map file"
    echo "-n <anchor file>         specify a anchor file, use map file if not specified"
    echo "-w <world file>          specify a world file"
    echo "-s                       specify its on simulation (gazebo)"
    echo "-r                       launch realsense camera"
    echo "-p                       publish simulation people instead of detected people from camera"
    echo "-K                       use people tracker"
    echo "-D                       use people detector"
    echo "-C                       check required before launch"
    echo "-W                       wait roscore"
    echo "-t <roll>                publish map camera_link tf"
    echo "-c [1-2]                 camera type"
    echo "   1: RealSense, 2: FRAMOS"
    echo "-v [1-3]                 use specified opencv dnn implementation"
    echo "   1: python-opencv, 2: cpp-opencv-node, 3: cpp-opencv-nodelet"
    echo "-N <name space>          namespace for tracking"
    echo "-f <camera_link_frame>   specify camera link frame"
    echo "-F <fps>                 specify camera RGB fps"
    echo "-P <fps>                 specify camera depth fps"
    echo "-S <camera serial>       specify serial number of realsense camera"
    echo "-R 1280/848/640          specify camera resolution"
    echo "-O                       obstacle detection/tracking"
    echo "-a                       no resetrs each"
    exit
}

while getopts "hdm:n:w:srVCt:pWc:v:N:f:KDF:P:S:R:Oa" arg; do
    case $arg in
    h)
        usage
        exit
        ;;
    d)
        debug=1
        command="setsid xterm -e '"
        commandpost=";read'&"
        ;;
    m)
        map=$OPTARG
        ;;
    n)
        anchor=$OPTARG
        ;;
    w)
        world=$OPTARG
        ;;
    s)
        gazebo=1
        ;;
    r)
        realsense_camera=1
        ;;
    V)
        show_rviz=1
        ;;
    C)
        check_required=1
        ;;
    t)
        publish_tf=1
            roll=$OPTARG
        ;;
    p)
        publish_sim_people=1
        ;;
    W)
        wait_roscore=1
        ;;
    c)
        camera_type=$OPTARG
        ;;
    v)
        opencv_dnn_ver=$OPTARG
        ;;
    N)
        namespace=$OPTARG
        ;;
    f)
        camera_link_frame=$OPTARG
        ;;
    K)
        tracking=1
        ;;
    D)
        detection=1
        ;;
    F)
        rgb_fps=$OPTARG
        ;;
    P)
        depth_fps=$OPTARG
        ;;
    S)
        serial_no=$OPTARG
        ;;
    R)
        resolution=$OPTARG
        ;;
    O)
        obstacle=1
        ;;
    a)
        noreset=1
        ;;
    esac
done
shift $((OPTIND-1))

width=$resolution
if [ $width -eq 1280 ]; then
    height=720
elif [ $width -eq 848 ]; then
    height=480
elif [ $width -eq 640 ]; then
    height=360
else
    red "resolution should be one of 1280, 848, or 640"
    exit
fi

if [ $camera_type -eq 2 ] && [ $opencv_dnn_ver -eq 3 ]; then
    red "FRAMOS SDK does not support intra process communication yet, do not set CABOT_DETECT_VERSION=3"
    exit
fi

if [ $check_required -eq 1 ]; then
    flag=1
    while [ $flag -eq 1 ];
    do
        flag=0

        if [ $gazebo -eq 1 ]; then
            # Check Gazebo
            if [ `rostopic list | grep gazebo | wc -l` -eq 0 ]; then
                echo "Gazebo is not working"
                flag=1
            fi
        else
            # Check RealSense
            if [ `rs-fw-update -l | grep D435 | wc -l` -eq 0 ]; then
                echo "Realsense is not working"
                flag=1
            fi
        fi
        
        # Check weight
        if [ `find $scriptdir/../../ -name yolov4.weights | wc -l` -eq 0 ]; then
            echo "yolov4.weights is not found"
            flag=1
        fi
        
        snore 2
    done
fi

## debug output
echo "Use Realsense : $realsense_camera"
echo "Debug         : $debug ($command, $commandpost)"
echo "World         : $world"
echo "Map           : $map"
echo "Anchor        : $anchor"
echo "Simulation    : $gazebo"
echo "Camera type   : $camera_type"
echo "DNN impl      : $opencv_dnn_ver"
echo "Namespace     : $namespace"
echo "Camera frame  : $camera_link_frame"
echo "RGB FPS       : $rgb_fps"
echo "Depth FPS     : $depth_fps"
echo "Resolution    : $width x $height"
echo "Obstacle      : $obstacle"


if [ $publish_tf -eq 1 ]; then
    eval "$command ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 $roll map ${camera_link_frame} $commandpost"
    pids+=($!)
fi

### launch rviz2
if [ $show_rviz -eq 1 ]; then
    echo "launch rviz2"
    eval "$command rviz2 -d $scriptdir/cabot_people.rviz $commandpost"
    pids+=($!)
fi

# ToDo: workaround https://github.com/CMU-cabot/cabot/issues/86
jetpack5_workaround=false
if [[ $processor == 'aarch64' ]]; then
    jetpack5_workaround=true
fi

### launch realsense camera
if [ $realsense_camera -eq 1 ]; then

    if [ $noreset -eq 0 ]; then
        # reset RealSense or FRAMOS
        if [ $camera_type -eq 1 ]; then
            sudo /resetrs.sh $serial_no
        elif [ $camera_type -eq 2 ]; then
            sudo /resetframos.sh $serial_no
        else
            red "invalid camera type"
            exit
        fi
    fi

    option=""
    # work around to specify number string as string
    if [[ ! -z $serial_no ]]; then option="$option \"serial_no:='$serial_no'\""; fi
    use_intra_process_comms=false
    if [ $opencv_dnn_ver -eq 3 ]; then
        use_intra_process_comms=true
    fi
    if [ $camera_type -eq 1 ]; then
        launch_file="cabot_people rs_composite.launch.py"
        echo "launch $launch_file"
        eval "$command ros2 launch $launch_file \
                        align_depth.enable:=true \
                        depth_module.profile:=$width,$height,$depth_fps \
                        rgb_camera.profile:=$width,$height,$rgb_fps \
                        use_intra_process_comms:=$use_intra_process_comms \
                        jetpack5_workaround:=$jetpack5_workaround \
                        $option \
                        camera_name:=${namespace} $commandpost"
        pids+=($!)
    elif [ $camera_type -eq 2 ]; then
        # if FPS is intger, convert to float for FRAMOS SDK
        if [[ $rgb_fps =~ ^[+-]?[0-9]+$ ]]; then
            rgb_fps="${rgb_fps}.0" 
        fi
        if [[ $depth_fps =~ ^[+-]?[0-9]+$ ]]; then
            depth_fps="${depth_fps}.0" 
        fi

        launch_file="cabot_people d400e_rs_composite.launch.py"
        echo "launch $launch_file"
        eval "$command ros2 launch $launch_file \
                        align_depth:=true \
                        depth_width:=$width \
                        depth_height:=$height \
                        depth_fps:=$depth_fps \
                        color_width:=$width \
                        color_height:=$height \
                        color_fps:=$rgb_fps \
                        use_intra_process_comms:=$use_intra_process_comms \
                        jetpack5_workaround:=$jetpack5_workaround \
                        $option \
                        camera_name:=${namespace} $commandpost"
        pids+=($!)
    else
        red "invalid camera type"
        exit
    fi
fi

opt_predict=''

if [ $detection -eq 1 ]; then
    ### launch people detect
    map_frame='map'
    depth_registered_topic='aligned_depth_to_color/image_raw'
    if [ $gazebo -eq 1 ]; then
        depth_registered_topic='depth/image_raw'
    fi
        
    if [ $opencv_dnn_ver -ge 2 ]; then
        use_composite=0

        # do not use nodelet if it is on gazebo
        if [ $gazebo -eq 0 ] && [ $opencv_dnn_ver -eq 3 ]; then
            sleep 2
            use_composite=1
        fi
        # cpp
        com="$command ros2 launch track_people_cpp detect_darknet.launch.py \
                      namespace:=$namespace \
                      map_frame:=$map_frame \
                      camera_link_frame:=$camera_link_frame \
                      use_composite:=$use_composite \
                      depth_registered_topic:=$depth_registered_topic \
                      jetpack5_workaround:=$jetpack5_workaround \
                      $commandpost"
        echo $com
        eval $com
        pids+=($!)
    else
        # python
        launch_file="track_people_py detect_darknet.launch.py"
        echo "launch $launch_file"
        com="$command ros2 launch $launch_file \
                      namespace:=$namespace \
                      map_frame:=$map_frame \
                      camera_link_frame:=$camera_link_frame \
                      depth_registered_topic:=$depth_registered_topic \
                      jetpack5_workaround:=$jetpack5_workaround \
                      $commandpost"
        echo $com
        eval $com
        pids+=($!)
    fi

fi

if [ $tracking -eq 1 ]; then
    ### launch people track
    launch_file="track_people_py track_sort_3d.launch.py"
    echo "launch $launch_file"
    com="$command ros2 launch $launch_file \
                  jetpack5_workaround:=$jetpack5_workaround \
                  $commandpost"
    echo $com
    eval $com
    pids+=($!)

    ### launch people predict
    opt_predict=''
    if [ $gazebo -eq 1 ] && [ $publish_sim_people -eq 1 ]; then
        opt_predict='publish_simulator_people:=true'
    fi
    launch_file="track_people_py predict_kf.launch.py"
    echo "launch $launch_file"
    com="$command ros2 launch $launch_file $opt_predict \
                  jetpack5_workaround:=$jetpack5_workaround \
                  $commandpost"
    echo $com
    eval $com
    pids+=($!)
fi

### obstacle detect/track
if [ $obstacle -eq 1 ]; then
    launch_file="track_people_cpp track_obstacles.launch.py"
    echo "launch $launch_file"
    com="$command ros2 launch $launch_file \
                  jetpack5_workaround:=$jetpack5_workaround \
                  $commandpost"
    echo $com
    eval $com
    pids+=($!)
fi

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done
