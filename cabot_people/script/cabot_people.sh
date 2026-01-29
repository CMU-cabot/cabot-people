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

set -m

## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    echo "trap cabot_people.sh "

    for pid in ${pids[@]}; do
        echo "send SIGINT to $pid"
        com="kill -INT $pid"
        eval $com
    done
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
: ${CABOT_PUBLISH_DETECT_IMAGE:=0}
: ${CABOT_REALSENSE_SERIAL:=}
: ${CABOT_CAMERA_NAME:=camera}
: ${CABOT_CAMERA_RGB_FPS:=30}
: ${CABOT_CAMERA_DEPTH_FPS:=15}
: ${CABOT_CAMERA_RESOLUTION:=1280}
: ${CABOT_DETECT_VERSION:=3}
: ${CABOT_DETECT_PEOPLE_CONF_THRES:=0.6}
: ${CABOT_DETECT_PEOPLE_CLEAR_TIME:=0.2}
: ${CABOT_DETECT_PEOPLE_REMOVE_GROUND:=1}
: ${CABOT_LOW_OBSTABLE_DETECT_VERSION:=0}
: ${CABOT_HEADLESS:=0}
if [[ $CABOT_HEADLESS -eq 1 ]]; then
    CABOT_SHOW_PEOPLE_RVIZ=0
fi

gazebo=$CABOT_GAZEBO
show_rviz=$CABOT_SHOW_PEOPLE_RVIZ
publish_detect_image=false
if [[ $CABOT_PUBLISH_DETECT_IMAGE -eq 1 ]]; then
    publish_detect_image=true
fi
realsense_camera=$CABOT_USE_REALSENSE
serial_no=$CABOT_REALSENSE_SERIAL

namespace=$CABOT_CAMERA_NAME
camera_link_frame="${CABOT_CAMERA_NAME}_link"

rgb_fps=$CABOT_CAMERA_RGB_FPS
depth_fps=$CABOT_CAMERA_DEPTH_FPS
resolution=$CABOT_CAMERA_RESOLUTION

cabot_detect_ver=$CABOT_DETECT_VERSION
remove_ground=false
if [[ $CABOT_DETECT_PEOPLE_REMOVE_GROUND -eq 1 ]]; then
    remove_ground=true
fi

cabot_low_obstacle_detect_ver=$CABOT_LOW_OBSTABLE_DETECT_VERSION

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
run_test=0

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
    echo "-v [1-9]                 specify detect implementation"
    echo "   1: python-opencv, 2: cpp-opencv-node, 3: cpp-opencv-nodelet"
    echo "   4: python-mmdet, 5: cpp-mmdet-node, 6: cpp-mmdet-nodelet"
    echo "   7: python-mmdet-seg, 8: cpp-mmdet-seg-node, 9: cpp-mmdet-seg-nodelet"
    echo "-N <name space>          namespace for tracking"
    echo "-f <camera_link_frame>   specify camera link frame"
    echo "-F <fps>                 specify camera RGB fps"
    echo "-P <fps>                 specify camera depth fps"
    echo "-S <camera serial>       specify serial number of realsense camera"
    echo "-R 1280/848/640          specify camera resolution"
    echo "-O                       obstacle detection/tracking"
    echo "-a                       no resetrs each"
    echo "-T                       run in test mode (remap /people and /obstacles topic for accuracy test)"
    exit
}

while getopts "hdm:n:w:srVCt:pWc:v:N:f:KDF:P:S:R:OaT" arg; do
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
        cabot_detect_ver=$OPTARG
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
    T)
        run_test=1
        ;;
    esac
done
shift $((OPTIND-1))

use_sim_time=false
if [ $gazebo -eq 1 ]; then
    use_sim_time=true;
fi

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

if [ $camera_type -eq 2 ]; then
    if [ $rgb_fps -lt 15 ] || [ $depth_fps -lt 15 ]; then
        red "FRAMOS camera will be reset if FPS is low, do not set FPS less than 15"
        exit
    fi
    if [ $cabot_detect_ver -eq 3 ] || [ $cabot_detect_ver -eq 6 ] || [ $cabot_detect_ver -eq 9 ]; then
        red "FRAMOS SDK does not support intra process communication yet, do not set 3, 6, 9 for CABOT_DETECT_VERSION"
        exit
    fi
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
echo "Detect impl   : $cabot_detect_ver"
echo "Namespace     : $namespace"
echo "Camera frame  : $camera_link_frame"
echo "RGB FPS       : $rgb_fps"
echo "Depth FPS     : $depth_fps"
echo "Resolution    : $width x $height"
echo "Obstacle      : $obstacle"


if [ $publish_tf -eq 1 ]; then
    eval "$command ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_footprint $commandpost"
    eval "$command ros2 run tf2_ros static_transform_publisher 0 0 1 0 0 $roll base_footprint ${camera_link_frame} $commandpost"
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
    if [ $cabot_detect_ver -eq 3 ] || [ $cabot_detect_ver -eq 6 ] || [ $cabot_detect_ver -eq 9 ]; then
        use_intra_process_comms=true
    fi
    if [ $camera_type -eq 1 ]; then
        launch_file="cabot_people rs_composite.launch.py"
        echo "launch $launch_file"
        eval "$command ros2 launch -n $launch_file \
                        align_depth.enable:=true \
                        depth_module.depth_profile:=$width,$height,$depth_fps \
                        rgb_camera.color_profile:=$width,$height,$rgb_fps \
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

        launch_file="cabot_people d400e_rs.launch.py"
        echo "launch $launch_file"
        eval "$command ros2 launch -n $launch_file \
                        align_depth:=true \
                        depth_width:=$width \
                        depth_height:=$height \
                        depth_fps:=$depth_fps \
                        color_width:=$width \
                        color_height:=$height \
                        color_fps:=$rgb_fps \
                        $option \
                        camera_name:=${namespace} \
                        camera_link_frame:=${camera_link_frame} \
                        $commandpost"
        pids+=($!)
    else
        red "invalid camera type"
        exit
    fi
fi

if [ $detection -eq 1 ]; then
    ### launch people detect
    map_frame='map'
    depth_registered_topic='aligned_depth_to_color/image_raw'
    if [ $gazebo -eq 1 ]; then
        depth_registered_topic='depth/image_raw'
    fi
    min_bbox_size=20.0

    # overwrite mmdeploy model by environment variables
    mmdeploy_model_option=''
    segment_model_option=''
    if [ $cabot_detect_ver -ge 4 ] && [ $cabot_detect_ver -le 9 ]; then
        install_model_dir=$scriptdir/../../../install/track_people_py/share/track_people_py/models
        mmdeploy_model_dir=/tmp/mmdeploy_model

        # copy mmdeploy model
        rm -rf $mmdeploy_model_dir
        if [ $cabot_detect_ver -ge 4 ] && [ $cabot_detect_ver -le 6 ]; then
            if [ ! -e "$install_model_dir/rtmdet/$processor/end2end.engine" ]; then
                red "model does not exists"
                exit
            fi
            cp -r $install_model_dir/rtmdet/$processor $mmdeploy_model_dir
        else
            if [ ! -e "$install_model_dir/rtmdet-ins/$processor/end2end.engine" ]; then
                red "model does not exists"
                exit
            fi
            cp -r $install_model_dir/rtmdet-ins/$processor $mmdeploy_model_dir
        fi

        # read input size
        mmdeploy_input_width=$(jq ".pipeline.tasks[0].transforms[] | select(.type==\"Resize\").size[0]" $mmdeploy_model_dir/pipeline.json)
        mmdeploy_input_height=$(jq ".pipeline.tasks[0].transforms[] | select(.type==\"Resize\").size[1]" $mmdeploy_model_dir/pipeline.json)

        # resize min bbox size by mmdeploy model input width and height
        int_min_bbox_size=$(echo "$min_bbox_size / 1" | bc)

        resized_min_bbox_width=$(echo "scale=2; $int_min_bbox_size * ($mmdeploy_input_width / $width)" | bc)
        int_resized_min_bbox_width=$(echo "$resized_min_bbox_width / 1" | bc)

        resized_min_bbox_height=$(echo "scale=2; $int_min_bbox_size * ($mmdeploy_input_height / $height)" | bc)
        int_resized_min_bbox_height=$(echo "$resized_min_bbox_height / 1" | bc)

        # set smaller value of resized min bbox size as min_bbox_size for mmdeploy model
        int_resized_min_bbox_size=$(($int_resized_min_bbox_width<$int_resized_min_bbox_height ? $int_resized_min_bbox_width : $int_resized_min_bbox_height))

        # set score_thr, min_bbox_size in pipeline.json
        cat $mmdeploy_model_dir/pipeline.json | jq ".pipeline.tasks[-1].params.score_thr|=${CABOT_DETECT_PEOPLE_CONF_THRES}" | jq ".pipeline.tasks[-1].params.min_bbox_size|=$int_resized_min_bbox_size" > $mmdeploy_model_dir/pipeline.json.tmp
        mv $mmdeploy_model_dir/pipeline.json.tmp $mmdeploy_model_dir/pipeline.json

        # create mmdeploy launch options
        mmdeploy_model_option="detect_model_dir:=$mmdeploy_model_dir"
        if [ $cabot_detect_ver -ge 7 ] && [ $cabot_detect_ver -le 9 ]; then
            segment_model_option="model_input_width:=$mmdeploy_input_width model_input_height:=$mmdeploy_input_height"
        fi
    fi

    if [ $cabot_detect_ver -ge 1 ] && [ $cabot_detect_ver -le 3 ]; then
        launch_file="detect_darknet.launch.py"
    elif [ $cabot_detect_ver -ge 4 ] && [ $cabot_detect_ver -le 6 ]; then
        launch_file="detect_mmdet.launch.py"
    elif [ $cabot_detect_ver -ge 7 ] && [ $cabot_detect_ver -le 9 ]; then
        launch_file="detect_mmdet_seg.launch.py"
    else
        red "detect implementation should be from 1 to 9"
        exit
    fi

    camera_id_option=""
    if [ -n "$serial_no" ]; then
        camera_id_option="camera_id:=$serial_no"
    fi

    if [ $cabot_detect_ver -eq 1 ] || [ $cabot_detect_ver -eq 4 ] || [ $cabot_detect_ver -eq 7 ]; then
        # python
        echo "launch track_people_py $launch_file"
        com="$command ros2 launch -n track_people_py $launch_file \
                      namespace:=$namespace \
                      $camera_id_option \
                      map_frame:=$map_frame \
                      camera_link_frame:=$camera_link_frame \
                      depth_registered_topic:=$depth_registered_topic \
                      remove_ground:=$remove_ground \
                      minimum_detection_size_threshold:=$min_bbox_size \
                      publish_detect_image:=$publish_detect_image \
                      jetpack5_workaround:=$jetpack5_workaround \
                      $mmdeploy_model_option \
                      $segment_model_option \
                      $commandpost"
        echo $com
        eval $com
        pids+=($!)
    else
        use_composite=0

        # do not use nodelet if it is on gazebo
        if [ $gazebo -eq 0 ] && { [ $cabot_detect_ver -eq 3 ] || [ $cabot_detect_ver -eq 6 ] || [ $cabot_detect_ver -eq 9 ]; }; then
            sleep 2
            use_composite=1
        fi
        # cpp
        echo "launch track_people_cpp $launch_file"
        com="$command ros2 launch -n track_people_cpp $launch_file \
                      namespace:=$namespace \
                      $camera_id_option \
                      map_frame:=$map_frame \
                      camera_link_frame:=$camera_link_frame \
                      use_composite:=$use_composite \
                      depth_registered_topic:=$depth_registered_topic \
                      remove_ground:=$remove_ground \
                      minimum_detection_size_threshold:=$min_bbox_size \
                      publish_detect_image:=$publish_detect_image \
                      jetpack5_workaround:=$jetpack5_workaround \
                      $mmdeploy_model_option \
                      $segment_model_option \
                      $commandpost"
        echo $com
        eval $com
        pids+=($!)
    fi
fi

if [ $tracking -eq 1 ]; then
    ### launch people track
    opt_track=''
    if [ $run_test -eq 1 ]; then
        opt_track+=' remap_people_topic:=/test/people'
    fi
    launch_file="track_people_py track_sort_3d.launch.py"
    echo "launch $launch_file"
    com="$command ros2 launch -n $launch_file $opt_track \
                  jetpack5_workaround:=$jetpack5_workaround \
                  use_sim_time:=$use_sim_time \
                  $commandpost"
    echo $com
    eval $com
    pids+=($!)
fi

### obstacle detect/track
if [ $obstacle -eq 1 ]; then
    target_fps=10.0
    launch_file="track_people_cpp detect_obstacles.launch.py sensor_id:=velodyne scan_topic:=/scan"
    echo "launch $launch_file"
    com="$command ros2 launch -n $launch_file \
                  sensor_id:=velodyne \
                  scan_topic:=/scan \
                  $commandpost"
    echo $com
    eval $com
    pids+=($!)

    if [ $cabot_low_obstacle_detect_ver -gt 0 ]; then
	target_fps=20.0
        launch_file="track_people_cpp detect_obstacles.launch.py sensor_id:=livox scan_topic:=/livox_scan"
        echo "launch $launch_file"
        com="$command ros2 launch -n $launch_file \
                    sensor_id:=livox \
                    scan_topic:=/livox_scan \
                    $commandpost"
        echo $com
        eval $com
        pids+=($!)
    fi

    opt_track=''
    if [ $run_test -eq 1 ]; then
        opt_track+=' remap_obstacles_topic:=/test/obstacles'
    fi
    launch_file="track_people_cpp track_obstacles.launch.py"
    echo "launch $launch_file"
    com="$command ros2 launch -n $launch_file $opt_track \
                  jetpack5_workaround:=$jetpack5_workaround \
                  target_fps:=$target_fps \
                  use_sim_time:=$use_sim_time \
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
