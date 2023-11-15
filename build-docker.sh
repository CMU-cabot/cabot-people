#!/bin/bash

# Copyright (c) 2020, 2023  Carnegie Mellon University, IBM Corporation, and others
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

## build base image

function red {
    echo -en "\033[31m"  ## red
    echo $1
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $1
    echo -en "\033[0m"  ## reset color
}
function help {
    echo "Usage"
    echo "$0 [<option>] [<targets>]"
    echo ""
    echo "targets : all: all targets"
    echo "          x86_64: build ROS2 humble on CUDA"
    echo "          aarch64:  build ROS2 humble for Jeston"
    echo "          see bellow if targets is not specified"
    echo ""
    echo "  Your env: arch=$arch"
    echo "    default target=\"x86_64\" if arch=x86_64"
    echo "    default target=\"aarch64\"  if arch=aarch64"
    echo ""
    echo "-h                    show this help"
    echo "-q                    quiet option for docker build"
    echo "-n                    nocache option for docker build"
    echo "-t <time_zone>        set time zone (default=$time_zone, your local time zone)"
    echo "-d                    debug without BUILDKIT"
    echo "-y                    no confirmation"
    echo "-w                    build only workspace"
}

arch=$(uname -m)
time_zone=$(cat /etc/timezone)

if [ $arch != "x86_64" ] && [ $arch != "aarch64" ]; then
    red "Unknown architecture: $arch"
    exit 1
fi

pwd=$(pwd)
scriptdir=$(dirname $0)
cd $scriptdir
scriptdir=$(pwd)
prefix=$(basename $scriptdir)_

build_dir=$scriptdir/docker
common_dir=$scriptdir/cabot-common/docker
option="--progress=auto"
confirmation=1
build_image=1
build_workspace=1

export DOCKER_BUILDKIT=1
export DEBUG_FLAG="--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
export UNDERLAY_MIXINS="rel-with-deb-info"
export OVERLAY_MIXINS="rel-with-deb-info"
debug_ros2="--build-arg DEBUG_FLAG"

while getopts "hqnt:c:u:dyw" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	q)
	    option="-q $option"
	    ;;
	n)
	    option="--no-cache $option"
	    ;;
	t)
	    time_zone=$OPTARG
	    ;;
	d)
	    export DOCKER_BUILDKIT=0
	    ;;
	y)
	    confirmation=0
	    ;;
	w)
	    build_image=0
	    ;;
    esac
done
shift $((OPTIND-1))
targets=$@

#
# specify default targets if targets is not specified
# if targets include all set all targets
#
if [ -z "$targets" ]; then
    targets=$arch
elif [[ "$targets" =~ "all" ]]; then
    targets="x86_64 aarch64"
fi

CUDAV=11.7.1
CUDNNV=8
ROS2_UBUNTUV=22.04
ROS2_UBUNTU_DISTRO=jammy
ROS2_DISTRO=humble

#people     - jammy-cuda-humble-base                   nvidia
#people-l4t - l4t-ros-desktop

function check_to_proceed {
    if [[ "$targets" =~ "x86_64" ]]; then
	REQUIRED_DRIVERV=450.80.02

	DRIVERV=`nvidia-smi | grep "Driver"`
	re=".*Driver Version: ([0-9\.]+) .*"
	if [[ $DRIVERV =~ $re ]];then
	    DRIVERV=${BASH_REMATCH[1]}
	    echo "NVDIA driver version $DRIVERV is found"
	    if $(dpkg --compare-versions $DRIVERV ge $REQUIRED_DRIVERV); then
		blue "Installed NVIDIA driver satisfies the required version $REQUIRED_DRIVERV"
	    else
		red "Installed NVIDIA driver does not satisfy the required version $REQUIRED_DRIVERV"
		if [ $confirmation -eq 1 ]; then
		    read -p "Press enter to continue or terminate"
		fi
	    fi
	fi
    fi

    if [[ "$targets" =~ "aarch64" ]]; then
	if [ $arch = "aarch64" ]; then
	    blue "Building l4t image on aarch64 machine"
	elif [ $arch = "x86_64" ]; then
	    red "Building l4t image not on x86_64 machine"
	    if [ $(apt list qemu 2> /dev/null | grep installed | wc -l) -eq 1 ]; then
		red "It takes time to build l4t image on emulator. Do you want to proceed?"
		if [ $confirmation -eq 1 ]; then
		    read -p "Press enter to continue or terminate"
		fi
	    else
		red "You need to install arm emurator to build l4t image on "
		exit 1
	    fi
	fi
    fi
}


function build_ros_base_image {
    local FROM_IMAGE=$1
    local IMAGE_TAG_PREFIX=$2
    local UBUNTU_DISTRO=$3
    local ROS_DISTRO=$4
    local ROS_COMPONENT=$5
    local -n IMAGE_TAG=$6

    echo ""
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO
    blue "## build $IMAGE_TAG"
    pushd $build_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-core/

    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd

    echo ""
    FROM_IMAGE=$IMAGE_TAG
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO-base
    blue "## build $IMAGE_TAG"
    pushd $build_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-base/
    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd

    if [[ $ROS_COMPONENT = "ros-base" ]]; then
	returnn
    fi

    echo ""
    IMAGE_TAG=$IMAGE_TAG_PREFIX-$ROS_DISTRO-desktop
    blue "## build $IMAGE_TAG"
    pushd $build_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/desktop/
    sed s=FROM.*=FROM\ $FROM_IMAGE= Dockerfile > Dockerfile.temp && \
        docker build -f Dockerfile.temp -t $IMAGE_TAG $option .
    if [ $? -ne 0 ]; then
        red "failed to build $IMAGE_TAG"
        exit 1
    fi
    popd
}

function prebuild {
    local FROM_IMAGE=$1
    local IMAGE_BASE_NAME=$2
    local IMAGE_DIR=$3
    local -n IMAGE_TAG=$4         # output variable name

    IMAGE_TAG=$IMAGE_BASE_NAME-$(basename $IMAGE_DIR)

    pushd $IMAGE_DIR
    blue "## build $IMAGE_TAG"
    docker build -t $IMAGE_TAG \
	   --file Dockerfile \
	   --build-arg TZ=$time_zone \
	   --build-arg FROM_IMAGE=$FROM_IMAGE \
	   . && popd
}

function build_x86_64 {
    blue "- CUDAV=$CUDAV"
    blue "- CUDNNV=$CUDNNV"
    blue "- UBUNTUV=$ROS2_UBUNTUV"
    blue "- UBUNTU_DISTRO=$ROS2_UBUNTU_DISTRO"
    blue "- ROS_DISTRO=$ROS2_DISTRO"

    cuda_base=nvidia/cuda:$CUDAV-cudnn$CUDNNV-devel-ubuntu$ROS2_UBUNTUV
    base_name=${prefix}_$ROS2_UBUNTU_DISTRO-cuda$CUDAV-cudnn$CUDNNV-devel
    prebuild $cuda_base $base_name $build_dir/realsense image_tag
    if [ $? -ne 0 ]; then
	return 1
    fi

    base_name=${image_tag}
    blue "## build $base_name-$ROS2_DISTRO-desktop"
    build_ros_base_image $image_tag $image_tag $ROS2_UBUNTU_DISTRO $ROS2_DISTRO desktop image_tag

    prebuild $image_tag $base_name $build_dir/${ROS2_DISTRO}-custom image_tag
    if [ $? -ne 0 ]; then
	return 1
    fi

    prebuild $image_tag $image_tag $build_dir/opencv image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi

    prebuild $image_tag $image_tag $build_dir/open3d image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi

    prebuild $image_tag $image_tag $common_dir/mesa image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi

    local image=${prefix}_jammy-cuda11.7.1-cudnn8-devel-realsense-humble-custom-opencv-open3d-mesa
    docker compose build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   people

    if [[ $? -ne 0 ]]; then
	return 1
    fi

    docker compose -f docker-compose-test-rs3.yaml build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   rs1 rs2 rs3 track
}

function build_aarch64 {
    export DOCKER_BUILDKIT=0
    L4T_IMAGE="nvcr.io/nvidia/l4t-base:r35.1.0"

    echo ""
    local name1=${prefix}_l4t-realsense
    blue "# build ${prefix}_l4t-realsense"
    pushd $build_dir/realsense
    docker build -f Dockerfile.jetson -t $name1 \
	   --build-arg FROM_IMAGE=$L4T_IMAGE \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name1"
	exit 1
    fi
    popd

    echo ""
    local name2=${prefix}_l4t-realsense-opencv
    blue "# build ${prefix}_l4t-realsense-opencv"
    pushd $build_dir/opencv
    docker build -f Dockerfile.jetson -t $name2 \
	   --build-arg FROM_IMAGE=$name1 \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name2"
	exit 1
    fi
    popd

    echo ""
    local name3=${prefix}_l4t-realsense-opencv-humble-base
    blue "# build ${prefix}_l4t-realsense-opencv-humble-base"
    pushd $build_dir/jetson-humble-base-src
    docker build -t $name3 \
	   --build-arg FROM_IMAGE=$name2 \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name3"
	exit 1
    fi
    popd

    echo ""
    local name4=${prefix}_l4t-realsense-opencv-humble-custom
    blue "# build ${prefix}_l4t-realsense-opencv-humble-custom"
    pushd $build_dir/jetson-humble-custom
    docker build -t $name4 \
	   --build-arg FROM_IMAGE=$name3 \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name4"
	exit 1
    fi
    popd

    echo ""
    local name5=${prefix}_l4t-realsense-opencv-humble-custom-open3d
    blue "# build ${prefix}_l4t-realsense-opencv-humble-custom-open3d"
    pushd $build_dir/open3d
    docker build -f Dockerfile.jetson -t $name5 \
	   --build-arg FROM_IMAGE=$name4 \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name5"
	exit 1
    fi
    popd
    
    local image=${prefix}_l4t-realsense-opencv-humble-custom-open3d
    export DOCKER_BUILDKIT=0
    docker compose -f docker-compose-jetson.yaml build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   people-jetson

    if [[ $? -ne 0 ]]; then
	return 1
    fi

    docker compose -f docker-compose-jetson-test-rs3.yaml build \
		   --build-arg FROM_IMAGE=$image \
		   --build-arg UID=$UID \
		   --build-arg TZ=$time_zone \
		   $option \
		   rs1 rs2 rs3 track
}

function build_x86_64_ws {
    docker compose run --rm people /launch.sh build
}

function build_aarch64_ws {
    docker compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh build
}


blue "Targets: $targets"
check_to_proceed
for target in $targets; do
    if [[ $build_image -eq 1 ]]; then
	blue "# Building $target"
	eval "build_${target}"
	if [[ $? -ne 0 ]]; then
	    red "failed to build $target"
	    break
	fi
    fi
    if [[ $build_workspace -eq 1 ]]; then
	blue "# Building $target workspace"
	eval "build_${target}_ws"
    fi
done
