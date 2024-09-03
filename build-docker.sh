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
    echo "-p                    prebuild images"
    echo "-P <prefix>           prebuild with prefix"
    echo "-i                    build images"
    echo "-w                    build workspace"
    echo "-c                    camera target (default=\"realsense framos\", set \"realsense\" for RealSense, and \"framos\" for FRAMOS camera)"
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
prefix=$(basename $scriptdir)

build_dir=$scriptdir/docker
common_dir=$scriptdir/cabot-common/docker
option="--progress=auto"
confirmation=1
prebuild=0
build_image=0
build_workspace=0
camera_targets="realsense framos"

export DOCKER_BUILDKIT=1
export DEBUG_FLAG="--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
export UNDERLAY_MIXINS="rel-with-deb-info"
export OVERLAY_MIXINS="rel-with-deb-info"
debug_ros2="--build-arg DEBUG_FLAG"

while getopts "hqno:t:u:dypP:iwc:" arg; do
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
    o)
        option=$OPTARG
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
    p)
        prebuild=1
        ;;
    P)
        prebuild=1
        prefix=${OPTARG}
        ;;
    i)
        build_image=1
        ;;
	w)
	    build_workspace=1
	    ;;
	c)
	    camera_targets=$OPTARG
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

if [[ "$camera_targets" =~ "all" ]]; then
    camera_targets="realsense framos"
fi

CUDAV=11.8.0
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
    pushd $common_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-core/

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
    pushd $common_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/ros-base/
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
    pushd $common_dir/docker_images/ros/$ROS_DISTRO/ubuntu/$UBUNTU_DISTRO/desktop/
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

function prebuild_x86_64 {
    local CAMERA_IMAGE=$1
    blue "- CUDAV=$CUDAV"
    blue "- CUDNNV=$CUDNNV"
    blue "- UBUNTUV=$ROS2_UBUNTUV"
    blue "- UBUNTU_DISTRO=$ROS2_UBUNTU_DISTRO"
    blue "- CAMERA_IMAGE=$CAMERA_IMAGE"

    cuda_base=nvidia/cuda:${CUDAV}-cudnn${CUDNNV}-devel-ubuntu${ROS2_UBUNTUV}
    base_name=${prefix}__${ROS2_UBUNTU_DISTRO}-cuda${CUDAV}-cudnn${CUDNNV}-devel
    prebuild $cuda_base $base_name $build_dir/$CAMERA_IMAGE image_tag
    if [ $? -ne 0 ]; then
	return 1
    fi

    base_name=${image_tag}
    blue "## build $base_name-$ROS2_DISTRO-desktop"
    build_ros_base_image $image_tag $image_tag $ROS2_UBUNTU_DISTRO $ROS2_DISTRO desktop image_tag

    prebuild $image_tag $base_name $common_dir/${ROS2_DISTRO}-custom image_tag
    if [ $? -ne 0 ]; then
	return 1
    fi

    prebuild $image_tag $image_tag $build_dir/opencv image_tag
    if [ $? -ne 0 ]; then
	red "failed to build $image_tag"
	return 1
    fi

    prebuild $image_tag $image_tag $build_dir/mmdeploy image_tag
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
}

function build_x86_64 {
    local CAMERA_IMAGE=$1

    if [[ $CAMERA_IMAGE == 'realsense' ]]; then
        local image=${prefix}__${ROS2_UBUNTU_DISTRO}-cuda${CUDAV}-cudnn${CUDNNV}-devel-realsense-humble-custom-opencv-mmdeploy-open3d-mesa
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
        if [[ $? -ne 0 ]]; then
            return 1
        fi
    elif [[ $CAMERA_IMAGE == 'framos' ]]; then
        local image=${prefix}__${ROS2_UBUNTU_DISTRO}-cuda${CUDAV}-cudnn${CUDNNV}-devel-framos-humble-custom-opencv-mmdeploy-open3d-mesa
        docker compose build \
            --build-arg FROM_IMAGE=$image \
            --build-arg UID=$UID \
            --build-arg TZ=$time_zone \
            $option \
            people-framos
        if [[ $? -ne 0 ]]; then
            return 1
        fi

        docker compose -f docker-compose-test-rs3-framos.yaml build \
            --build-arg FROM_IMAGE=$image \
            --build-arg UID=$UID \
            --build-arg TZ=$time_zone \
            $option \
            rs1-framos-camera rs1-framos-detection rs2-framos-camera rs2-framos-detection rs3-framos-camera rs3-framos-detection track-framos
        if [[ $? -ne 0 ]]; then
            return 1
        fi
    fi
}

function prebuild_aarch64 {
    local CAMERA_IMAGE=$1
    export DOCKER_BUILDKIT=0
    blue "- CAMERA_IMAGE=$CAMERA_IMAGE"

    # check host OS JetPack version
    HOST_L4T_V=$(dpkg-query --showformat='${Version}' --show nvidia-l4t-core)
    HOST_L4T_V_ARRAY=(${HOST_L4T_V//./ })
    HOST_L4T_RELEASE_V=${HOST_L4T_V_ARRAY[0]}
    blue "- HOST_L4T_RELEASE_V=$HOST_L4T_RELEASE_V"

    if [[ $HOST_L4T_RELEASE_V -lt 36 ]]; then
        L4T_IMAGE="nvcr.io/nvidia/l4t-base:35.3.1"
        OPENCV_V=4.5.4
        L4T_CUDA=11-4
    else
        L4T_IMAGE="nvcr.io/nvidia/l4t-base:r36.2.0"
        OPENCV_V=4.9.0
        L4T_CUDA=12-2
    fi
    L4T_MAJOR_MINOR_V="${HOST_L4T_V_ARRAY[0]}.${HOST_L4T_V_ARRAY[1]}"
    TEGRA_V=t234
    blue "- L4T_IMAGE=$L4T_IMAGE"
    blue "- OPENCV_V=$OPENCV_V"
    blue "- L4T_CUDA=$L4T_CUDA"
    blue "- L4T_MAJOR_MINOR_V=$L4T_MAJOR_MINOR_V"
    blue "- TEGRA_V=$TEGRA_V"

    echo ""
    local name1=${prefix}_l4t-$CAMERA_IMAGE
    blue "# build ${prefix}_l4t-$CAMERA_IMAGE"
    pushd $build_dir/$CAMERA_IMAGE
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
    local name2=${prefix}_l4t-$CAMERA_IMAGE-opencv
    blue "# build ${prefix}_l4t-$CAMERA_IMAGE-opencv"
    pushd $build_dir/opencv
    docker build -f Dockerfile.jetson -t $name2 \
	   --build-arg FROM_IMAGE=$name1 \
	   --build-arg OPENCV_V=$OPENCV_V \
	   --build-arg CUDA_V=$L4T_CUDA \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name2"
	exit 1
    fi
    popd

    echo ""
    local name3=${prefix}_l4t-$CAMERA_IMAGE-opencv-humble-base
    blue "# build ${prefix}_l4t-$CAMERA_IMAGE-opencv-humble-base"
    pushd $common_dir/jetson-humble-base-src
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
    local name4=${prefix}_l4t-$CAMERA_IMAGE-opencv-humble-custom
    blue "# build ${prefix}_l4t-$CAMERA_IMAGE-opencv-humble-custom"
    pushd $build_dir/jetson-humble-custom
    if [[ $CAMERA_IMAGE == 'realsense' ]]; then
        docker build -t $name4 \
        --build-arg FROM_IMAGE=$name3 \
        $option \
        .
    elif [[ $CAMERA_IMAGE == 'framos' ]]; then
        docker build -f Dockerfile.framos -t $name4 \
        --build-arg FROM_IMAGE=$name3 \
        $option \
        .
    fi
    if [ $? -ne 0 ]; then
	red "failed to build $name4"
	exit 1
    fi
    popd

    echo ""
    local name5=${prefix}_l4t-$CAMERA_IMAGE-opencv-humble-custom-mmdeploy
    blue "# build ${prefix}_l4t-$CAMERA_IMAGE-opencv-humble-custom-mmdeploy"
    pushd $build_dir/mmdeploy
    docker build -f Dockerfile.jetson -t $name5 \
	   --build-arg FROM_IMAGE=$name4 \
	   --build-arg CUDA_V=$L4T_CUDA \
	   --build-arg L4T_V=$L4T_MAJOR_MINOR_V \
	   --build-arg TEGRA_V=$TEGRA_V \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name5"
	exit 1
    fi
    popd

    echo ""
    local name6=${prefix}_l4t-$CAMERA_IMAGE-opencv-humble-custom-mmdeploy-open3d
    blue "# build ${prefix}_l4t-$CAMERA_IMAGE-opencv-humble-custom-mmdeploy-open3d"
    pushd $build_dir/open3d
    docker build -f Dockerfile.jetson -t $name6 \
	   --build-arg FROM_IMAGE=$name5 \
	   $option \
	   .
    if [ $? -ne 0 ]; then
	red "failed to build $name6"
	exit 1
    fi
    popd
}

function build_aarch64 {
    local CAMERA_IMAGE=$1
    export DOCKER_BUILDKIT=0

    if [[ $CAMERA_IMAGE == 'realsense' ]]; then
        local image=${prefix}_l4t-realsense-opencv-humble-custom-mmdeploy-open3d
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
        if [[ $? -ne 0 ]]; then
            return 1
        fi
    elif [[ $CAMERA_IMAGE == 'framos' ]]; then
        local image=${prefix}_l4t-framos-opencv-humble-custom-mmdeploy-open3d
        docker compose -f docker-compose-jetson.yaml build \
            --build-arg FROM_IMAGE=$image \
            --build-arg UID=$UID \
            --build-arg TZ=$time_zone \
            $option \
            people-framos-jetson
        if [[ $? -ne 0 ]]; then
            return 1
        fi

        docker compose -f docker-compose-jetson-test-rs3-framos.yaml build \
            --build-arg FROM_IMAGE=$image \
            --build-arg UID=$UID \
            --build-arg TZ=$time_zone \
            $option \
            rs1-framos-camera rs1-framos-detection rs2-framos-camera rs2-framos-detection rs3-framos-camera rs3-framos-detection track-framos
        if [[ $? -ne 0 ]]; then
            return 1
        fi
    fi
}

function build_ws {
    local CAMERA_IMAGE=$1

    if [[ $CAMERA_IMAGE == 'realsense' ]]; then
        docker compose run --rm people /launch.sh build
    elif [[ $CAMERA_IMAGE == 'framos' ]]; then
        docker compose run --rm people-framos /launch.sh build
    fi
}

blue "Targets: $targets"
blue "Camera targets: $camera_targets"
check_to_proceed
for target in $targets; do
    for camera_target in $camera_targets; do
        if [ $camera_target != "realsense" ] && [ $camera_target != "framos" ]; then
            red "invalid camera target $camera_target"
            exit 1
        fi

        if [[ $prebuild -eq 1 ]]; then
            blue "# Building prebuild $target $camera_target"
            eval "prebuild_${target} ${camera_target}"
            if [[ $? -ne 0 ]]; then
                red "failed to prebuild $target $camera_target"
                break
            fi
        fi
        if [[ $build_image -eq 1 ]]; then
            blue "# Building $target $camera_target"
            eval "build_${target} ${camera_target}"
            if [[ $? -ne 0 ]]; then
                red "failed to build $target $camera_target"
                break
            fi
        fi
        if [[ $build_workspace -eq 1 ]]; then
            blue "# Building $target $camera_target workspace"
            eval "build_ws ${camera_target}"
        fi
    done
done
