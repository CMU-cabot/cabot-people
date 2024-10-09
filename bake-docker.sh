#!/usr/bin/env bash

# Copyright (c) 2024  Carnegie Mellon University
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

# This script overview:
# - Builds the necessary images for cabot on multiple platforms and pushes them to cmucal.
#   - Since push requires cmucal permissions, it is usually handled by an Action script.
# - If you want to debug by modifying the base image, perform a local build (-l).
#   - Set up a local registry server, build, and overwrite the cmucal image tag.
#   - It is also advisable to specify the platform (-p) in this case.

function help {
    echo "Usage: $0 [-i] [-l] [-b <base_name>] [-P <platform>]"
    echo ""
    echo "-h                    show this help"
    echo "-b <base_name>        bake with base_name"
    echo "-n <nuc_base_name>    bake with nuc_base_name"
    echo "-s <services>         target services (default=\"people people-framos people-nuc\", set more than one services from \"people\", \"people-framos\", \"people-nuc\")"
    echo "-i                    image build for debug - shorthand for -l and -P with host platform"
    echo "-l                    build using local registry"
    echo "-P <platform>         specify platform"
    echo "                      build linux/arm64 and linux/amd64 if not specified"
    echo "-t <tags>             additional tags"
}

platform=
base_name=cabot-gpu-base
nuc_base_name=cabot-base
image_name=cabot-people
local=0
tags=
services="people people-framos people-nuc"

while getopts "hb:n:s:ilP:t:" arg; do
    case $arg in
    h)
        help
        exit
        ;;
    b)
        base_name=${OPTARG}
        ;;
    n)
        nuc_base_name=${OPTARG}
        ;;
    s)
        services=${OPTARG}
        ;;
    i)
        if [[ $(uname -m) = "x86_64" ]]; then
            platform="linux/amd64"
        elif [[ $(uname -m) = "aarch64" ]]; then
            platform="linux/arm64"
        fi
        local=1
        ;;
    l)
        local=1
        ;;
    P)
        platform=${OPTARG}
        ;;
    t)
        tags=${OPTARG}
        ;;
    esac
done
shift $((OPTIND-1))

if [[ -z $base_name ]] || [[ -z $nuc_base_name ]]; then
    help
    exit 1
fi

# check if all models exist
has_all_models=1
m_dir="./track_people_py/models"
if [ ! -e "${m_dir}/yolov4.cfg" ] || [ ! -e "${m_dir}/yolov4.weights" ] || [ ! -e "${m_dir}/coco.names" ]; then
    echo "You do not have darknet model"
    has_all_models=0
fi
for a_dir in "x86_64" "aarch64"; do
    if [ ! -e "${m_dir}/rtmdet/${a_dir}/deploy.json" ] || [ ! -e "${m_dir}/rtmdet/${a_dir}/pipeline.json" ] || [ ! -e "${m_dir}/rtmdet/${a_dir}/end2end.engine" ]; then
        echo "You do not have rtmdet model for architecture $a_dir"
        has_all_models=0
    fi
    if [ ! -e "${m_dir}/rtmdet-ins/${a_dir}/deploy.json" ] || [ ! -e "${m_dir}/rtmdet-ins/${a_dir}/pipeline.json" ] || [ ! -e "${m_dir}/rtmdet-ins/${a_dir}/end2end.engine" ]; then
        echo "You do not have rtmdet-ins model for architecture $a_dir"
        has_all_models=0
    fi
done
if [ $has_all_models -eq 0 ]; then
    read -p "Built docker images will not have all detection models. Press enter to continue, or terminate and build workspace in each architecture to prepare models"
fi

export ROS_DISTRO=humble
export UBUNTU_DISTRO=jammy

if [[ -z $(docker network ls | grep "registry-network") ]]; then
    docker network create registry-network
fi
if [[ $local -eq 1 ]]; then
    export REGISTRY=registry:5000
    # setup local docker registry for multiplatform support
    if [[ -z $(docker ps -f "name=registry" -q) ]]; then
        docker run -d \
        --rm \
            --name registry \
            --network registry-network \
            -p 127.0.0.1:9092:5000 \
            registry:2.7
    fi
else
    export REGISTRY=cmucal
fi

# setup multiplatform builder
# docker buildx rm mybuilder
if [[ -z $(docker buildx ls | grep "mybuilder\*") ]]; then
    echo "mybuilder is not selected"
    if [[ -z $(docker buildx ls | grep mybuilder) ]]; then
        echo "creating mybuilder"
        docker buildx create --use --name mybuilder --driver docker-container \
           --config buildkitd.toml \
           --driver-opt network=registry-network  # option to make the builder access to the registry on the localhost
    else
        echo "use mybuilder"
        docker buildx use mybuilder
    fi
fi

# replace ros Dockerfile FROM instruction to replace base image
# this isn't good, but cannot find alternative
if [[ -e ./cabot-common/docker/docker_images ]]; then
    while read -r line; do
        sed 's=FROM.*=ARG\ FROM_IMAGE\
    FROM\ $FROM_IMAGE=' "$line" > "$line.tmp"
    done < <(find ./cabot-common/docker/docker_images/ -wholename "*/$ROS_DISTRO/*/Dockerfile")
fi

# camera option
camera_option=
cameras=
for service in $services; do
    if [[ $service == "people" ]]; then
        if [[ -n $cameras ]]; then
            cameras+=" "
        fi
        cameras+="realsense"
    elif [[ $service == "people-framos" ]]; then
        if [[ -n "$cameras" ]]; then
            cameras+=" "
        fi
        cameras+="framos"
    fi
done
if [[ -n $cameras ]]; then
    camera_option='CAMERAS="$cameras"'
fi

# platform option
platform_option=
if [[ $platform = "linux/amd64" ]] || [[ $platform = "linux/arm64" ]]; then
    platform_option="--set=*.platform=\"$platform\""
fi

# target
target=
if [[ $platform = "linux/amd64" ]]; then
    target="targets-amd64"
elif [[ $platform = "linux/arm64" ]]; then
    target="targets-arm64"
fi

# run bake for cabot-gpu-base images
if [[ -n $camera_option ]]; then
    com="$camera_option docker buildx bake -f docker-bake.hcl $platform_option $tag_option $target"
    export BASE_IMAGE=$base_name
    export NUC_BASE_IMAGE=$nuc_base_name
    echo $com
    eval $com
    if [[ $? -ne 0 ]]; then
        exit 1
    fi

    # create multi platform cabot-people final base images
    if [[ $local -eq 1 ]]; then
        final_registry="localhost:9092"
    else
        final_registry="cmucal"
    fi
    for camera in $cameras; do
        if [[ $platform = "linux/amd64" ]]; then
            sources="${final_registry}/${base_name}:${camera}-final-amd64"
        elif [[ $platform = "linux/arm64" ]]; then
            sources="${final_registry}/${base_name}:${camera}-final-arm64"
        else
            sources="${final_registry}/${base_name}:${camera}-final-amd64 ${final_registry}/${base_name}:${camera}-final-arm64"
        fi

        com="docker buildx imagetools create --tag ${final_registry}/${base_name}:${camera}-final ${sources}"
        echo $com
        eval $com
        if [[ $? -ne 0 ]]; then
            exit 1
        fi
    done
else
    echo "Specified target services do not need GPU, skip build cabot-gpu-base images"
fi

# tag option
tag_option=
if [[ -n $tags ]]; then
    for service in $services; do
        if [[ -n $tag_option ]]; then
            tag_option+=" "
        fi
        tag_option+="--set=${service}.tags=${REGISTRY}/cabot-${service}:{${tags}}"
    done
fi

# run bake for cabot-people images
com="docker buildx bake -f docker-compose.yaml $platform_option $tag_option $services $@"
export BASE_IMAGE=$base_name
export NUC_BASE_IMAGE=$nuc_base_name
echo $com
eval $com

# copy images from local registry
# this can override image tag
if [[ $local -eq 1 ]]; then
    tags=($(eval "$com --print" 2> /dev/null | jq -r '.target[].tags[] | split("/")[-1]' | jq --raw-input | jq -r --slurp 'join(" ")'))
    for tag in "${tags[@]}"; do
        echo "Pulling tag ($tag) from $REGISTRY (platform=$(uname -m))"
        com="docker pull localhost:9092/$tag"
        echo $com
        eval $com
        com="docker image tag localhost:9092/$tag cmucal/$tag"
        echo $com
        eval $com
    done
fi

# reset buildx builder to default
docker buildx use default
