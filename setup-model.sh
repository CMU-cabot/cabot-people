#!/bin/bash

# Copyright (c) 2021  IBM Corporation
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

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

if [[ -n $1 ]]; then
    # When you call this script from cabot project, set docker compose directory by input arguments
    dc_dir=$1
else
    dc_dir=$scriptdir
fi

cd $scriptdir/track_people_py/models

if [ ! -e "yolov4.cfg" ]; then
    echo "Downloading yolov4.cfg"
    wget https://raw.githubusercontent.com/AlexeyAB/darknet/yolov4/cfg/yolov4.cfg
else
    echo "You already have yolov4.cfg"
fi

if [ ! -e "yolov4.weights" ]; then
    echo "Downloading yolov4.weights"
    wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
else
    echo "You already have yolov4.weights"
fi

if [ ! -e "coco.names" ]; then
    echo "Downloading coco.names"
    wget https://raw.githubusercontent.com/AlexeyAB/darknet/yolov4/cfg/coco.names
else
    echo "You already have coco.names"
fi


if [ ! -e "rtmdet/end2end.engine" ]; then
    arch=$(uname -m)
    if [ $arch != "x86_64" ] && [ $arch != "aarch64" ]; then
        echo "Unknown architecture: $arch"
        exit 1
    fi
    dc_file=$dc_dir/docker-compose-people-setup-model.yaml
    if [ $arch = "x86_64" ]; then
        people_service=people-setup-model
    elif [ $arch = "aarch64" ]; then
        people_service=people-jetson-setup-model
    fi

    docker compose -f $dc_file run --rm $people_service bash -c "exit"
    if [[ $? -ne 0 ]]; then
        echo "build docker at first to prepare rtmdet model"
        exit 1
    fi

    # setting for input image size
    model_input_size=416
    # model_input_size=640

    # settings for RTMDet-tiny model
    model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet_tiny_8xb32-300e_coco/rtmdet_tiny_8xb32-300e_coco_20220902_112414-78e30dcc.pth
    mmdet_config_filename=rtmdet_tiny_8xb32-300e_coco_reduce-postprocess.py

    # settings for RTMDet-s model
    # model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet_s_8xb32-300e_coco/rtmdet_s_8xb32-300e_coco_20220905_161602-387a891e.pth
    # mmdet_config_filename=rtmdet_s_8xb32-300e_coco_reduce-postprocess.py

    # settings for RTMDet-m model
    # model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet_m_8xb32-300e_coco/rtmdet_m_8xb32-300e_coco_20220719_112220-229f527c.pth
    # mmdet_config_filename=rtmdet_m_8xb32-300e_coco_reduce-postprocess.py

    # deploy RTMDet model
    mmdeploy_config_filename=detection_tensorrt-fp16_static-${model_input_size}x${model_input_size}.py
    model_filename=$(basename $model_url)
    if [ ! -e "rtmdet/$model_filename" ]; then
        echo "Downloading rtmdet/$model_filename"
        wget -P rtmdet $model_url
    else
        echo "You already have rtmdet/$model_filename"
    fi
    docker compose -f $dc_file run --rm $people_service bash -c "
        echo 'prepare file for deployment...' &&
        sudo cp src/track_people_py/models/configs/mmdetection/$mmdet_config_filename /opt/mmdetection/configs/rtmdet/ &&
        sudo cp src/track_people_py/models/configs/mmdeploy/$mmdeploy_config_filename /opt/mmdeploy/configs/mmdet/detection/ &&

        echo 'run model deployment...' &&
        cd src/track_people_py/models &&
        python3 /opt/mmdeploy/tools/deploy.py \
            /opt/mmdeploy/configs/mmdet/detection/$mmdeploy_config_filename \
            /opt/mmdetection/configs/rtmdet/$mmdet_config_filename \
            rtmdet/$model_filename \
            /opt/mmdeploy/demo/resources/det.jpg \
            --work-dir ./rtmdet \
            --device cuda:0 \
            --dump-info &&

        echo 'run mmdeploy profiler...' &&
        python3 /opt/mmdeploy/tools/profiler.py \
            /opt/mmdeploy/configs/mmdet/detection/$mmdeploy_config_filename \
            /opt/mmdetection/configs/rtmdet/$mmdet_config_filename \
            /opt/mmdeploy/demo/resources \
            --model rtmdet/end2end.engine \
            --device cuda:0 \
            --shape ${model_input_size}x${model_input_size} &&

        echo 'run mmdeploy SDK for mmdetection (mmdeploy/demo/csrc/cpp/detector.cxx)...' &&
        /usr/local/bin/detector /home/developer/people_ws/src/track_people_py/models/rtmdet /opt/mmdeploy/demo/resources/det.jpg --device cuda &&

        echo 'install texttable for mmdeploy SDK profiler...' &&
        sudo pip3 install texttable &&

        echo 'run mmdeploy SDK profiler for the output file of detector...' &&
        python3 /opt/mmdeploy/tools/sdk_analyze.py /tmp/profile.bin"

    echo "Finished to deploy rtmdet/end2end.engine"
else
    echo "You already have rtmdet/end2end.engine"
fi

if [ ! -e "rtmdet-ins/end2end.engine" ]; then
    arch=$(uname -m)
    if [ $arch != "x86_64" ] && [ $arch != "aarch64" ]; then
        echo "Unknown architecture: $arch"
        exit 1
    fi
    dc_file=$dc_dir/docker-compose-people-setup-model.yaml
    if [ $arch = "x86_64" ]; then
        people_service=people-setup-model
    elif [ $arch = "aarch64" ]; then
        people_service=people-jetson-setup-model
    fi

    docker compose -f $dc_file run --rm $people_service bash -c "exit"
    if [[ $? -ne 0 ]]; then
        echo "build docker at first to prepare rtmdet-ins model"
        exit 1
    fi

    # setting for input image size
    model_input_size=416
    # model_input_size=640

    # settings for RTMDet-Ins-tiny model
    model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet-ins_tiny_8xb32-300e_coco/rtmdet-ins_tiny_8xb32-300e_coco_20221130_151727-ec670f7e.pth
    mmdet_config_filename=rtmdet-ins_tiny_8xb32-300e_coco_reduce-postprocess.py

    # settings for RTMDet-Ins-s model
    # model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet-ins_s_8xb32-300e_coco/rtmdet-ins_s_8xb32-300e_coco_20221121_212604-fdc5d7ec.pth
    # mmdet_config_filename=rtmdet-ins_s_8xb32-300e_coco_reduce-postprocess.py

    # settings for RTMDet-Ins-m model
    # model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet-ins_m_8xb32-300e_coco/rtmdet-ins_m_8xb32-300e_coco_20221123_001039-6eba602e.pth
    # mmdet_config_filename=rtmdet-ins_m_8xb32-300e_coco_reduce-postprocess.py

    # deploy RTMDet-Ins model
    mmdeploy_config_filename=instance-seg_rtmdet-ins_tensorrt-fp16_static-${model_input_size}x${model_input_size}.py
    model_filename=$(basename $model_url)
    if [ ! -e "rtmdet-ins/$model_filename" ]; then
        echo "Downloading rtmdet-ins/$model_filename"
        wget -P rtmdet-ins $model_url
    else
        echo "You already have rtmdet-ins/$model_filename"
    fi
    # set is_resize_mask option as false for RTMDet-Ins to avoid performance issue
    # note that the mask output in detector_output.jpg from profiler.py will not be correct because is_resize_mask is set as false
    # https://github.com/open-mmlab/mmdeploy/issues/2445#issuecomment-1725109397
    docker compose -f $dc_file run --rm $people_service bash -c "
        echo 'prepare file for deployment...' &&
        sudo cp src/track_people_py/models/configs/mmdetection/$mmdet_config_filename /opt/mmdetection/configs/rtmdet/ &&
        sudo cp src/track_people_py/models/configs/mmdeploy/$mmdeploy_config_filename /opt/mmdeploy/configs/mmdet/instance-seg/ &&

        echo 'run model deployment (ignore error because visualize pytorch model will fail if input image size is not 640x640)...' &&
        cd src/track_people_py/models &&
        python3 /opt/mmdeploy/tools/deploy.py \
            /opt/mmdeploy/configs/mmdet/instance-seg/$mmdeploy_config_filename \
            /opt/mmdetection/configs/rtmdet/$mmdet_config_filename \
            rtmdet-ins/$model_filename \
            /opt/mmdeploy/demo/resources/det.jpg \
            --work-dir ./rtmdet-ins \
            --device cuda:0 \
            --dump-info || true &&

        echo 'fix deployed pipeline to enable is_crop_rtmdt_ins_mask option...' &&
        sed -i 's/\"is_resize_mask\": true/\"is_resize_mask\": false,\"is_crop_rtmdt_ins_mask\": true/g' rtmdet-ins/pipeline.json &&

        echo 'run mmdeploy profiler...' &&
        python3 /opt/mmdeploy/tools/profiler.py \
            /opt/mmdeploy/configs/mmdet/instance-seg/$mmdeploy_config_filename \
            /opt/mmdetection/configs/rtmdet/$mmdet_config_filename \
            /opt/mmdeploy/demo/resources \
            --model rtmdet-ins/end2end.engine \
            --device cuda:0 \
            --shape ${model_input_size}x${model_input_size} &&

        echo 'run mmdeploy SDK for mmdetection (mmdeploy/demo/csrc/cpp/detector.cxx)...' &&
        /usr/local/bin/detector /home/developer/people_ws/src/track_people_py/models/rtmdet-ins /opt/mmdeploy/demo/resources/det.jpg --device cuda &&

        echo 'install texttable for mmdeploy SDK profiler...' &&
        sudo pip3 install texttable &&

        echo 'run mmdeploy SDK profiler for the output file of detector...' &&
        python3 /opt/mmdeploy/tools/sdk_analyze.py /tmp/profile.bin"

    echo "Finished to deploy rtmdet-ins/end2end.engine"
else
    echo "You already have rtmdet-ins/end2end.engine"
fi
