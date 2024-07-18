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

function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h                    show this help"
    echo "-p                    run profiler"
}

run_profiler=0
while getopts "hp" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        p)
            run_profiler=1
            ;;
    esac
done
shift $((OPTIND-1))
model_dir=$1

if [ -z "$model_dir" ]; then
    echo "please specify model directory"
    exit 1
fi
cd $model_dir

if [ ! -e "rtmdet/end2end.engine" ]; then
    # setting for input image size
    model_input_height=384
    # model_input_height=416
    # model_input_height=512
    # model_input_height=640
    if [ $model_input_height -eq 384 ]; then
        model_input_width=640
    else
        model_input_width=$model_input_height
    fi

    # settings for RTMDet-tiny model
    # model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet_tiny_8xb32-300e_coco/rtmdet_tiny_8xb32-300e_coco_20220902_112414-78e30dcc.pth
    # mmdet_config_filename=rtmdet_tiny_8xb32-300e_coco_reduce-postprocess.py

    # settings for RTMDet-s model
    model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet_s_8xb32-300e_coco/rtmdet_s_8xb32-300e_coco_20220905_161602-387a891e.pth
    mmdet_config_filename=rtmdet_s_8xb32-300e_coco_reduce-postprocess.py

    # settings for RTMDet-m model
    # model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet_m_8xb32-300e_coco/rtmdet_m_8xb32-300e_coco_20220719_112220-229f527c.pth
    # mmdet_config_filename=rtmdet_m_8xb32-300e_coco_reduce-postprocess.py

    # deploy RTMDet model
    mmdeploy_config_filename=detection_tensorrt-fp16_static-${model_input_height}x${model_input_width}.py
    model_filename=$(basename $model_url)
    if [ ! -e "rtmdet/$model_filename" ]; then
        echo "Downloading rtmdet/$model_filename"
        wget -P rtmdet $model_url
    else
        echo "You already have rtmdet/$model_filename"
    fi
    echo "prepare file for deployment..."
    sudo cp configs/mmdetection/$mmdet_config_filename /opt/mmdetection/configs/rtmdet/
    sudo cp configs/mmdeploy/$mmdeploy_config_filename /opt/mmdeploy/configs/mmdet/detection/

    echo "run model deployment..."
    python3 /opt/mmdeploy/tools/deploy.py \
        /opt/mmdeploy/configs/mmdet/detection/$mmdeploy_config_filename \
        /opt/mmdetection/configs/rtmdet/$mmdet_config_filename \
        rtmdet/$model_filename \
        /opt/mmdeploy/demo/resources/det.jpg \
        --work-dir ./rtmdet \
        --device cuda:0 \
        --dump-info

    if [ $run_profiler -eq 1 ]; then
        echo "run mmdeploy profiler..."
        python3 /opt/mmdeploy/tools/profiler.py \
            /opt/mmdeploy/configs/mmdet/detection/$mmdeploy_config_filename \
            /opt/mmdetection/configs/rtmdet/$mmdet_config_filename \
            /opt/mmdeploy/demo/resources \
            --model rtmdet/end2end.engine \
            --device cuda:0 \
            --shape ${model_input_height}x${model_input_width}

        echo "run mmdeploy SDK for mmdetection (mmdeploy/demo/csrc/cpp/detector.cxx)..."
        /usr/local/bin/detector $model_dir/rtmdet /opt/mmdeploy/demo/resources/det.jpg --device cuda

        echo "run mmdeploy SDK profiler for the output file of detector..."
        python3 /opt/mmdeploy/tools/sdk_analyze.py /tmp/profile.bin
    fi

    echo "Finished to deploy rtmdet/end2end.engine"
else
    echo "You already have rtmdet/end2end.engine"
fi

if [ ! -e "rtmdet-ins/end2end.engine" ]; then
    # setting for input image size
    model_input_height=384
    # model_input_height=416
    # model_input_height=512
    # model_input_height=640
    if [ $model_input_height -eq 384 ]; then
        model_input_width=640
    else
        model_input_width=$model_input_height
    fi

    # settings for RTMDet-Ins-tiny model
    # model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet-ins_tiny_8xb32-300e_coco/rtmdet-ins_tiny_8xb32-300e_coco_20221130_151727-ec670f7e.pth
    # mmdet_config_filename=rtmdet-ins_tiny_8xb32-300e_coco_reduce-postprocess.py

    # settings for RTMDet-Ins-s model
    model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet-ins_s_8xb32-300e_coco/rtmdet-ins_s_8xb32-300e_coco_20221121_212604-fdc5d7ec.pth
    mmdet_config_filename=rtmdet-ins_s_8xb32-300e_coco_reduce-postprocess.py

    # settings for RTMDet-Ins-m model
    # model_url=https://download.openmmlab.com/mmdetection/v3.0/rtmdet/rtmdet-ins_m_8xb32-300e_coco/rtmdet-ins_m_8xb32-300e_coco_20221123_001039-6eba602e.pth
    # mmdet_config_filename=rtmdet-ins_m_8xb32-300e_coco_reduce-postprocess.py

    # deploy RTMDet-Ins model
    mmdeploy_config_filename=instance-seg_rtmdet-ins_tensorrt-fp16_static-${model_input_height}x${model_input_width}.py
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
    echo "prepare file for deployment..."
    sudo cp configs/mmdetection/$mmdet_config_filename /opt/mmdetection/configs/rtmdet/
    sudo cp configs/mmdeploy/$mmdeploy_config_filename /opt/mmdeploy/configs/mmdet/instance-seg/

    echo "run model deployment (ignore error because visualize pytorch model will fail if input image size is not 640x640)..."
    python3 /opt/mmdeploy/tools/deploy.py \
        /opt/mmdeploy/configs/mmdet/instance-seg/$mmdeploy_config_filename \
        /opt/mmdetection/configs/rtmdet/$mmdet_config_filename \
        rtmdet-ins/$model_filename \
        /opt/mmdeploy/demo/resources/det.jpg \
        --work-dir ./rtmdet-ins \
        --device cuda:0 \
        --dump-info || true &&

    echo "fix deployed pipeline to enable is_crop_rtmdt_ins_mask option..."
    sed -i 's/\"is_resize_mask\": true/\"is_resize_mask\": false,\"is_crop_rtmdt_ins_mask\": true/g' rtmdet-ins/pipeline.json

    if [ $run_profiler -eq 1 ]; then
        echo "run mmdeploy profiler..."
        python3 /opt/mmdeploy/tools/profiler.py \
            /opt/mmdeploy/configs/mmdet/instance-seg/$mmdeploy_config_filename \
            /opt/mmdetection/configs/rtmdet/$mmdet_config_filename \
            /opt/mmdeploy/demo/resources \
            --model rtmdet-ins/end2end.engine \
            --device cuda:0 \
            --shape ${model_input_height}x${model_input_width}

        echo "run mmdeploy SDK for mmdetection (mmdeploy/demo/csrc/cpp/detector.cxx)..."
        /usr/local/bin/detector $model_dir/rtmdet-ins /opt/mmdeploy/demo/resources/det.jpg --device cuda

        echo "run mmdeploy SDK profiler for the output file of detector..."
        python3 /opt/mmdeploy/tools/sdk_analyze.py /tmp/profile.bin
    fi

    echo "Finished to deploy rtmdet-ins/end2end.engine"
else
    echo "You already have rtmdet-ins/end2end.engine"
fi
