#!/bin/bash

# Copyright (c) 2024  Carnegie Mellon University, IBM Corporation, and others
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

debug=0
sequential=0
skip_model=0

while getopts "dsm" arg; do
    case $arg in
        d)
            debug=1
            ;;
        s)
            sequential=1
            ;;
        m)
            skip_model=1
            ;;
    esac
done
shift $((OPTIND-1))

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

if [[ $skip_model -eq 0 ]]; then
    arch=$(uname -m)
    if { [ $arch = "x86_64" ] && [ -n "$(which nvidia-smi)" ]; } || [ $arch = "aarch64" ]; then
        echo "Setup model"
        /setup-model.sh $scriptdir/../../track_people_py/models
        if [ $? -ne 0 ]; then
            echo "Failed to setup model"
            exit 1
        fi
    else
        echo "Failed to setup model. GPU is not detected."
    fi
else
    echo "Skip to setup model."
fi

echo "Build workspace"
cd $scriptdir/../../../

build_option=
if [[ $debug -eq 1 ]]; then
    build_option+=" --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install"
else
    build_option+=" --cmake-args -DCMAKE_BUILD_TYPE=Release"
fi
if [[ $sequential -eq 1 ]]; then
    build_option+=" --executor sequential"
fi
colcon build $build_option
