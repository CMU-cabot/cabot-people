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

skip_model=0
while getopts "s" arg; do
    case $arg in
    s)
        skip_model=1
        ;;
    esac
done
shift $((OPTIND-1))

WS=$HOME/people_ws

if [[ $skip_model -eq 0 ]]; then
    arch=$(uname -m)
    if { [ $arch = "x86_64" ] && [ -n "$(which nvidia-smi)" ]; } || [ $arch = "aarch64" ]; then
        echo "Setup model"
        /setup-model.sh $WS/src/track_people_py/models
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
cd $WS
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
