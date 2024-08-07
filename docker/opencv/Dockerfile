# Copyright (c) 2022  Carnegie Mellon University
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

ARG FROM_IMAGE

FROM $FROM_IMAGE AS build

# CMake 3.19+ is required to build Open3D
ARG CMAKE_V=3.19.3 \
	OPENCV_V=4.5.4 \
	COMP_CAP=6.1,7.5,8.6

ENV DEBIAN_FRONTEND="noninteractive"


RUN apt update && \
	apt install -y --no-install-recommends \
	libtbb2 \
	libtbb2-dev \
	libhdf5-103-1 \
	libhdf5-dev \
	libjpeg8 \
	libjpeg8-dev \
	libpng16-16 \
	libpng-dev \
	libtiff5 \
	libtiff-dev \
	libwebp7 \
	libwebp-dev \
	libopenjp2-7 \
	libopenjp2-7-dev \
        libeigen3-dev \
	python3 \
	python3-dev \
	python3-numpy \
#	python-tk \
#	git \
#        libfreetype6-dev \
#        libglu1-mesa-dev \
	python3-pip \
#	python3-vcstool \
#	rsync \
#	software-properties-common \
	unzip \
#	usbutils \
	wget \
#	xorg-dev \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

# install cmake
RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_V}/cmake-${CMAKE_V}-linux-x86_64.sh && \
    sh cmake-${CMAKE_V}-linux-x86_64.sh --skip-license --prefix=/usr && \
    rm cmake-${CMAKE_V}-linux-x86_64.sh

# install opencv        
RUN mkdir -p src/opencv/build && cd src/opencv && \
	wget https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_V}.zip && \
	unzip ${OPENCV_V}.zip && rm ${OPENCV_V}.zip && \
	wget https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_V}.zip && \
	unzip ${OPENCV_V}.zip && rm ${OPENCV_V}.zip && \
	cd build && \
	cmake \
	-D BUILD_LIST=core,cudev,dnn,imgproc,imgcodecs,calib3d,highgui,python3,video,videoio \
	-D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D WITH_TBB=ON \
	-D WITH_CUDA=ON \
	-D WITH_CUDNN=ON \
	-D OPENCV_DNN_CUDA=ON \
	-D CUDA_ARCH_BIN=${COMP_CAP} \
	-D BUILD_opencv_cudacodec=OFF \
	-D ENABLE_FAST_MATH=1 \
	-D CUDA_FAST_MATH=1 \
	-D WITH_CUBLAS=1 \
	-D WITH_V4L=ON \
	-D WITH_QT=ON \
	-D WITH_OPENGL=ON \
	-D WITH_GSTREAMER=ON \
	-D WITH_FFMPEG=ON \
	-D OPENCV_GENERATE_PKGCONFIG=ON \
	-D OPENCV_PC_FILE_NAME=opencv4.pc \
	-D OPENCV_ENABLE_NONFREE=OFF \
	-D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib-${OPENCV_V}/modules \
	-D PYTHON_DEFAULT_EXECUTABLE=$(which python3) \
	-D BUILD_EXAMPLES=OFF \
	../opencv-${OPENCV_V} && \
	make -j$(nproc) && \
	make install && \
	ldconfig && \
	cd ../../ && \
	rm -rf opencv

