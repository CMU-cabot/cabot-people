# Copyright (c) 2023  IBM Corporation and Carnegie Mellon University
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

# build cache image for mmdeploy, mmdetection
FROM $FROM_IMAGE AS cache

ARG TORCH_CUDA_ARCH_LIST="7.5;8.6;12.0"
ARG CUDA_CMAKE_ARCHITECTURES="75;86;120"

# install TensorRT packages
RUN TENSORRT_DEB_V="10.10.0.31-1+cuda12.9" && \
	apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub && \
	apt update && \
	apt-get install -y --no-install-recommends \
	libnvinfer-dev=${TENSORRT_DEB_V} \
	libnvinfer-headers-dev=${TENSORRT_DEB_V} \
	libnvinfer-headers-plugin-dev=${TENSORRT_DEB_V} \
	libnvinfer-plugin-dev=${TENSORRT_DEB_V} \
	libnvinfer-plugin10=${TENSORRT_DEB_V} \
	libnvinfer-vc-plugin10=${TENSORRT_DEB_V} \
	libnvinfer10=${TENSORRT_DEB_V} \
	libnvonnxparsers10=${TENSORRT_DEB_V} \ 
	python3-libnvinfer=${TENSORRT_DEB_V} \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

# install mmdeploy requirements
RUN pip3 install --no-cache-dir torch==2.8.0 torchvision==0.23.0 --index-url https://download.pytorch.org/whl/cu129 && \
	pip3 install --no-cache-dir openmim==0.3.9 && \
	# upgrade setuptools to build mmdeploy
	pip3 install --no-cache-dir setuptools==69.0.2 && \
	# install all requirements for mmengine except opencv-python
	pip3 install --no-cache-dir addict termcolor pyyaml matplotlib yapf numpy rich && \
	# install mmengine without opencv-python to use opencv-python built from source
	pip3 install --no-cache-dir --no-deps mmengine==0.10.3 && \
	# install psutil, ninja for fast build of python packages
	pip3 install --no-cache-dir psutil==5.9.6 ninja==1.11.1.1 && \
	pip3 install --no-cache-dir pycuda==2023.1

# install mmcv
RUN git clone --branch v2.1.0 https://github.com/open-mmlab/mmcv.git && \
	cd mmcv && \
	FORCE_CUDA=1 MMCV_WITH_OPS=1 TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST}" \
	python3 setup.py bdist_wheel -d /tmp/wheelhouse && \
	# install all requirements for mmcv except opencv-python, mmengine
	pip3 install --no-cache-dir yapf addict pyyaml Pillow numpy packaging termcolor matplotlib rich "platformdirs>=3.5.1" \
		"importlib-metadata>=6.6.0" "tomli>=2.0.1" "zipp>=0.5" "pygments<3.0.0,>=2.13.0" "markdown-it-py>=2.2.0" "mdurl~=0.1" && \
	# install mmcv without opencv-python to use opencv-python built from source
	pip3 install --no-cache-dir --no-deps /tmp/wheelhouse/mmcv-*.whl && \
	cd ../ && \
	rm -rf mmcv

# workaround to to find TensorRT directory when building mmdeploy
# https://github.com/open-mmlab/mmdeploy/issues/2514
ENV TENSORRT_DIR=/opt/tensorrt
RUN mkdir $TENSORRT_DIR && \
	ln -sf /usr/lib/x86_64-linux-gnu $TENSORRT_DIR/lib && \
	ln -sf /usr/include/x86_64-linux-gnu $TENSORRT_DIR/include

# workaround to find libcuda.so when building mmdeploy
# https://github.com/open-mmlab/mmdeploy/issues/246
ENV BACKUP_LD_LIBRARY_PATH=$LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda/compat:$LD_LIBRARY_PATH

# build ppl.cv
RUN cd /opt && \
	# use ppl.cv version which supports CUDA 12
	git clone https://github.com/VBTI-development/ppl.cv.git && \
	cd ppl.cv && \
	git checkout v0.8.0 && \
	mkdir cuda-build && cd cuda-build && \
	cmake .. \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_CUDA_ARCHITECTURES="${CUDA_CMAKE_ARCHITECTURES}" \
	-DPPLCV_USE_CUDA=ON \
	-DCMAKE_INSTALL_PREFIX=/opt/ppl.cv/cuda-build/install && \
	cmake --build . -j$(nproc) --config Release && \
	cmake --build . --target install -j$(nproc) --config Release && \
	# build mmdeploy model converter
	cd /opt && \
	# use mmdeploy version which supports CUDA 12
	git clone -b cabot-main https://github.com/CMU-cabot/onedl-mmdeploy.git --recursive mmdeploy && \
	cd mmdeploy && \
	mkdir build && cd build && \
	cmake .. \
	-DCMAKE_INSTALL_PREFIX=/usr/mmdeploy \
	-DCMAKE_CUDA_ARCHITECTURES="${CUDA_CMAKE_ARCHITECTURES}" \
	-DTENSORRT_DIR=${TENSORRT_DIR} \
	-DMMDEPLOY_TARGET_BACKENDS="trt" && \
	make -j$(nproc) && make install && \
	cd ../ && \
	python3 setup.py bdist_wheel -d /tmp/wheelhouse && \
	pip3 install --no-cache-dir --no-deps /tmp/wheelhouse/onedl_mmdeploy-*.whl && \
	rm -rf build && \
	# build mmdeploy SDK
	mkdir build && cd build && \
	cmake .. \
	-DCMAKE_INSTALL_PREFIX=/usr/mmdeploy \
	-DCMAKE_CUDA_ARCHITECTURES="${CUDA_CMAKE_ARCHITECTURES}" \
	-DMMDEPLOY_BUILD_SDK=ON \
	-DMMDEPLOY_BUILD_EXAMPLES=ON \
	-DCMAKE_CXX_COMPILER=g++ \
	-Dpplcv_DIR=/opt/ppl.cv/cuda-build/install/lib/cmake/ppl \
	-DTENSORRT_DIR=${TENSORRT_DIR} \
	-DMMDEPLOY_BUILD_SDK_MONOLITHIC=ON \
	-DMMDEPLOY_BUILD_SDK_PYTHON_API=ON \
	-DMMDEPLOY_TARGET_DEVICES="cuda;cpu" \
	-DMMDEPLOY_TARGET_BACKENDS="trt" \
	-DMMDEPLOY_CODEBASES="mmdet" && \
	make -j$(nproc) && make install && \
	# copy mmdeploy_runtime library manually to use mmdeploy_runtime python package
	mkdir -p /usr/mmdeploy_runtime/lib && \
	cp lib/mmdeploy_runtime*.so /usr/mmdeploy_runtime/lib/ && \
	cd ../../ && \
	rm -rf ppl.cv mmdeploy

ENV LD_LIBRARY_PATH=${BACKUP_LD_LIBRARY_PATH}

# install mmdetection requirements
RUN pip3 install --no-cache-dir pycocotools==2.0.7 shapely==2.0.4 texttable==1.7.0

# install mmdetection
RUN cd /opt && \
	git clone -b v3.3.0 https://github.com/open-mmlab/mmdetection.git && \
	cd mmdetection && \
	python3 setup.py bdist_wheel -d /tmp/wheelhouse && \
	pip3 install --no-cache-dir --no-deps /tmp/wheelhouse/mmdet-*.whl && \
	cd ../ && \
	rm -rf mmdetection

# copy mmdeploy, mmdetection from cache image
FROM $FROM_IMAGE AS build

# install TensorRT packages
RUN TENSORRT_DEB_V="10.10.0.31-1+cuda12.9" && \
	apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub && \
	apt update && \
	apt-get install -y --no-install-recommends \
	libnvinfer-dev=${TENSORRT_DEB_V} \
	libnvinfer-headers-dev=${TENSORRT_DEB_V} \
	libnvinfer-headers-plugin-dev=${TENSORRT_DEB_V} \
	libnvinfer-plugin-dev=${TENSORRT_DEB_V} \
	libnvinfer-plugin10=${TENSORRT_DEB_V} \
	libnvinfer-vc-plugin10=${TENSORRT_DEB_V} \
	libnvinfer10=${TENSORRT_DEB_V} \
	libnvonnxparsers10=${TENSORRT_DEB_V} \ 
	python3-libnvinfer=${TENSORRT_DEB_V} \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

# install mmdeploy requirements
RUN pip3 install --no-cache-dir torch==2.8.0 torchvision==0.23.0 --index-url https://download.pytorch.org/whl/cu129 && \
	pip3 install --no-cache-dir openmim==0.3.9 && \
	# install all requirements for mmengine except opencv-python
	pip3 install --no-cache-dir addict termcolor pyyaml matplotlib yapf numpy rich && \
	# install mmengine without opencv-python to use opencv-python built from source
	pip3 install --no-cache-dir --no-deps mmengine==0.10.3 && \
	pip3 install --no-cache-dir pycuda==2023.1

# install all requirements for mmcv except opencv-python, mmengine
RUN pip3 install --no-cache-dir yapf addict pyyaml Pillow numpy packaging termcolor matplotlib rich "platformdirs>=3.5.1" \
	"importlib-metadata>=6.6.0" "tomli>=2.0.1" "zipp>=0.5" "pygments<3.0.0,>=2.13.0" "markdown-it-py>=2.2.0" "mdurl~=0.1"
# install mmcv built in the cache image
COPY --from=cache /usr/local/lib/python3.10/dist-packages/mmcv /usr/local/lib/python3.10/dist-packages/mmcv
COPY --from=cache /usr/local/lib/python3.10/dist-packages/mmcv-2.1.0.dist-info /usr/local/lib/python3.10/dist-packages/mmcv-2.1.0.dist-info

# workaround to to find TensorRT directory when running mmdeploy
# https://github.com/open-mmlab/mmdeploy/issues/2514
ENV TENSORRT_DIR=/opt/tensorrt
RUN mkdir $TENSORRT_DIR && \
	ln -sf /usr/lib/x86_64-linux-gnu $TENSORRT_DIR/lib && \
	ln -sf /usr/include/x86_64-linux-gnu $TENSORRT_DIR/include

# install mmdeploy model converter, mmdeploy SDK
COPY --from=cache /usr/mmdeploy /usr/local
# install all requirements for mmdeploy except opencv-python, mmengine
RUN pip3 install --no-cache-dir terminaltables grpcio numpy matplotlib prettytable six aenum "protobuf<=3.20.2" multiprocess onnx>=1.13.0 \
		addict termcolor pyyaml rich yapf dill>=0.3.7 wcwidth "pygments<3.0.0,>=2.13.0" markdown-it-py>=2.2.0 importlib-metadata>=6.6.0 \
		tomli>=2.0.1 platformdirs>=3.5.1 zipp>=0.5 mdurl~=0.1
# install mmdeploy without opencv-python to use opencv-python built from source
COPY --from=cache /usr/local/lib/python3.10/dist-packages/mmdeploy /usr/local/lib/python3.10/dist-packages/mmdeploy
COPY --from=cache /usr/local/lib/python3.10/dist-packages/onedl_mmdeploy-1.4.0.dist-info /usr/local/lib/python3.10/dist-packages/onedl_mmdeploy-1.4.0.dist-info

# copy mmdeploy_runtime library manually to use mmdeploy_runtime python package
COPY --from=cache /usr/mmdeploy_runtime /opt/mmdeploy_runtime
ENV PYTHONPATH=/opt/mmdeploy_runtime/lib:$PYTHONPATH

# install mmdetection requirements
RUN pip3 install --no-cache-dir pycocotools==2.0.7 shapely==2.0.4 texttable==1.7.0
# install mmdetection
COPY --from=cache /usr/local/lib/python3.10/dist-packages/mmdet /usr/local/lib/python3.10/dist-packages/mmdet
COPY --from=cache /usr/local/lib/python3.10/dist-packages/mmdet-3.3.0.dist-info /usr/local/lib/python3.10/dist-packages/mmdet-3.3.0.dist-info

# clone mmdeploy, mmdetection repository which is used when deploying models
RUN cd /opt && \
	git clone -b cabot-main https://github.com/CMU-cabot/onedl-mmdeploy.git --recursive mmdeploy && \
	cd mmdeploy && \
	cd /opt && \
	git clone -b v3.3.0 https://github.com/open-mmlab/mmdetection.git
