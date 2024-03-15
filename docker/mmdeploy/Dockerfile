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

# install TensorRT packages
RUN TENSORRT_DEB_V="8.6.1.6-1+cuda11.8" && \
	apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub && \
	apt update && \
	apt-get install -y --no-install-recommends \
	libnvinfer-dev=${TENSORRT_DEB_V} \
	libnvinfer-headers-dev=${TENSORRT_DEB_V} \
	libnvinfer-headers-plugin-dev=${TENSORRT_DEB_V} \
	libnvinfer-plugin-dev=${TENSORRT_DEB_V} \
	libnvinfer-plugin8=${TENSORRT_DEB_V} \
	libnvinfer-vc-plugin8=${TENSORRT_DEB_V} \
	libnvinfer8=${TENSORRT_DEB_V} \
	libnvonnxparsers8=${TENSORRT_DEB_V} \ 
	libnvparsers8=${TENSORRT_DEB_V} \
	python3-libnvinfer=${TENSORRT_DEB_V} \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

# install mmdeploy requirements
RUN pip3 install --no-cache-dir torch==2.1.1 torchvision==0.16.1 --index-url https://download.pytorch.org/whl/cu118 && \
	pip3 install --no-cache-dir openmim==0.3.9 && \
	# install all requirements for mmengine except opencv-python
	pip3 install --no-cache-dir addict termcolor pyyaml matplotlib yapf numpy rich && \
	# install mmengine without opencv-python to use opencv-python built from source
	pip3 install --no-cache-dir --no-deps mmengine==0.10.3 && \
	# install all requirements for mmcv except opencv-python, mmengine
	pip3 install --no-cache-dir yapf addict pyyaml Pillow numpy packaging termcolor matplotlib rich platformdirs>=3.5.1 \
		importlib-metadata>=6.6.0 tomli>=2.0.1 zipp>=0.5 "pygments<3.0.0,>=2.13.0" markdown-it-py>=2.2.0 mdurl~=0.1 && \
	# install mmcv without opencv-python to use opencv-python built from source
	pip3 install --no-cache-dir --no-deps mmcv==2.1.0 -f https://download.openmmlab.com/mmcv/dist/cu118/torch2.0/index.html && \
	pip3 install --no-cache-dir pycuda==2023.1

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
	git clone -b v0.7.0 https://github.com/openppl-public/ppl.cv.git && \
	cd ppl.cv && \
	./build.sh cuda && \
# build mmdeploy model converter
	cd /opt && \
	git clone -b cabot-main https://github.com/CMU-cabot/mmdeploy.git --recursive && \
	cd mmdeploy && \
	mkdir build && cd build && \
	cmake .. \
	-DCMAKE_INSTALL_PREFIX=/usr/mmdeploy \
	-DTENSORRT_DIR=${TENSORRT_DIR} \
	-DMMDEPLOY_TARGET_BACKENDS="trt" && \
	make -j$(nproc) && make install && \
	cd ../ && \
	python3 setup.py bdist_wheel -d /tmp/wheelhouse && \
	rm -rf build && \
# build mmdeploy SDK
	mkdir build && cd build && \
	cmake .. \
	-DCMAKE_INSTALL_PREFIX=/usr/mmdeploy \
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

# build mmdetection
RUN cd /opt && \
	git clone -b v3.3.0 https://github.com/open-mmlab/mmdetection.git && \
	cd mmdetection && \
	python3 setup.py bdist_wheel -d /tmp/wheelhouse && \
	cd ../ && \
	rm -rf mmdetection

# copy mmdeploy, mmdetection from cache image
FROM $FROM_IMAGE AS build

# install TensorRT packages
RUN TENSORRT_DEB_V="8.6.1.6-1+cuda11.8" && \
	apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub && \
	apt update && \
	apt-get install -y --no-install-recommends \
	libnvinfer-dev=${TENSORRT_DEB_V} \
	libnvinfer-headers-dev=${TENSORRT_DEB_V} \
	libnvinfer-headers-plugin-dev=${TENSORRT_DEB_V} \
	libnvinfer-plugin-dev=${TENSORRT_DEB_V} \
	libnvinfer-plugin8=${TENSORRT_DEB_V} \
	libnvinfer-vc-plugin8=${TENSORRT_DEB_V} \
	libnvinfer8=${TENSORRT_DEB_V} \
	libnvonnxparsers8=${TENSORRT_DEB_V} \ 
	libnvparsers8=${TENSORRT_DEB_V} \
	python3-libnvinfer=${TENSORRT_DEB_V} \
	&& \
	apt clean && \
	rm -rf /var/lib/apt/lists/*

# install mmdeploy requirements
RUN pip3 install --no-cache-dir torch==2.1.1 torchvision==0.16.1 --index-url https://download.pytorch.org/whl/cu118 && \
	pip3 install --no-cache-dir openmim==0.3.9 && \
	# install all requirements for mmengine except opencv-python
	pip3 install --no-cache-dir addict termcolor pyyaml matplotlib yapf numpy rich && \
	# install mmengine without opencv-python to use opencv-python built from source
	pip3 install --no-cache-dir --no-deps mmengine==0.10.3 && \
	# install all requirements for mmcv except opencv-python, mmengine
	pip3 install --no-cache-dir yapf addict pyyaml Pillow numpy packaging termcolor matplotlib rich platformdirs>=3.5.1 \
		importlib-metadata>=6.6.0 tomli>=2.0.1 zipp>=0.5 "pygments<3.0.0,>=2.13.0" markdown-it-py>=2.2.0 mdurl~=0.1 && \
	# install mmcv without opencv-python to use opencv-python built from source
	pip3 install --no-cache-dir --no-deps mmcv==2.1.0 -f https://download.openmmlab.com/mmcv/dist/cu118/torch2.1/index.html && \
	pip3 install --no-cache-dir pycuda==2023.1

# workaround to to find TensorRT directory when running mmdeploy
# https://github.com/open-mmlab/mmdeploy/issues/2514
ENV TENSORRT_DIR=/opt/tensorrt
RUN mkdir $TENSORRT_DIR && \
	ln -sf /usr/lib/x86_64-linux-gnu $TENSORRT_DIR/lib && \
	ln -sf /usr/include/x86_64-linux-gnu $TENSORRT_DIR/include

# install mmdeploy, mmdetection packages
COPY --from=cache /usr/mmdeploy /usr/local
COPY --from=cache /tmp/wheelhouse /tmp/wheelhouse
# install all requirements for mmdeploy except opencv-python, mmengine
RUN pip3 install --no-cache-dir terminaltables grpcio numpy matplotlib prettytable six aenum "protobuf<=3.20.2" multiprocess onnx>=1.13.0 \
		addict termcolor pyyaml rich yapf dill>=0.3.7 wcwidth "pygments<3.0.0,>=2.13.0" markdown-it-py>=2.2.0 importlib-metadata>=6.6.0 \
		tomli>=2.0.1 platformdirs>=3.5.1 zipp>=0.5 mdurl~=0.1 && \
	# install mmdeploy without opencv-python to use opencv-python built from source
	pip3 install --no-cache-dir --no-deps /tmp/wheelhouse/mmdeploy-*.whl && \
	pip3 install --no-cache-dir /tmp/wheelhouse/mmdet-*.whl && \
	rm -rf /tmp/wheelhouse

# copy mmdeploy_runtime library manually to use mmdeploy_runtime python package
COPY --from=cache /usr/mmdeploy_runtime /opt/mmdeploy_runtime
ENV PYTHONPATH=/opt/mmdeploy_runtime/lib:$PYTHONPATH

# clone mmdeploy, mmdetection repository which is used when deploying models
RUN cd /opt && \
	git clone -b cabot-main https://github.com/CMU-cabot/mmdeploy.git --recursive && \
	cd mmdeploy && \
	cd /opt && \
	git clone -b v3.3.0 https://github.com/open-mmlab/mmdetection.git