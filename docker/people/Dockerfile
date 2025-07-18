# Copyright (c) 2021, 2022  IBM Corporation and Carnegie Mellon University
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

ARG GPU_BASE_REGISTRY=cmucal
ARG BASE_IMAGE=cabot-gpu-base
ARG FROM_IMAGE=${GPU_BASE_REGISTRY}/${BASE_IMAGE}:realsense-final

FROM $FROM_IMAGE AS build

ARG TARGETARCH
ARG ROS_DISTRO=humble \
    UBUNTU_DISTRO=jammy \
    GAZEBOV=11 \
    TZ="Etc/UTC"

ENV ROS_DISTRO=$ROS_DISTRO \
    TZ=$TZ

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN if [ "${TARGETARCH}" = "amd64" ]; then \
        apt update && \
        apt install -y --no-install-recommends \
        git \
        bc \
        gosu \
        jq \
        libfreetype6-dev \
        libglu1-mesa-dev \
        python-tk \
        python3-pip \
        python3-vcstool \
        rsync \
        software-properties-common \
        unzip \
        usbutils \
        wget \
        xorg-dev \
        && \
        apt clean && \
        rm -rf /var/lib/apt/lists/* ; \
    elif [ "${TARGETARCH}" = "arm64" ]; then \
        apt update && \
        apt install -y --no-install-recommends \
        bc \
        gosu \
        jq \
        && \
        apt clean && \
        rm -rf /var/lib/apt/lists/* ; \
    fi

# install ros packages
RUN if [ "${TARGETARCH}" = "amd64" ]; then \
        apt update && \
        apt install -y --no-install-recommends \
        libogg-dev \
        libpcl-dev \
        libtheora-dev \
        libturbojpeg0-dev \
        python3-pcl \
        ros-$ROS_DISTRO-diagnostic-updater \
        ros-$ROS_DISTRO-compressed-image-transport \
        ros-$ROS_DISTRO-gazebo-msgs \
        ros-$ROS_DISTRO-pcl-msgs \
        # ros-$ROS_DISTRO-people-msgs \
        ros-$ROS_DISTRO-rviz2 \
        # ros-$ROS_DISTRO-rviz \
        ros-$ROS_DISTRO-tf2 \
        ros-$ROS_DISTRO-tf2-geometry-msgs \
        ros-$ROS_DISTRO-tf-transformations \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
        ros-dev-tools \
        qtbase5-dev \
        libarmadillo-dev \
        && \
        apt clean && \
        rm -rf /var/lib/apt/lists/* ; \
    elif [ "${TARGETARCH}" = "arm64" ]; then \
        apt update && \
        apt install -y --no-install-recommends \
        libpcl-dev \
        python3-pcl \
        && \
        apt clean && \
        rm -rf /var/lib/apt/lists/* ; \
    fi

COPY requirements.txt /
RUN pip3 install --no-cache-dir \
        -r /requirements.txt

# install gazebo
#RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $UBUNTU_DISTRO main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
#	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
#	apt update && \
#	apt install -y --no-install-recommends \
#	gazebo$GAZEBOV \
#	libgazebo$GAZEBOV-dev \
#	&& \
#	apt clean && \
#	rm -rf /var/lib/apt/lists/*

COPY ./launch.sh \
     ./launch-core.sh \
     ./resetusb.sh ./resetrs.sh \
     ./setup-model.sh /
 
# setup for display
ENV USERNAME developer

### replace 1000 with your user/group id
ARG UID=1000
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d/ && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid $UID $USERNAME && \
        groupmod --gid $UID $USERNAME
# on jetson, add user to video group to use CUDA
RUN if [ "${TARGETARCH}" = "arm64" ]; then \
        usermod -aG video $USERNAME ; \
    fi

ENV HOME /home/$USERNAME

WORKDIR $HOME/people_ws
RUN chown $USERNAME:$USERNAME $HOME/people_ws

FROM build as cache

USER $USERNAME

COPY --from=src ./cabot-common/cabot_common src/cabot_common
COPY --from=src ./cabot-common/cabot_msgs src/cabot_msgs
COPY --from=src ./cabot_people src/cabot_people
COPY --from=src ./track_people_py src/track_people_py
COPY --from=src ./track_people_msgs src/track_people_msgs
COPY --from=src ./track_people_cpp src/track_people_cpp

RUN /ros_entrypoint.sh /launch.sh build -s

FROM build as final

# change location of ros base
COPY ros_entrypoint.sh /ros_entrypoint.sh

COPY --chown=$USERNAME:$USERNAME --from=cache $HOME/people_ws/install $HOME/people_ws/install
COPY --chown=$USERNAME:$USERNAME --from=cache $HOME/people_ws/src/cabot_people/script $HOME/people_ws/src/cabot_people/script

