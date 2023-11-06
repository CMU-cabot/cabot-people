FROM ros:humble-ros-base-jammy

RUN apt update && apt install -y \
	ros-humble-diagnostic-updater \
	ros-humble-cv-bridge

RUN apt update && apt install -y \
	wget

ARG OPEN3D_VERSION=0.15.1

# install open3d for c++
RUN cd /tmp && \
	wget https://github.com/isl-org/Open3D/releases/download/v${OPEN3D_VERSION}/open3d-devel-linux-x86_64-cxx11-abi-cuda-${OPEN3D_VERSION}.tar.xz && \
        tar xf open3d-devel-linux-x86_64-cxx11-abi-cuda-${OPEN3D_VERSION}.tar.xz && \
	cp -r open3d-devel-linux-x86_64-cxx11-abi-cuda-${OPEN3D_VERSION}/* /usr/local && \
	rm -rf *

ENV USERNAME developer
# Replace 1000 with your user/group id
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

USER $USERNAME

ENV HOME /home/$USERNAME

WORKDIR $HOME/ros2_ws

RUN git clone https://github.com/wg-perception/people.git -b ros2 src/people

