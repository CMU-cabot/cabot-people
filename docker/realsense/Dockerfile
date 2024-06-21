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

ARG FROM_IMAGE
FROM $FROM_IMAGE AS build
ARG TARGETARCH
ENV DEBIAN_FRONTEND="noninteractive"

RUN if [ "$TARGETARCH" = "amd64" ]; then \
        apt update && \
        apt install -y --no-install-recommends \
        dirmngr \
        gpg-agent \
        software-properties-common \
        curl \
        ; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
        apt update && \
        apt-get install -y --no-install-recommends \
        libogg-dev \
        libtheora-dev \
        software-properties-common \
        ; \
    fi; \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# install realsense
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        mkdir -p /etc/apt/keyrings && \
        curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
        echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
        tee /etc/apt/sources.list.d/librealsense.list && \
        apt-get update && \
        apt install -y --no-install-recommends \
        librealsense2 \
        librealsense2-gl \
    #    librealsense2-net \  # avoid dependency error @ 2023.06.07, may not be used
        librealsense2-udev-rules \
        librealsense2-utils \
        librealsense2-dev \
        librealsense2-dbg \
        # librealsense2-dkms 1.3.28 fails @ 2024.11.21
        librealsense2-dkms=1.3.27-0ubuntu1 \
        ; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
        apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
        apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
        add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" -u \
        && apt update && \
        apt install -y --no-install-recommends \
        librealsense2-utils \
        librealsense2-dev \
        && \
        apt-get clean && \
        rm -rf /var/lib/apt/lists/* \
    ; \
    fi; \
    apt clean && \
    rm -rf /var/lib/apt/lists/*
