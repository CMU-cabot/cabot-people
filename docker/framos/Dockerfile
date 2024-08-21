# Copyright (c) 2023  Carnegie Mellon University, IBM Corporation, and others
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
ARG UBUNTU_DISTRO=jammy
ARG TARGETARCH
ENV DEBIAN_FRONTEND="noninteractive"
ARG FRAMOS_V=2-9-0

RUN apt update && \
    apt install -y --no-install-recommends \
        dirmngr \
        gpg-agent \
        software-properties-common \
        curl \
        wget \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# install Framos realsense
RUN cd /tmp && \
    if [[ ${TARGETARCH} = "amd64" ]]; \
        wget https://www.framos.com/framos3d/D400e/Software/PreviousVersions/FRAMOS_D400e_Software_Package_v${FRAMOS_V}_Linux64_x64.tar.gz && \
        tar xzf FRAMOS_D400e_Software_Package_v${FRAMOS_V}_Linux64_x64.tar.gz ; \
    elif [ "${TARGETARCH}" = "arm64" ]; then \
        wget https://www.framos.com/framos3d/D400e/Software/PreviousVersions/FRAMOS_D400e_Software_Package_v${FRAMOS_V}_Linux64_ARM.tar.gz && \
        tar xzf FRAMOS_D400e_Software_Package_v${FRAMOS_V}_Linux64_ARM.tar.gz ; \
    fi; \
    cd FRAMOS_D400e_Software_Package && \
    apt update && \
    apt install -y --no-install-recommends \
        ./FRAMOS*.deb \
    && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* && \
    cd ../ && \
    rm -rf *
