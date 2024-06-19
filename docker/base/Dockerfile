FROM --platform=linux/amd64 nvidia/cuda:11.7.1-cudnn8-devel-ubuntu22.04 as build-amd64
FROM --platform=linux/arm64 nvcr.io/nvidia/l4t-base:r36.2.0 as build-arm64

FROM build-$TARGETARCH
