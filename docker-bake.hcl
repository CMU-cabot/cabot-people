variable "PLATFORMS" {
  default = ["linux/arm64", "linux/amd64"]
}

variable "CAMERAS" {
  default = ["realsense", "framos"]
}

variable "ROS_DISTRO" {
  default = "humble"
}

variable "UBUNTU_DISTRO" {
  default = "jammy"
}

variable "BASE_IMAGE" {
  default = "cabot-gpu-base"
}

variable "REGISTRY" {
  default = "registry"
}

group "default" {
  targets = [
    "base",
    "camera-base",
    "ros-core-amd64",
    "ros-base-amd64",
    "ros-desktop-amd64",
    "ros-desktop-custom-amd64",
    "ros-desktop-custom-opencv-amd64",
    "ros-desktop-custom-opencv-open3d-amd64",
    "ros-desktop-custom-opencv-open3d-mesa-amd64",
  ]
}

#### COMMON

target "base" {
  context    = "./docker/base"
  dockerfile-ineline = <<EOF
FROM --platform=linux/amd64 nvidia/cuda:11.7.1-cudnn8-devel-ubuntu22.04 as build-amd64
FROM --platform=linux/arm64 nvcr.io/nvidia/l4t-base:r36.2.0 as build-arm64
# FROM --platform=linux/arm64 nvcr.io/nvidia/l4t-base:r35.1.0 as build-arm64
FROM build-$TARGETARCH
EOF
  platforms  = "${PLATFORMS}"
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:base" ]
  output     = [ "type=registry" ]
}

target "camera-base" {
  matrix     = {
    camera = ["realsense", "framos"]
  }
  name       = "${camera}-base"
  platforms  = "${PLATFORMS}"
  context    = "./docker/${camera}"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}" = "target:base" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}" ]
  output     = [ "type=registry" ]
}

#### AMD64

target "ros-common-amd64" {
  platforms  = [ "linux/amd64" ]
  output     = [ "type=registry" ]
}

target "ros-core-amd64" {
  inherits   = [ "ros-common-amd64" ]
  matrix     = {
    camera = ["realsense", "framos"]
  }
  name       = "${camera}-ros-core-amd64"
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-core"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-amd64" = "target:${camera}-base" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-amd64" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-amd64" ]
}

target "ros-base-amd64" {
  inherits   = [ "ros-common-amd64" ]
  matrix     = {
    camera = ["realsense", "framos"]
  }
  name       = "${camera}-ros-base-amd64"
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-base"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-amd64" = "target:${camera}-ros-core-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-amd64" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-base-amd64" ]
}

target "ros-desktop-amd64" {
  inherits   = [ "ros-common-amd64" ]
  matrix     = {
    camera = ["realsense", "framos"]
  }
  name       = "${camera}-ros-desktop-amd64"
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/desktop"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-base-amd64" = "target:${camera}-ros-base-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-base-amd64" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-amd64" ]
}

target "ros-desktop-custom-amd64" {
  inherits   = [ "ros-common-amd64" ]
  matrix     = {
    camera = ["realsense", "framos"]
  }
  name       = "${camera}-ros-desktop-custom-amd64"
  context    = "./cabot-common/docker/humble-custom"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-amd64" = "target:${camera}-ros-desktop-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-amd64" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom-amd64" ]
}

target "ros-desktop-custom-opencv-amd64" {
  inherits   = [ "ros-common-amd64" ]
  matrix     = {
    camera = ["realsense", "framos"]
  }
  name       = "${camera}-ros-desktop-custom-opencv-amd64"
  context    = "./docker/opencv"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom-amd64" = "target:${camera}-ros-desktop-custom-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom-amd64" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom-opencv-amd64" ]
}

target "ros-desktop-custom-opencv-open3d-amd64" {
  inherits   = [ "ros-common-amd64" ]
  matrix     = {
    camera = ["realsense", "framos"]
  }
  name       = "${camera}-ros-desktop-custom-opencv-open3d-amd64"
  context    = "./docker/open3d"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom-opencv-amd64" = "target:${camera}-ros-desktop-custom-opencv-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom-opencv-amd64" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom-opencv-open3d-amd64" ]
}

target "ros-desktop-custom-opencv-open3d-mesa-amd64" {
  inherits   = [ "ros-common-amd64" ]
  matrix     = {
    camera = ["realsense", "framos"]
  }
  name       = "${camera}-ros-desktop-custom-opencv-open3d-mesa-amd64"
  context    = "./cabot-common/docker/mesa"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom-opencv-open3d-amd64" = "target:${camera}-ros-desktop-custom-opencv-open3d-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom-opencv-open3d-amd64" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom-opencv-open3d-mesa-amd64" ]
}

/*
#### ARM64

target "ros-arm64-common" {
  platforms  = [ "linux/arm64" ]
  output     = [ "type=registry" ]
}
*/