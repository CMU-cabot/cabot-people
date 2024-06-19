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
    "ros-core",
    "ros-base",
    "ros-desktop",
    "ros-desktop-custom",
    "ros-desktop-custom-opencv",
    "ros-desktop-custom-opencv-open3d",
    "ros-desktop-custom-opencv-open3d-mesa",
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
}

#### AMD64

target "ros-common-amd64" {
  matrix     = {
    camera = ["realsense", "framos"]
  }
  platforms  = [ "linux/amd64" ]
  output     = [ "type=registry" ]
}

target "ros-core-amd64" {
  inherits   = [ "ros-common-amd64" ]
  name       = "${camera}-ros-core-amd64"
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-core"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}" = "target:${camera}-base" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}" ]
}

target "ros-base-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-base"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}" = "target:${camera}-ros-core-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-base" ]
}

target "ros-desktop-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/desktop"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-base" = "target:${camera}-ros-base-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-base" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop" ]
}

target "ros-desktop-custom-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/desktop"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop" = "target:${camera}-ros-desktop-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom" ]
}

target "ros-desktop-custom-opencv-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/opencv"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop" = "target:${camera}-ros-desktop-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom" ]
}

target "ros-desktop-custom-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/desktop"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop" = "target:${camera}-ros-desktop-amd64" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-${ROS_DISTRO}-desktop-custom" ]
}

target "ros-desktop-custom-opencv" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/opencv"
  dockerfile = "Dockerfile"
  platforms  = "${PLATFORMS}"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom" = "target:ros-desktop-custom" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv" ]
}

target "ros-desktop-custom-opencv-open3d" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/open3d"
  dockerfile = "Dockerfile"
  platforms  = "${PLATFORMS}"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv" = "target:ros-desktop-custom-opencv" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-open3d" ]
}

target "ros-desktop-custom-opencv-open3d-mesa" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/mesa"
  dockerfile = "Dockerfile"
  platforms  = "${PLATFORMS}"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-open3d" = "target:ros-desktop-custom-opencv-open3d" }
  args       = { FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-open3d" }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-open3d-mesa" ]
}

#### ARM64

target "ros-arm64-common" {
  platforms  = [ "linux/arm64" ]
  output     = [ "type=registry" ]
}
