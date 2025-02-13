variable "PLATFORMS" {
  default = ["linux/arm64", "linux/amd64"]
}

# can be overriden by env variable
variable "CAMERAS" {
  default = "realsense framos"
}

variable "CAMERAS_" {
  default = split(" ", "${CAMERAS}")
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

variable "L4T_V" {
  default = "35.3.1"
#  default = "36.2.0"
}

variable "L4T_IMAGE" {
  default = "nvcr.io/nvidia/l4t-base:${L4T_V}"
#  default = "nvcr.io/nvidia/l4t-base:${L4T_V}"
}

variable "L4T_MAJOR_MINOR_V" {
  default = join(".", slice(split(".", "${L4T_V}"), 0, 2))
}

variable "TEGRA_V" {
  default = "t234"
}

variable "OPENCV_V" {
  default = "4.5.4"
}

variable "L4T_CUDA" {
  default = "11-4"
}

group "default" {
  targets = [
    "targets-amd64",
    "targets-arm64",
  ]
}

group "targets-amd64" {
  targets = [
    "base",
    "ros-core-amd64",
    "ros-base-amd64",
    "ros-desktop-amd64",
    "ros-desktop-custom-amd64",
    "ros-desktop-custom-opencv-amd64",
    "ros-desktop-custom-opencv-mmdeploy-amd64",
    "ros-desktop-custom-opencv-mmdeploy-open3d-amd64",
    "ros-desktop-custom-opencv-mmdeploy-open3d-mesa-amd64",
    "ros-desktop-custom-opencv-mmdeploy-open3d-mesa-camera-amd64",
    "final-amd64",
  ]
}

group "targets-arm64" {
  targets = [
    "base",
    "opencv-arm64",
    "opencv-ros-base-arm64",
    "opencv-ros-custom-arm64",
    "opencv-ros-custom-mmdeploy-arm64",
    "opencv-ros-custom-mmdeploy-open3d-arm64",
    "opencv-ros-custom-mmdeploy-open3d-camera-arm64",
    "final-arm64",
  ]
}

#### COMMON

target "base" {
  context    = "."
  dockerfile-inline = <<EOF
FROM --platform=linux/amd64 nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04 as build-amd64
FROM --platform=linux/arm64 ${L4T_IMAGE} as build-arm64
FROM build-$TARGETARCH
EOF
  platforms  = "${PLATFORMS}"
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:base" ]
  output     = [ "type=cacheonly" ]
}

#### AMD64

target "ros-common-amd64" {
  platforms  = [ "linux/amd64" ]
  output     = [ "type=cacheonly" ]
}

target "ros-core-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./cabot-base/docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-core"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:base" = "target:base" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:base"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-amd64" ]
}

target "ros-base-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./cabot-base/docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/ros-base"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-amd64" = "target:ros-core-amd64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-amd64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base-amd64" ]
}

target "ros-desktop-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./cabot-base/docker/docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/desktop"
  dockerfile = "Dockerfile.tmp"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base-amd64" = "target:ros-base-amd64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-base-amd64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-amd64" ]
}

target "ros-desktop-custom-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./cabot-base/docker/humble-custom"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-amd64" = "target:ros-desktop-amd64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-amd64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-amd64" ]
}

target "ros-desktop-custom-opencv-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/opencv"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-amd64" = "target:ros-desktop-custom-amd64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-amd64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-amd64" ]
}

target "ros-desktop-custom-opencv-mmdeploy-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/mmdeploy"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-amd64" = "target:ros-desktop-custom-opencv-amd64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-amd64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-amd64" ]
}

target "ros-desktop-custom-opencv-mmdeploy-open3d-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./docker/open3d"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-amd64" = "target:ros-desktop-custom-opencv-mmdeploy-amd64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-amd64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-open3d-amd64" ]
}

target "ros-desktop-custom-opencv-mmdeploy-open3d-mesa-amd64" {
  inherits   = [ "ros-common-amd64" ]
  context    = "./cabot-base/docker/mesa"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-open3d-amd64" = "target:ros-desktop-custom-opencv-mmdeploy-open3d-amd64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-open3d-amd64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-open3d-mesa-amd64" ]
}

target "ros-desktop-custom-opencv-mmdeploy-open3d-mesa-camera-amd64" {
  inherits   = [ "ros-common-amd64" ]
  matrix     = {
    camera = "${CAMERAS_}"
  }
  name       = "ros-desktop-custom-opencv-mmdeploy-open3d-mesa-${camera}-amd64"
  context    = "./docker/${camera}"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-open3d-mesa-amd64" = "target:ros-desktop-custom-opencv-mmdeploy-open3d-mesa-amd64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-open3d-mesa-amd64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-open3d-mesa-${camera}-amd64" ]
}


#### ARM64

target "ros-common-arm64" {
  platforms  = [ "linux/arm64" ]
  output     = [ "type=cacheonly" ]
}

target "opencv-arm64" {
  inherits   = [ "ros-common-arm64" ]
  context    = "./docker/opencv"
  dockerfile = "Dockerfile.jetson"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:base" = "target:base" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:base",
    OPENCV_V   = "${OPENCV_V}",
    CUDA_V     = "${L4T_CUDA}",
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:opencv-arm64" ]
}

target "opencv-ros-base-arm64" {
  inherits   = [ "ros-common-arm64" ]
  context    = "./cabot-base/docker/jetson-humble-base-src"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:opencv-arm64" = "target:opencv-arm64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:opencv-arm64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-base-arm64" ]
}

target "opencv-ros-custom-arm64" {
  inherits   = [ "ros-common-arm64" ]
  context    = "./docker/jetson-humble-custom"
  dockerfile = "Dockerfile"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-base-arm64" = "target:opencv-ros-base-arm64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-base-arm64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-arm64" ]
}

target "opencv-ros-custom-mmdeploy-arm64" {
  inherits   = [ "ros-common-arm64" ]
  context    = "./docker/mmdeploy"
  dockerfile = "Dockerfile.jetson"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-arm64" = "target:opencv-ros-custom-arm64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-arm64",
    CUDA_V     = "${L4T_CUDA}",
    L4T_V      = "${L4T_MAJOR_MINOR_V}",
    TEGRA_V    = "${TEGRA_V}",
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-mmdeploy-arm64" ]
}

target "opencv-ros-custom-mmdeploy-open3d-arm64" {
  inherits   = [ "ros-common-arm64" ]
  context    = "./docker/open3d"
  dockerfile = "Dockerfile.jetson"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-mmdeploy-arm64" = "target:opencv-ros-custom-mmdeploy-arm64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-mmdeploy-arm64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-mmdeploy-open3d-arm64" ]
}

target "opencv-ros-custom-mmdeploy-open3d-camera-arm64" {
  inherits   = [ "ros-common-arm64" ]
  matrix     = {
    camera = "${CAMERAS_}"
  }
  name       = "opencv-ros-custom-mmdeploy-open3d-${camera}-arm64"
  context    = "./docker/${camera}"
  dockerfile = "Dockerfile.jetson"
  contexts   = { "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-mmdeploy-open3d-arm64" = "target:opencv-ros-custom-mmdeploy-open3d-arm64" }
  args       = {
    FROM_IMAGE = "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-mmdeploy-open3d-arm64"
  }
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-mmdeploy-open3d-${camera}-arm64" ]
}


### integration

target "final-amd64" {
  matrix     = {
    camera = "${CAMERAS_}"
  }
  name       = "${camera}-final-amd64"
  context    = "."
  contexts   = {
    "${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-open3d-mesa-${camera}-amd64" = "target:ros-desktop-custom-opencv-mmdeploy-open3d-mesa-${camera}-amd64",
  }
  dockerfile-inline = <<EOF
FROM --platform=linux/amd64 ${REGISTRY}/${BASE_IMAGE}:${ROS_DISTRO}-desktop-custom-opencv-mmdeploy-open3d-mesa-${camera}-amd64 as build-amd64
FROM build-$TARGETARCH
EOF
  platforms  = [ "linux/amd64" ]
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-final-amd64" ]
  output     = [ "type=registry" ]
}

target "final-arm64" {
  matrix     = {
    camera = "${CAMERAS_}"
  }
  name       = "${camera}-final-arm64"
  context    = "."
  contexts   = {
    "${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-mmdeploy-open3d-${camera}-arm64" = "target:opencv-ros-custom-mmdeploy-open3d-${camera}-arm64",
  }
  dockerfile-inline = <<EOF
FROM --platform=linux/arm64 ${REGISTRY}/${BASE_IMAGE}:opencv-${ROS_DISTRO}-custom-mmdeploy-open3d-${camera}-arm64 as build-arm64
FROM build-$TARGETARCH
EOF
  platforms  = [ "linux/arm64" ]
  tags       = [ "${REGISTRY}/${BASE_IMAGE}:${camera}-final-arm64" ]
  output     = [ "type=registry" ]
}
