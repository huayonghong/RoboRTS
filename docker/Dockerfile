FROM ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    libprotobuf-dev protobuf-compiler \
    libopencv-dev \
    libeigen3-dev \
    libboost-all-dev \
    libpcl-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    ros-humble-nav2-map-server \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-laser-geometry \
    ros-humble-pcl-conversions \
    ros-humble-message-filters \
    ros-humble-interactive-markers \
    ros-humble-rclcpp-action \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src
COPY . /ros2_ws/src/RoboRTS

WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
