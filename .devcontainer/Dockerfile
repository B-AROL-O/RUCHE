FROM ghcr.io/tiryoh/ros2-desktop-vnc:jazzy

ARG USERNAME

USER root

ENV DEBIAN_FRONTEND noninteractive

# Install dependencies
RUN apt-get update && \
    apt-get -y install --no-install-recommends \
    tar curl zip unzip \
    ros-jazzy-ros-gz ros-jazzy-rviz2 \ 
    python3-rosdep nlohmann-json3-dev && \
    apt-get autoremove -y

# User handling
RUN useradd -ms /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    mkdir -p "/etc/sudoers.d" && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD: ALL" | tee "/etc/sudoers.d/$USERNAME"

    COPY ./entrypoint.sh /entrypoint.sh

ENV USER ubuntu
ENV PASSWD ubuntu

ENTRYPOINT ["/bin/bash", "-c", "/entrypoint.sh"]
