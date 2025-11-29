# File: RUCHE/.devcontainer/Dockerfile

# FROM ghcr.io/tiryoh/ros2-desktop-vnc:jazzy
FROM ghcr.io/tiryoh/ros2-desktop-vnc:jazzy-20251124T0218

LABEL org.opencontainers.image.description="RUCHE Dev Container (based on tiryoh/ros2-desktop-vnc)"

ARG USERNAME="vscode"
# USER root
ENV DEBIAN_FRONTEND="noninteractive"

# Install dependencies
# hadolint ignore=DL3009
RUN apt-get update && \
    apt-get -y install --no-install-recommends \
        tar curl zip unzip \
        python3-rosdep \
        nlohmann-json3-dev
# hadolint ignore=DL3008
RUN apt-get -y install --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-rviz2 \
    && apt-get -y autoremove \
    && rm -rf /var/lib/apt/lists/*

# User handling
# hadolint ignore=DL4006
RUN useradd -ms /bin/bash "$USERNAME" && \
    usermod -aG sudo "$USERNAME" && \
    mkdir -p "/etc/sudoers.d" && \
    echo "$USERNAME ALL=(ALL:ALL) NOPASSWD: ALL" | tee "/etc/sudoers.d/$USERNAME"

COPY ./entrypoint.sh /entrypoint.sh

# ENV USER ubuntu
# ENV PASSWD ubuntu

USER ${USERNAME}

ENTRYPOINT ["/bin/bash", "-c", "/entrypoint.sh"]

# EOF
