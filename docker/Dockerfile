FROM dustynv/ros:jazzy-desktop-r36.4.0-cu128-24.04

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Only install essentials
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-rviz2 \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/sdrive
COPY src ./src

RUN rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /root/sdrive/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]