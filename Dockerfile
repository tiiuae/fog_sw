# fog-sw BUILDER
FROM ros:foxy-ros-base as fog-sw-builder

ARG UID=1000
ARG GID=1000

# workaround for ROS GPG Key Expiration Incident
RUN rm /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    apt-get install -y curl && \
    curl http://repo.ros2.org/repos.key | sudo apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update

RUN groupadd -g $GID builder && \
    useradd -m -u $UID -g $GID -g builder builder && \
    usermod -aG sudo builder && \
    echo 'builder ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

WORKDIR /env

COPY ./update_dependencies.sh ./rosdep.yaml /env/

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections && \
    export DEBIAN_FRONTEND=noninteractive && \
    ./update_dependencies.sh

USER builder

VOLUME /fog_sw
WORKDIR /fog_sw

RUN rosdep update
