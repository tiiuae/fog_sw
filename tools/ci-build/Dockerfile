# FOG SW BUILDER
# There should not be need to publish this builder image anywhere
FROM ubuntu:20.04 as fogsw_builder

ARG user_id

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update && apt-get install -q -y tzdata \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=C.UTF-8 \
    LANGUAGE=C.UTF-8 \
    LC_ALL=C.UTF-8

# Install ROS 2
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    build-essential \
    dh-make debhelper \
    fakeroot \
    git-core \
    golang \
    libasio-dev \
    openjdk-11-jdk-headless \
    openssh-client \
    python3-bloom \
    python3-colcon-common-extensions \
    python3-pip \
    python3-future \
    python3-genmsg \
    ros-foxy-ros-base \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    nlohmann-json3-dev \
    zlib1g-dev \
    libusb-1.0-0-dev \
    freeglut3-dev \
    liblapacke-dev \
    libopenblas-dev \
    libatlas-base-dev \
    dh-python \
    batctl \
    alfred \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://github.com/mavlink/MAVSDK/releases/download/v0.34.0/mavsdk_0.34.0_ubuntu20.04_amd64.deb -o mavsdk_0.34.0_ubuntu20.04_amd64.deb \
    && dpkg -i mavsdk_0.34.0_ubuntu20.04_amd64.deb \
    && rm mavsdk_0.34.0_ubuntu20.04_amd64.deb

RUN mkdir -p /fastdds
WORKDIR /fastdds

RUN git clone --depth 1 --branch v1.0.4 https://github.com/eProsima/Fast-DDS-Gen.git Fast-DDS-Gen \
    && cd Fast-DDS-Gen \
    && git submodule update --init --recursive

# Build fastddsgen
RUN cd Fast-DDS-Gen \
    && ./gradlew build -x test \
    && mkdir -p /usr/bin/ \
    && mkdir -p /usr/share/fastddsgen/java/ \
    && cp build/libs/fastddsgen.jar /usr/share/fastddsgen/java/fastddsgen.jar \
    && cp scripts/fastddsgen /usr/bin/fastddsgen

COPY fog_sw   /fog_sw
WORKDIR /fog_sw

RUN ./build_setup.sh \
    && . /opt/ros/foxy/setup.sh \
    && cd packaging \
    && ./package.sh

FROM scratch
COPY --from=fogsw_builder /fog_sw/packaging/*.deb /packages/
