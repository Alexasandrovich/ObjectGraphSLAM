FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# 1. Зависимости
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release git build-essential cmake python3-pip python3-tk wget unzip pkg-config \
    libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols \
    libglew-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev \
    libepoxy-dev libpng-dev libjpeg-dev \
    libceres-dev libpcl-dev libtbb-dev libyaml-cpp-dev libboost-filesystem-dev \
    && rm -rf /var/lib/apt/lists/*

RUN ln -s /usr/bin/python3 /usr/bin/python

# 2. ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init && rosdep update

# 3. Pangolin
WORKDIR /root
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && git checkout v0.6 && \
    mkdir build && cd build && cmake .. && make -j$(nproc) && make install && \
    cd ../.. && rm -rf Pangolin

# 4. Внешние библиотеки
RUN mkdir -p /root/external

# amrl_msgs
WORKDIR /root/external
RUN git clone https://github.com/ut-amrl/amrl_msgs.git && \
    cd amrl_msgs && git checkout orbSlamSwitchTraj && \
    . /opt/ros/noetic/setup.sh && \
    export ROS_PACKAGE_PATH=$(pwd):$ROS_PACKAGE_PATH && \
    make

# ORB-SLAM2
WORKDIR /root/external
RUN git clone https://github.com/ut-amrl/ORB_SLAM2.git && \
    cd ORB_SLAM2 && git checkout writeTimestamps && \
    chmod +x build.sh && \
    sed -i '1i#include <unistd.h>' src/System.cc || true && \
    ./build.sh && \
    cd Examples/ROS/ORB_SLAM2 && mkdir build && cd build && \
    . /opt/ros/noetic/setup.sh && \
    export ROS_PACKAGE_PATH=/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:$ROS_PACKAGE_PATH && \
    cmake .. -DROS_BUILD_TYPE=Release && make -j$(nproc)

# 5. ObVi-SLAM (СРАЗУ КАК ut_vslam)
WORKDIR /root/external
RUN git clone https://github.com/ut-amrl/ObVi-SLAM.git ut_vslam

# Подмодуль
WORKDIR /root/external/ut_vslam/src
RUN git clone https://github.com/ut-amrl/amrl_shared_lib.git shared && \
    cd shared && git checkout glogTimer

# Сборка
WORKDIR /root/external/ut_vslam
RUN . /opt/ros/noetic/setup.sh && \
    export ROS_PACKAGE_PATH=/root/external/ut_vslam:/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:$ROS_PACKAGE_PATH && \
    mkdir build && cd build && \
    cmake .. -DROS_BUILD_TYPE=Release && \
    make -j$(nproc)

# 6. Python & YOLO
RUN pip3 install --upgrade pip && \
    pip3 install --no-cache-dir \
    "numpy>=1.21.0,<1.24.0" \
    "pandas>=1.3.0,<2.0.0" \
    "torch==1.13.1" \
    "torchvision==0.14.1" \
    "matplotlib>=3.2.2,<3.8.0" \
    "opencv-python>=4.1.2,<4.9.0.0" \
    "Pillow>=7.1.2" \
    "PyYAML>=5.3.1" \
    "requests>=2.23.0" \
    "scipy>=1.4.1,<1.11.0" \
    "tqdm>=4.64.0" \
    "seaborn>=0.11.0" \
    "rospkg" \
    "empy" \
    "thop"

WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/ut-amrl/yolov5.git && \
    cd yolov5 && git checkout ROS

WORKDIR /root/catkin_ws
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash && \
    export ROS_PACKAGE_PATH=/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:\$ROS_PACKAGE_PATH && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

# ENV
ENV ROS_PACKAGE_PATH=/root/external/ut_vslam:/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:/root/catkin_ws/src:/opt/ros/noetic/share

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
COPY run_pipeline.sh /usr/local/bin/run_pipeline.sh
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/run_pipeline.sh

WORKDIR /root
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]