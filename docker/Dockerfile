# Используем образ с уже установленным ROS Noetic и поддержкой GPU/CUDA
FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu20.04

# Отключаем интерактивные вопросы
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# 1. Установка базовых утилит, ROS и ключей
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-tk \
    wget \
    unzip \
    pkg-config \
    libegl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    && rm -rf /var/lib/apt/lists/*

# Добавляем ключи ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -

# Устанавливаем ROS Desktop Full
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

# 2. Установка зависимостей (Ceres, Eigen, GLEW + Графика)
RUN apt-get update && apt-get install -y \
    libglew-dev \
    libgoogle-glog-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libepoxy-dev \
    libpng-dev \
    libjpeg-dev \
    && rm -rf /var/lib/apt/lists/*

# --- Сборка Pangolin (ВЕРСИЯ v0.6) ---
WORKDIR /root
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git checkout v0.6 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && rm -rf Pangolin

# Создаем ссылку python -> python3 (нужно для старых скриптов ROS)
RUN ln -s /usr/bin/python3 /usr/bin/python

# Создаем папку для внешних библиотек (НЕ catkin_ws)
RUN mkdir -p /root/external

# --- Сборка amrl_msgs (ВНЕШНЯЯ) ---
WORKDIR /root/external
# Клонируем и собираем здесь, чтобы catkin_make не видел исходники
RUN git clone https://github.com/ut-amrl/amrl_msgs.git && \
    cd amrl_msgs && \
    git checkout orbSlamSwitchTraj && \
    . /opt/ros/noetic/setup.sh && \
    export ROS_PACKAGE_PATH=$(pwd):$ROS_PACKAGE_PATH && \
    make

# --- Сборка ORB-SLAM2 (ВНЕШНЯЯ) ---
WORKDIR /root/external
RUN git clone https://github.com/ut-amrl/ORB_SLAM2.git && \
    cd ORB_SLAM2 && \
    git checkout writeTimestamps && \
    chmod +x build.sh && \
    sed -i '1i#include <unistd.h>' src/System.cc || true && \
    ./build.sh

# 5. Установка Python-зависимостей для YOLO
RUN pip3 install --upgrade pip
RUN pip3 install \
    torch torchvision torchaudio \
    matplotlib \
    numpy \
    opencv-python \
    Pillow \
    PyYAML \
    requests \
    scipy \
    tqdm \
    pandas \
    seaborn \
    rospkg \
    empy

# 6. Сборка основного проекта (ObVi-SLAM)
# Только теперь мы идем в catkin_ws
WORKDIR /root/catkin_ws/src

# Клонируем ObVi-SLAM (это единственный пакет, который увидит catkin_make)
RUN git clone https://github.com/ut-amrl/ObVi-SLAM.git

# YOLOv5
RUN git clone https://github.com/ut-amrl/yolov5.git && \
    cd yolov5 && \
    git checkout ROS

# Сборка ROS пакетов
WORKDIR /root/catkin_ws
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash && \
    export ROS_PACKAGE_PATH=/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:\$ROS_PACKAGE_PATH && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

ENV ROS_PACKAGE_PATH=/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:/root/catkin_ws/src:/opt/ros/noetic/share

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
COPY run_pipeline.sh /usr/local/bin/run_pipeline.sh

RUN chmod +x /usr/local/bin/entrypoint.sh && \
    chmod +x /usr/local/bin/run_pipeline.sh

WORKDIR /root
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/usr/local/bin/run_pipeline.sh"]