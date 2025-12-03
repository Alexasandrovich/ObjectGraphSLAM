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

# 3. Настройка рабочего пространства ROS
WORKDIR /root/catkin_ws/src

# --- ВАЖНОЕ ИЗМЕНЕНИЕ: Ручная сборка amrl_msgs ---
# Мы собираем его через make, а потом скрываем от catkin через .CATKIN_IGNORE
RUN git clone https://github.com/ut-amrl/amrl_msgs.git && \
    cd amrl_msgs && \
    git checkout orbSlamSwitchTraj && \
    make && \
    touch .CATKIN_IGNORE

# Клонируем остальные репозитории
RUN git clone https://github.com/ut-amrl/ORB_SLAM2.git && \
    cd ORB_SLAM2 && \
    git checkout writeTimestamps

RUN git clone https://github.com/ut-amrl/ObVi-SLAM.git

# 4. Сборка ORB-SLAM2 (Вручную)
WORKDIR /root/catkin_ws/src/ORB_SLAM2
RUN chmod +x build.sh
# Фикс для usleep
RUN sed -i '1i#include <unistd.h>' src/System.cc || true
RUN ./build.sh

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

# 6. Сборка ROS пакетов (Обычный catkin_make)
WORKDIR /root/catkin_ws
# Мы добавляем amrl_msgs в ROS_PACKAGE_PATH вручную перед сборкой, чтобы ObVi-SLAM его нашел
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    export ROS_PACKAGE_PATH=/root/catkin_ws/src/amrl_msgs:\$ROS_PACKAGE_PATH && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

# 7. Настройка .bashrc
# Прописываем пути навсегда, включая legacy пакет amrl_msgs и ORB_SLAM2 examples
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "export ROS_PACKAGE_PATH=/root/catkin_ws/src/amrl_msgs:/root/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2:\$ROS_PACKAGE_PATH" >> ~/.bashrc

WORKDIR /root
CMD ["bash"]