#!/bin/bash
set -e

# 1. Подгружаем стандартный ROS
source /opt/ros/noetic/setup.bash

# 2. Подгружаем наш workspace
if [ -f "/root/catkin_ws/devel/setup.bash" ]; then
    source /root/catkin_ws/devel/setup.bash
fi

# 3. Добавляем внешние библиотеки (amrl_msgs, ORB_SLAM2) в путь ROS
export ROS_PACKAGE_PATH=/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:$ROS_PACKAGE_PATH

# 4. Выполняем команду, переданную в docker run (или bash по умолчанию)
exec "$@"