#!/bin/bash

# Функция очистки
cleanup() {
    echo "[INFO] Stopping background processes..."
    kill $(jobs -p) 2>/dev/null
    exit
}
trap cleanup SIGINT SIGTERM

echo "========================================"
echo "[INFO] Configuring Environment..."
echo "========================================"

source /opt/ros/noetic/setup.bash

# 1. CRITICAL FIX:
# Мы должны явно добавить путь к ObVi-SLAM, иначе rosbuild сойдет с ума при сборке.
# Порядок важен: сначала сам проект, потом зависимости.
export ROS_PACKAGE_PATH=/root/external/ObVi-SLAM:/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:$ROS_PACKAGE_PATH

if [ -f "/root/catkin_ws/devel/setup.bash" ]; then
    source /root/catkin_ws/devel/setup.bash
    # Добавляем workspace
    export ROS_PACKAGE_PATH=/root/catkin_ws/src:$ROS_PACKAGE_PATH
fi

# Обновляем индекс пакетов, чтобы ROS увидел новые пути
rospack profile > /dev/null

# ======================================================
# [SELF-HEALING] Проверка и сборка
# ======================================================
echo "[CHECK] Verifying ObVi-SLAM packages..."

# Проверяем наличие frontend пакета
if ! rospack find obvislam_frontend > /dev/null 2>&1; then
    echo "[ERROR] Package 'obvislam_frontend' NOT found (or not compiled)!"
else
    echo "[INFO] Package 'obvislam_frontend' found."
fi
# ======================================================

echo "------------------------------------------------"
echo "[STEP 1/4] Starting ROS Core..."
roscore > /root/roscore.log 2>&1 &
ROSCORE_PID=$!
sleep 5

echo "[STEP 2/4] Starting YOLOv5 Object Detector..."
cd /root/catkin_ws/src/yolov5
python3 -u detect_ros.py --weights yolov5m.pt --img 640 --conf 0.5 2>&1 | sed "s/^/[YOLO] /" &
YOLO_PID=$!

echo "[INFO] Waiting 15s for YOLO model to load..."
sleep 15

echo "------------------------------------------------"
echo "[STEP 3/4] Running ObVi-SLAM Frontend..."
cd /root
rosrun obvislam_frontend run_orb_slam --config_file /root/data/run_kitti.json

echo "------------------------------------------------"
echo "[STEP 4/4] Running ObVi-SLAM Backend..."
rosrun obvislam_backend run_backend --config_file /root/data/run_kitti.json

echo "------------------------------------------------"
echo "[SUCCESS] Pipeline Finished! Results are in /root/data/ut_vslam_results"
echo "------------------------------------------------"
echo "[INFO] To visualize: open new terminal and run: rviz"
echo "[INFO] Press Ctrl+C to stop container."

# Держим контейнер активным
wait $ROSCORE_PID