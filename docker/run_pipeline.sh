#!/bin/bash

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
if [ -f "/root/catkin_ws/devel/setup.bash" ]; then
    source /root/catkin_ws/devel/setup.bash
fi

# 1. rosbuild требует, чтобы имя папки совпадало с именем пакета (ut_vslam)
if [ -d "/root/external/ObVi-SLAM" ]; then
    mv /root/external/ObVi-SLAM /root/external/ut_vslam
fi

# 2. ЭКСПОРТ ПУТЕЙ (Теперь путь ведет к ut_vslam)
export ROS_PACKAGE_PATH=/root/external/ut_vslam:/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:/root/catkin_ws/src:$ROS_PACKAGE_PATH

# Обновляем базу пакетов ROS
rospack profile > /dev/null

# 3. ПОДГОТОВКА СЛОВАРЯ (Frontend)
if [ ! -f "/root/external/ORB_SLAM2/Vocabulary/ORBvoc.txt" ]; then
    echo "[INFO] Unzipping ORB Vocabulary..."
    cd /root/external/ORB_SLAM2/Vocabulary
    tar -xf ORBvoc.txt.tar.gz
fi

# ======================================================
# ЗАПУСК ПРОЦЕССОВ
# ======================================================

echo "------------------------------------------------"
echo "[STEP 1/5] Starting ROS Core..."
roscore > /root/roscore.log 2>&1 &
ROSCORE_PID=$!
sleep 5

echo "[STEP 2/5] Starting YOLOv5..."
cd /root/catkin_ws/src/yolov5
python3 -u detect_ros.py --weights yolov5m.pt --img 640 --conf 0.5 2>&1 | sed "s/^/[YOLO] /" &
sleep 15

echo "------------------------------------------------"
echo "[STEP 3/5] Starting Frontend (ORB-SLAM2 Mono)..."
# Мы используем стандартный конфиг KITTI00-02.yaml, который уже есть в репозитории
# Аргументы: VOCABULARY  SETTINGS_FILE  OUTPUT_FOLDER
rosrun ORB_SLAM2 Mono \
    /root/external/ORB_SLAM2/Vocabulary/ORBvoc.txt \
    /root/external/ORB_SLAM2/Examples/Monocular/KITTI00-02.yaml \
    /root/data/orb_out \
    > /root/frontend.log 2>&1 &

FRONTEND_PID=$!
echo "[INFO] Frontend started in background (PID: $FRONTEND_PID). Waiting for ready..."
sleep 5

echo "------------------------------------------------"
echo "[STEP 4/5] Playing ROS Bag (Input Data)..."
# Запускаем воспроизведение видео. Frontend будет обрабатывать его в реальном времени.
rosbag play /root/data/original_data/kitti.bag --clock

echo "[INFO] Bag finished. Stopping Frontend..."
# Даем время на сохранение результатов
sleep 5
kill $FRONTEND_PID 2>/dev/null

echo "------------------------------------------------"
echo "[STEP 5/5] Running Backend (ut_vslam)..."
# Запускаем построение карты
rosrun ut_vslam offline_object_visual_slam_main /root/data/run_kitti.json

echo "------------------------------------------------"
echo "[SUCCESS] Finished! Check /root/data/ut_vslam_results"
echo "------------------------------------------------"
echo "[INFO] To visualize: open new terminal and run: rviz"
echo "[INFO] Press Ctrl+C to stop container."

wait $ROSCORE_PID