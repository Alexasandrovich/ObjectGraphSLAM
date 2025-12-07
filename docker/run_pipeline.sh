#!/bin/bash

cleanup() {
    echo "[INFO] Stopping processes..."
    kill $(jobs -p) 2>/dev/null
    exit
}
trap cleanup SIGINT SIGTERM

echo "========================================"
echo "[INFO] DEV PIPELINE START"
echo "========================================"

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Настройка путей
PROJECT_DIR="/root/external/ut_vslam"
export ROS_PACKAGE_PATH=$PROJECT_DIR:/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:$ROS_PACKAGE_PATH
export LD_LIBRARY_PATH=$PROJECT_DIR/lib:$LD_LIBRARY_PATH

rospack profile > /dev/null

# ======================================================
# СБОРКА (КЭШИРОВАННАЯ)
# ======================================================
echo "[BUILD] Checking mounted source code..."
cd "$PROJECT_DIR"

# Проверка на всякий случай
if [ ! -d "src/shared" ]; then
    echo "❌ ERROR: src/shared is missing! Check your host folder."
    exit 1
fi

# Папка build теперь примонтирована с хоста.
# Если это первый запуск, она может быть пустой.
if [ ! -d "build" ]; then
    mkdir -p build
fi
cd build

# Если Makefile нет (первый запуск или очистили кэш) -> запускаем cmake
if [ ! -f "Makefile" ]; then
    echo "[BUILD] Makefile not found. Configuring CMake..."
    cmake .. -DROS_BUILD_TYPE=Release
else
    echo "[BUILD] Makefile found. Skipping CMake config."
fi

# Запускаем инкрементальную сборку
# make сам решит, что нужно пересобрать, исходя из дат файлов в src
echo "[BUILD] Compiling changes..."
if make -j$(nproc); then
    echo "✅ Build SUCCESS."
else
    echo "❌ Build FAILED."
    exit 1
fi

# ======================================================
# ЗАПУСК
# ======================================================

echo "[STEP 1] Starting ROS Core..."
roscore > /root/roscore.log 2>&1 &
ROSCORE_PID=$!
sleep 5

echo "[STEP 2] Starting YOLOv5..."
cd /root/catkin_ws/src/yolov5
python3 -u detect_ros.py --weights yolov5m.pt --img 640 --conf 0.5 2>&1 | sed "s/^/[YOLO] /" &
sleep 10

echo "[STEP 3] Running Frontend..."
rosrun ORB_SLAM2 Mono \
    /root/external/ORB_SLAM2/Vocabulary/ORBvoc.txt \
    /root/external/ORB_SLAM2/Examples/Monocular/KITTI00-02.yaml \
    /root/data/orb_out \
    > /root/frontend.log 2>&1 &
FRONTEND_PID=$!
sleep 5

echo "[STEP 4] Playing Data..."
# Проверяем наличие файла
BAG_FILE="/root/data/original_data/kitti.bag"
[ ! -f "$BAG_FILE" ] && BAG_FILE="/root/data/kitti.bag"

if [ -f "$BAG_FILE" ]; then
    rosbag play "$BAG_FILE" --clock
else
    echo "❌ ERROR: Bag file not found!"
fi

echo "[INFO] Bag finished. Stopping Frontend..."
sleep 2
kill $FRONTEND_PID 2>/dev/null

echo "[STEP 5] Running Backend..."
rosrun ut_vslam offline_object_visual_slam_main /root/data/run_kitti.json

echo "========================================"
echo "[SUCCESS] Finished!"
echo "========================================"
echo "[INFO] Open new terminal and run: rviz"

wait $ROSCORE_PID