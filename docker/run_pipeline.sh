#!/bin/bash

cleanup() {
    echo "[INFO] Stopping processes..."
    kill $(jobs -p) 2>/dev/null
    exit
}
trap cleanup SIGINT SIGTERM

echo "========================================"
echo "[INFO] PIPELINE START"
echo "========================================"

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

PROJECT_DIR="/root/external/ut_vslam"
DATA_DIR="/root/data"
CALIB_DIR="$DATA_DIR/calibration"
ORB_OUT_DIR="$DATA_DIR/orb_out"
RESULTS_DIR="$DATA_DIR/ut_vslam_results"
mkdir -p "$ORB_OUT_DIR" "$RESULTS_DIR" "$CALIB_DIR" "$RESULTS_DIR/logs"

# 1. НАСТРОЙКА ПУТЕЙ
export ROS_PACKAGE_PATH=$PROJECT_DIR:/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:$ROS_PACKAGE_PATH
export LD_LIBRARY_PATH=$PROJECT_DIR/lib:$LD_LIBRARY_PATH
rospack profile > /dev/null

# 2. СБОРКА
echo "[BUILD] Checking C++ build..."
cd "$PROJECT_DIR"
if [ ! -d "build" ]; then mkdir build; fi
cd build
if [ ! -f "Makefile" ]; then cmake .. -DROS_BUILD_TYPE=Release; fi

if ! make -j$(nproc); then
    echo "❌ Build FAILED."
    exit 1
fi
echo "Build SUCCESS."

# 3. ПОДГОТОВКА ДАННЫХ
echo "[DATA] Preparing data..."

# 3. Конвертация видео
python3 /root/scripts/convert_video.py \
    "$DATA_DIR/raw/video.mp4" \
    "$DATA_DIR/my_video.bag"

# ======================================================
# 4. ЗАПУСК ROS
# ======================================================

echo "[STEP 1] Starting ROS Core..."
roscore > /root/roscore.log 2>&1 &
ROSCORE_PID=$!
sleep 5

echo "[STEP 2] Starting YOLOv5..."
cd /root/catkin_ws/src/yolov5
python3 -u detect_ros.py --weights yolov5m.pt --img 640 --conf 0.5 2>&1 | sed "s/^/[YOLO] /" &
sleep 5

echo "------------------------------------------------"
echo "[STEP 3] Starting Frontend..."
rosrun ORB_SLAM2 Mono \
    /root/external/ORB_SLAM2/Vocabulary/ORBvoc.txt \
    "$CALIB_DIR/custom.yaml" \
    "$ORB_OUT_DIR" \
    > /root/frontend.log 2>&1 &
FRONTEND_PID=$!
sleep 5

echo "------------------------------------------------"
echo "[STEP 4] Playing Data..."
BAG_PATH="$DATA_DIR/my_video.bag"

rosbag play "$BAG_PATH" --clock -r 1.0 -d 2

echo "[INFO] Bag finished. Stopping Frontend..."
kill -SIGINT $FRONTEND_PID 2>/dev/null
sleep 5

# Проверка результата Frontend
if [ -f "$ORB_OUT_DIR/KeyFrameTrajectory.txt" ]; then
    cp "$ORB_OUT_DIR/KeyFrameTrajectory.txt" "$ORB_OUT_DIR/CameraTrajectory.txt"
fi

if [ ! -s "$ORB_OUT_DIR/CameraTrajectory.txt" ]; then
    echo "FAILURE: Trajectory not created. Check frontend.log"
    echo "Most likely reasons:"
    echo "1. Video frame rate/resolution mismatch in custom.yaml"
    echo "2. ORB-SLAM2 initialization failed (needs texture/movement)"
    exit 1
fi

echo "------------------------------------------------"
echo "[STEP 5] Running Backend..."

rosrun ut_vslam offline_object_visual_slam_main \
    --intrinsics_file="$CALIB_DIR/custom_camera.txt" \
    --extrinsics_file="$CALIB_DIR/extrinsics.txt" \
    --rosbag_file="$BAG_PATH" \
    --poses_by_node_id_file="$ORB_OUT_DIR/CameraTrajectory.txt" \
    --nodes_by_timestamp_file="$ORB_OUT_DIR/NodeIdAndTimestamp.csv" \
    --low_level_feats_dir="$ORB_OUT_DIR/feats" \
    --long_term_map_output="$RESULTS_DIR/long_term_map.xml" \
    --robot_poses_results_file="$RESULTS_DIR/robot_poses.txt" \
    --ellipsoids_results_file="$RESULTS_DIR/ellipsoids.txt" \
    --params_config_file="/root/data/running_config.json" \
    --logs_directory="$RESULTS_DIR/logs"

echo "========================================"
echo "[SUCCESS] Finished!"
echo "========================================"
echo "[INFO] Open new terminal and run: rviz"

wait $ROSCORE_PID