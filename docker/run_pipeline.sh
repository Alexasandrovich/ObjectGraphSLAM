#!/bin/bash

# Function to kill background processes on exit
cleanup() {
    echo "[INFO] Stopping background processes..."
    kill $(jobs -p) 2>/dev/null
    exit
}
trap cleanup SIGINT SIGTERM

echo "[INFO] Setting up environment..."
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# CRITICAL: Add paths to external libraries manually
export ROS_PACKAGE_PATH=/root/external/amrl_msgs:/root/external/ORB_SLAM2/Examples/ROS/ORB_SLAM2:$ROS_PACKAGE_PATH

# Refresh ROS package index
rospack profile > /dev/null

echo "------------------------------------------------"
echo "[1/4] Starting ROS Core..."
# Redirect roscore logs to file to keep console clean
roscore > /root/roscore.log 2>&1 &
ROSCORE_PID=$!
sleep 3

echo "[2/4] Starting YOLOv5 Object Detector..."
cd /root/catkin_ws/src/yolov5

# Use python3 -u for unbuffered output so logs appear immediately
# Add [YOLO] prefix to logs for readability
python3 -u detect_ros.py --weights yolov5m.pt --img 640 --conf 0.5 2>&1 | sed "s/^/[YOLO] /" &
YOLO_PID=$!

echo "[INFO] Waiting 15s for YOLO model to load..."
sleep 15

echo "------------------------------------------------"
echo "[3/4] Running ObVi-SLAM Frontend..."
cd /root
# This process will block until the bag file is finished
rosrun obvislam_frontend run_orb_slam --config_file /root/data/run_kitti.json

echo "------------------------------------------------"
echo "[4/4] Running ObVi-SLAM Backend..."
rosrun obvislam_backend run_backend --config_file /root/data/run_kitti.json

echo "------------------------------------------------"
echo "[SUCCESS] Pipeline Finished! Results are in /root/data/ut_vslam_results"
echo "Logs info:"
echo "   - YOLO: (displayed above)"
echo "   - ROSCORE: /root/roscore.log"
echo "------------------------------------------------"
echo "[INFO] To visualize: open new terminal and run: rviz"
echo "[INFO] Press Ctrl+C to stop container."

# Wait indefinitely
wait $ROSCORE_PID