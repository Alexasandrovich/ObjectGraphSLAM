import cv2
import rosbag
import rospy
import sys
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def convert(video_path, bag_path, topic='/camera/image_raw', target_fps=30.0):
    if not os.path.exists(video_path):
        print(f"Error: Video not found at {video_path}")
        sys.exit(1)

    # Удаляем старый bag, если есть, чтобы перезаписать корректно
    if os.path.exists(bag_path):
        os.remove(bag_path)

    print(f"Converting {video_path} -> {bag_path}...")

    cap = cv2.VideoCapture(video_path)
    bag = rosbag.Bag(bag_path, 'w')
    bridge = CvBridge()

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0 or fps > 1000:
        fps = target_fps

    print(f"Video FPS: {fps}")

    # [FIX] Начинаем не с 0.0, а с фиксированного времени (например, 1000.0 секунд),
    # чтобы rosbag не ругался на invalid time.
    start_time_sec = 1000.0

    frame_id = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Считаем время: старт + (номер кадра / fps)
        current_time = float(start_time_sec) + (float(frame_id) / fps)
        timestamp = rospy.Time.from_sec(current_time)

        msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = timestamp
        msg.header.frame_id = 'camera'

        bag.write(topic, msg, timestamp)

        frame_id += 1
        if frame_id % 50 == 0:
            print(f"Processed {frame_id} frames...", end='\r')

    cap.release()
    bag.close()
    print(f"\nDone. Saved {frame_id} frames to {bag_path}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 convert_video.py <input.mp4> <output.bag>")
        sys.exit(1)

    convert(sys.argv[1], sys.argv[2])