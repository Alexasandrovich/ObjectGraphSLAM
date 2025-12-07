import cv2
import rosbag
import rospy
import sys
import os
import time
from cv_bridge import CvBridge

def convert(video_path, bag_path, topic='/camera/image_raw', target_fps=30.0):
    if not os.path.exists(video_path):
        print(f"Error: Video not found at {video_path}")
        sys.exit(1)

    # Удаляем старый bag, чтобы не дописывать в конец
    if os.path.exists(bag_path):
        try:
            os.remove(bag_path)
        except OSError:
            pass

    print(f"Converting {video_path} -> {bag_path}...")

    cap = cv2.VideoCapture(video_path)
    bag = rosbag.Bag(bag_path, 'w')
    bridge = CvBridge()

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0 or fps > 1000:
        print(f"Warning: Cannot read FPS. Defaulting to {target_fps}")
        fps = target_fps

    print(f"Video FPS: {fps}")

    # Берем текущее время системы как точку старта.
    start_time = time.time()

    frame_id = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Считаем время: Старт + (смещение по кадрам)
        current_time = start_time + (float(frame_id) / fps)
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
    print(f"\nDone. Saved {frame_id} frames.")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 convert_video.py <input.mp4> <output.bag>")
        sys.exit(1)

    convert(sys.argv[1], sys.argv[2])