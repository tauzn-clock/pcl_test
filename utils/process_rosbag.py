import rosbag
import os
from sensor_msgs.msg import CameraInfo
import json
import numpy as np
import cv2
from cv_bridge import CvBridge

DESTINATION_DIR = "/scratchdata/processed/corridor"

# Open the rosbag file
bag = rosbag.Bag('/scratchdata/corridor.bag', 'r')

# Camera Info
camera_info = {}
for topic, msg, t in bag.read_messages(topics=['/camera/color/camera_info']):
    print(msg)
    camera_info["D"] = msg.D
    camera_info["K"] = msg.K
    camera_info["R"] = msg.R
    camera_info["P"] = msg.P
    camera_info["height"] = msg.height
    camera_info["width"] = msg.width
    break

rgb = []
depth = []

for topic, msg, t in bag.read_messages(topics=['/camera/color/image_raw']):
    rgb_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
    rgb.append((rgb_img, t))

for topic, msg, t in bag.read_messages(topics=['/camera/depth/image_raw']):
    depth_img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    depth.append((depth_img, t))

print(len(rgb))
print(len(depth))

for i in range(len(rgb)):
    cv2.imwrite(os.path.join(DESTINATION_DIR, f'rgb/{i}.png'), rgb[i][0])

for i in range(len(depth)):
    cv2.imwrite(os.path.join(DESTINATION_DIR, f'depth/{i}.png'), depth[i][0])

# Close the ROS bag
bag.close()

# Save the camera info as a JSON file
with open(os.path.join(DESTINATION_DIR,'camera_info.json'), 'w') as json_file:
    json.dump(camera_info, json_file, indent=4)

print("Camera info has been saved to 'camera_info.json'.")