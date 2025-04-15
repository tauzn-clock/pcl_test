import rosbag
import os
from sensor_msgs.msg import CameraInfo
import json
import numpy as np
import cv2
from cv_bridge import CvBridge

DESTINATION_DIR = "/scratchdata/processed/transition_grill"

BAG_DIR = "/scratchdata/transition_grill"

#Iterate through all the bag files in the directory
bag_files = []
for bag_file in os.listdir(BAG_DIR):
    bag_files.append(os.path.join(BAG_DIR, bag_file))
#Sort the bag files
bag_files.sort()
print(bag_files)

index = 0
rgb = []
depth = []

for bag_path in bag_files:
    # Open the rosbag file
    bag = rosbag.Bag(bag_path, 'r')

    # Camera Info
    camera_info = {}
    for topic, msg, t in bag.read_messages(topics=['/camera/color/camera_info']):
        camera_info["D"] = msg.D
        camera_info["K"] = msg.K
        camera_info["R"] = msg.R
        camera_info["P"] = msg.P
        camera_info["height"] = msg.height
        camera_info["width"] = msg.width
        break

    for topic, msg, t in bag.read_messages(topics=['/camera/color/image_raw']):
        rgb_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
        rgb.append((rgb_img, t))

    for topic, msg, t in bag.read_messages(topics=['/camera/depth/image_raw']):
        depth_img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        depth.append((depth_img, t))

    bag.close()

pt = 0
for i in range(len(rgb)):
    if pt >= len(depth):
        break
    closest = abs(rgb[i][1] - depth[pt][1])
    best_match = pt
    pt+=1
    while pt < len(depth):
        if abs(rgb[i][1] - depth[pt][1]) < closest:
            best_match = pt
            closest = abs(rgb[i][1] - depth[pt][1])
            pt += 1
        else:
            break
    
    rgb_img = rgb[i][0]
    depth_img = depth[best_match][0]

    # Save the RGB image
    cv2.imwrite(os.path.join(DESTINATION_DIR, "rgb", f'{index}.png'), rgb_img)
    # Save the depth image
    depth_img = depth_img.astype(np.uint16)
    cv2.imwrite(os.path.join(DESTINATION_DIR, "depth", f'{index}.png'), depth_img)   
    
    index += 1

    pt = best_match + 1 

# Save the camera info as a JSON file
with open(os.path.join(DESTINATION_DIR,'camera_info.json'), 'w') as json_file:
    json.dump(camera_info, json_file, indent=4)

print("Camera info has been saved to 'camera_info.json'.")