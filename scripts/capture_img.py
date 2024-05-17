###################################
# based on the official realsense python wrapper example script
###################################

import pyrealsense2 as rs
import numpy as np
import cv2
import shutil
import os
import subprocess
from time import sleep

LATEST_IMG_PATH = "/home/yara/camera_ws/src/visual_tools/docs/latest_imgs"
COMMAND_FILE_PATH = "/home/yara/camera_ws/src/visual_tools/docs/latest_imgs/command.txt"
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

def capture_img():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not aligned_depth_frame or not color_frame:
        print("ERROR: CAMERA IS NOT WORKING")
        exit(0)
    
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    cv2.imwrite(os.path.join(LATEST_IMG_PATH,"color.png"), color_image)
    cv2.imwrite(os.path.join(LATEST_IMG_PATH,"depth.png"), depth_image)
    
try:
   while (True):
    if (os.path.exists(COMMAND_FILE_PATH)):
        capture_img()
        os.remove(COMMAND_FILE_PATH)
        print("[CAPTURE IMG] DONE")
    else:   
        sleep(2)
except:
    pipeline.stop()