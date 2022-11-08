from http import server
import cv2 as cv
import numpy as np
import pyrealsense2 as rs
from socket import *

########################################
#   通过realsense摄像头获取道路图像
########################################

# serverName = 'picture'
serverPort = 12000
senderSocket = socket(AF_INET, SOCK_DGRAM)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

while(1):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    img_encode = cv.imencode('.jpg', color_image)[1]
    data_encode = np.array(img_encode)
    data = data_encode.tostring()
    senderSocket.sendto(data, ('192.168.123.218', serverPort))


# cv.imwrite("color_pick.jpg", color_image)

# pipeline.stop()