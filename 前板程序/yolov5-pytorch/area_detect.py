from random import random
from socket import *
import struct
import cv2
import numpy as np
import time
from PIL import Image
import pyrealsense2 as rs
import numpy as np
from socket import *
import cv2
import random
import torch
import time
from yolo import YOLO

def get_mid_pos(frame,result,depth_data):
    distance_list = []
    mid_pos = [(result[1] + result[3])//2, (result[2] + result[4])//2] #确定索引深度的中心像素位置
    min_val = min(abs(result[3] - result[1]), abs(result[4] - result[2])) #确定深度搜索范围
    dist = depth_data[int(mid_pos[0]), int(mid_pos[1])]
    
    print("curlen")   
    #for i in range(0,480):
    #	print(depth_data[i][320])
    

    #cv2.circle(frame, (int(mid_pos[0]), int(mid_pos[1])), 4, (255,0,0), -1)
    print(int(mid_pos[0] ), int(mid_pos[1]))
    # if dist:
    #     distance_list.append(dist)
    # distance_list = np.array(distance_list)
    # distance_list = np.sort(distance_list)[randnum//2-randnum//4:randnum//2+randnum//4] #冒泡排序+中值滤波
    #print(distance_list, np.mean(distance_list))
    return dist
if __name__ == "__main__":
    clientSocket = socket(AF_INET, SOCK_DGRAM)
    yolo = YOLO()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    # Start streaming
    pipeline.start(config)
    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            # Convert images to numpy arrays

            depth_image = np.asanyarray(depth_frame.get_data())

            color_image = np.asanyarray(color_frame.get_data())
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # Stack both images horizontally
            color_image=Image.fromarray(color_image)
            myres = yolo.detect_ourimage(color_image)

            if myres !=None:
                print(myres[0])
                dist=get_mid_pos(color_image, myres, depth_image)/1000
                print(dist)
                send_bytes = struct.pack('<f',float(dist))+bytes(myres[0],'utf-8')
                clientSocket.sendto(send_bytes, ("192.168.123.161", 12001))
            print("-----------------------")
            color_image=np.array(yolo.detect_image(color_image))
            images = np.hstack((color_image, depth_colormap))
            #Show images
            #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            #





#cv2.imshow('RealSense', color_image)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if 0xFF == ord('q') or key == 27:
                # cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        pipeline.stop()

