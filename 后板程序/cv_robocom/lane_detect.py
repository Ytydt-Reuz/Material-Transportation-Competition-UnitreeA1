import string
import numpy as np
import cv2
import pyrealsense2 as rs
from socket import *

serverPort = 12000
clientSocket = socket(AF_INET, SOCK_DGRAM)

# clientSocket.bind(('192.168.123.218', serverPort))"http://192.168.123.12:8080/?action=stream"
cap=cv2.VideoCapture(0)


while True:
    # # 读取摄像头采集到的图像，ret为True或False,代表有没有读取到图片。参数frame表示截取到一帧的图片
    # # ret, frame = video_capture.read()
    # cv2.namedWindow("input image", cv2.WINDOW_AUTOSIZE)
    # 读取摄像头采集到的图像，ret为True或False,代表有没有读取到图片。参数frame表示截取到一帧的图片
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)
    crop_img = frame
    

    # 改变色彩空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 高斯模糊(滤波)
    # blur = cv2.GaussianBlur(hsv, (15, 15), 0)  # 卷积核为：15*15
    # define range of yellow color in HSV
    # 15时可以识别到电线
    lower_yellow = np.array([0,0,50], np.uint8)
    upper_yellow = np.array([100,100,250],np.uint8)

    # lower_yellow = np.array([11,43,46])
    # upper_yellow = np.array([34,255,255])
    # Threshold the HSV image to get only yellow colors,二值化
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    res=cv2.bitwise_and(frame,frame,mask=mask)
    res=cv2.cvtColor(res,cv2.COLOR_HSV2BGR)

    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (15, 15), 0)
    # 图像二值化处理
    ret, thresh1 = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY)
    cv2.imshow("mask image", mask)

    # 膨胀和腐蚀，去除干扰
    mask = cv2.erode(thresh1, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cv2.imshow("mask image", mask)
    # 获取图像轮廓640-480
    contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
    # 找出最大轮廓，并找出其中心点坐标
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        # 求取中心点坐标（cx,cy）
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        print("cx: "+str(cx)+" "+"cy: "+str(cy))
        
        clientSocket.sendto(cx.to_bytes(4, byteorder="little", signed=True)+cy.to_bytes(4, byteorder="little", signed=True),("127.0.0.1",12000))

        cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
        cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
        # 绘制轮廓图
        cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
        
    else:
        print("I don't see the line")
    cv2.imshow('frame', crop_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.waitKey(0)
cv2.destroyAllWindows()
