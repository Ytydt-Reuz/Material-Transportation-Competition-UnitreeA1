import string
import numpy as np
#import pyrealsense2 as rs
from socket import *
import time
import cv2
from PIL import Image

from yolo import YOLO
serverPort = 12000
clientSocket = socket(AF_INET, SOCK_DGRAM)

# clientSocket.bind(('192.168.123.218', serverPort))"http://192.168.123.12:8080/?action=stream"
cap=cv2.VideoCapture(0)
yolo = YOLO()

while True:
    # # 读取摄像头采集到的图像，ret为True或False,代表有没有读取到图片。参数frame表示截取到一帧的图片
    # # ret, frame = video_capture.read()
    # cv2.namedWindow("input image", cv2.WINDOW_AUTOSIZE)
    # 读取摄像头采集到的图像，ret为True或False,代表有没有读取到图片。参数frame表示截取到一帧的图片
    t1 = time.time()
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)
    crop_img = frame
    if not ret:
        break
    # 格式转变，BGRtoRGB
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # 转变成Image
    frame = Image.fromarray(np.uint8(frame))
    # 进行检测
    frame = np.array(yolo.detect_image(frame))

    # RGBtoBGR满足opencv显示格式
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    fps = (fps + (1. / (time.time() - t1))) / 2
    print("fps= %.2f" % (fps))
    frame = cv2.putText(frame, "fps= %.2f" % (fps), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("video", frame)
    c = cv2.waitKey(1) & 0xff
    if c == 27:
        cap.release()
        break

    print("Video Detection Done!")
    clientSocket.sendto(frame.to_bytes(4, byteorder="little", signed=True),("127.0.0.1",12000))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.waitKey(0)
cv2.destroyAllWindows()
