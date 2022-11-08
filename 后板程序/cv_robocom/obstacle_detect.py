import numpy as np
import cv2

video_capture = cv2.VideoCapture("http://192.168.123.12:8080/?action=stream")
# video_capture.set(3, 300)
# video_capture.set(4, 300)


while 1:
    # 读取摄像头采集到的图像，ret为True或False,代表有没有读取到图片。参数frame表示截取到一帧的图片
    ret, frame = video_capture.read()
    cv2.namedWindow("input image", cv2.WINDOW_AUTOSIZE)
    crop_img = frame
    # 图像灰度化处理
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
    # 高斯模糊（滤波）
    blur = cv2.GaussianBlur(gray, (15, 15), 0)  # 卷积核为：15*15
    # 图像二值化处理
    ret, thresh1 = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
    # 膨胀和腐蚀，去除干扰
    mask = cv2.erode(thresh1, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cv2.imshow("mask image", mask)
    # 获取图像轮廓
    contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
    # 找出最大轮廓，并找出其中心点坐标
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        # 求取中心点坐标（cx,cy）
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
        cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
        # 绘制轮廓图
        cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
        
        # # PID控制
        # Error = Target_value - cx   # 计算偏差
        # total_Err = total_Err + Error    # 偏差累加
        # output = Kp * Error + Ki * total_Err + Kd * (Error - last_Err)   # PID运算
        # last_Err = Error   # 将本次偏差赋给上次一偏差
        # u = output   # cx的调整值直接传给速度（调整速度）
        # # print(cx)
        # # print(u)
        # u_l = 70 + 1.2 * u   # 根据偏差调整左轮的转速
        # u_r = 70 - 1.2 * u   # 根据偏差调整右轮的转速
        # # 假如左右轮的速度理论值大于等于95，则令其等于95（原因：速度最大为100，以及避免电机长时间满负荷工作）
        # if u_l >= 95:
        #     u_l = 95
        # if u_r >= 95:
        #     u_r = 95
        # # 假如左右轮的速度理论值小于等于0，则令其等于0（原因：速度最小为0）
        # if u_r<=0:
        #     u_r=0
        # if u_l<=0:
        #     u_l=0
        # t_up(u_l, u_r, 0)
    else:
        print("I don't see the line")
    cv2.imshow('frame', crop_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # L_Motor.stop()
        # R_Motor.stop()
        # GPIO.cleanup()
        break
cv2.waitKey(0)
cv2.destroyAllWindows()
