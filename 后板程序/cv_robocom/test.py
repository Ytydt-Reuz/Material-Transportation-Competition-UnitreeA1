import cv2
#url = 'rtsp://admin:123456@192.168.1.158:8554/11'
#cap = cv2.VideoCapture(url)
cap=cv2.VideoCapture("http://192.168.123.12:8080/?action=stream")
print(cap.isOpened())
while (cap.isOpened()):
    ret, frame = cap.read()
    cv2.imshow("video", frame)
    if cv2.waitKey(1)&0xFF==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
