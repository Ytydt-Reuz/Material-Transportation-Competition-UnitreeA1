import cv2
f = open("points.txt","a")

def mouseColor(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print('HSV:', hsv[y, x])
        # f.write("HSV: {}\n".format(hsv[y,x]))
        f.write(f"{hsv[y,x]}\n");


img = cv2.imread('color_pick.jpg')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
cv2.namedWindow("Color Picker")
cv2.setMouseCallback("Color Picker", mouseColor)
cv2.imshow("Color Picker", img)
if cv2.waitKey(0):
    cv2.destroyAllWindows()
    f.close()
