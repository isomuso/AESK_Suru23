import numpy as np
import cv2
import imutils
cap=cv2.VideoCapture(0)
#frame = cv2.imread('beforemasking".jpeg')
#image.set(3,1280)
#image.set(4,720)
#dim = (1080,720)
#frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
def nothing(x):
    pass
#IF YOU WANNA RED OBJECTS YOU MUST ADJUST RANGE LIKE 155-125-0 180-255-255
#cv2.namedWindow("HSV CONTROL")
"""
cv2.createTrackbar("L - H","HSV CONTROL", 0, 180, nothing)
cv2.createTrackbar("L - S","HSV CONTROL", 0, 255, nothing)
cv2.createTrackbar("L - V","HSV CONTROL", 0, 255, nothing)
cv2.createTrackbar("H - H","HSV CONTROL", 180, 180, nothing)
cv2.createTrackbar("H - S","HSV CONTROL", 255, 255, nothing)
cv2.createTrackbar("H - V","HSV CONTROL", 255, 255, nothing)

cv2.setTrackbarPos("L - H","HSV CONTROL",0)
cv2.setTrackbarPos("L - S","HSV CONTROL",0) #91
cv2.setTrackbarPos("L - V","HSV CONTROL",153) #142
cv2.setTrackbarPos("H - H","HSV CONTROL",180) #36
cv2.setTrackbarPos("H - S","HSV CONTROL",255)
cv2.setTrackbarPos("H - V","HSV CONTROL",255)
"""
t = 1

while True:
    t = t + 1
    #l_h = cv2.getTrackbarPos("L - H", "HSV CONTROL")
    #l_s = cv2.getTrackbarPos("L - S", "HSV CONTROL")
    #l_v = cv2.getTrackbarPos("L - V", "HSV CONTROL")
    #h_h = cv2.getTrackbarPos("H - H", "HSV CONTROL")
    #h_s = cv2.getTrackbarPos("H - S", "HSV CONTROL")
    #h_v = cv2.getTrackbarPos("H - V", "HSV CONTROL")
    l_h = 155
    l_s = 25
    l_v = 0
    h_h = 179
    h_s = 255
    h_v = 255
    low_color = np.array([l_h, l_s, l_v], np.uint8)
    high_color = np.array([h_h, h_s, h_v], np.uint8)

    _,frame=cap.read()
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,low_color,high_color)
    cns=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #while True:
     #   cv2.imshow("Fire Area", mask)
     #   k = cv2.waitKey(5)
     #   if k == 27:
     #       break
    cns=imutils.grab_contours(cns)
    for c in cns:
        area=cv2.contourArea(c)
        if area>10000:
            cv2.drawContours(frame,[c],-1,(0,255,0),3)
            M=cv2.moments(c)
            cx=int(M["m10"]/M["m00"])
            cy=int(M["m01"]/M["m00"])
            cv2.circle(frame,(cx,cy),7,(255,255,0),-1)
            cv2.putText(frame,"Fire Area",(cx-95,cy-20),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,140,255),3)
    if t == 2:
        break
"""
while True:
    cv2.imshow("Fire Area",frame)
    k=cv2.waitKey(5)
    if k== 27:
        break
#cap.release()
cv2.destroyAllWindows()
"""