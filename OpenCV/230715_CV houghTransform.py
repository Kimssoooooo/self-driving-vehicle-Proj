import cv2
from picamera2 import Picamera2
import numpy as np
from cv2 import imshow

#cap = cv2.VideoCapture(0)

picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1080)
picam2.preview_configuration.main.format="RGB888"
picam2.preview_configuration.align()
picam2.video_configuration.enable_raw(0)
picam2.start()

#적당한 수의 직선이 나타나야하기 때문에 CannyEdge 사용
gray = cv2.cvtColor(picam2, cv2.COLOR_BGR2GRAY)
frame = picam2.capture_array()
frame = cv2.flip(frame, -1)
#frame = cv2.resize(frame, dsize = (640, 360), interpolation = cv2.INTER_AREA)
cv2.imshow(' normal' , frame)
edges = cv2.Canny(gray, 300, 500, None, 3)
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
cv2.imshow('Edge', edges)

lines = cv2.HoughLines(edges, 1, np.pi/180, 170, srn = 100, stn = 200, min_theta = 0, max_theta =  np.pi)
#임계값에 따라 나타내는 직선의 수가 달라짐

for i in lines:
#for i in range(0, len(lines)):
    rho = lines[i][0][0]
    theta = lines[i][0][1]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int (x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    
    cv2.line(cap, (x1, y1), (x2, y2), (0, 0, 255), 1)

cv2.imshow('res', frame)
k = cv2.waitKey(0)

if k == 27:
    cv2.destroyAllWindows()