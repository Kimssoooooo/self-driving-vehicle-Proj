import cv2
from picamera2 import Picamera2
import numpy as np

#def main():
picam2 = Picamera2()
picam2.preview_configuration.main.size=(1920,1080)
picam2.preview_configuration.main.format="RGB888"
picam2.preview_configuration.align()
picam2.video_configuration.enable_raw(0)
picam2.start()

def ROI_img(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, [255, 255, 255])
    roi = cv2.bitwise_and(img, mask)
    return roi

def hough_line(img, rho, theta, threshold, min_line_len, max_line_gap):
    hough_lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), min_line_len, max_line_gap)
    background_img = np.zeros((img,shape[0], img.shape[1], 3), np.uint8)
    #if(np.all(hough_lines) === True):
    draw_hough_lines(background_img, hough_lines)
    return background_img

def draw_hough_lines(img, lines ,color = [0, 0, 255], thickness = 2):
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

while True:
    frame = picam2.capture_array()
    cv2.imshow("piCam", frame)
    if( picam2.is_open == True ):
        height, width = np.shape[:2]
        #frame = cv2.resize(frame, dsize = (640, 360), interpolation = cv2.INTER_AREA)
        #height = 360
        #width = 640
        
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        img = cv2.Canny(img, 70, 210)
        
        vertices = np.array([[(0, height/2-50), (width, height/2-50), (width, height/2+50), (0, height/2+50)]], np.int32)
        img = ROI_img(img, vertices)
        
        hough_lines = hough_line(img, 1, np.pi/180, 50, 50, 30)
        
        cv2.imshow('img', img)
        cv2.imshow('hough', hough_lines)
        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == 27:
            break

cv2.destroyAllWindows()