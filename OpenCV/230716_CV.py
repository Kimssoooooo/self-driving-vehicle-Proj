import cv2
import numpy as np
import picamera2

cap = cv2.VideoCapture(-1)

def ROI_img(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, [255, 255, 255])
    roi =cv2.vitwise_and(img, mask)
    return roi

def hough_line(img, rho, theta, threshold, min_line_len, max_line_gap):
    hough_line = v2.HoughLinesP(img, rho, theta, threshold, np.array([]), min_line_len, max_line_gap)
    
    background_img = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
    
    if(np.all(hough_lines) == True):
        draw_hough_linces(backgruond_img, hough_lines)
        
    return background_img

def draw_hough_lines(img, lines, color = [0, 0, 255], thickness =2):
    for line in lines:
        for x1, y1, x2, y2, in line:
            cv2.line(img, (x1,y1),(x2,y2),color,thickness)

def weighted_img(init_img, added_img):
    return cv2.addWeighted(init_img, 1.0, added_img, 1.0, 0.0)

while(cap.isOpened()):
        ret, frame = cap.read()
        if(ret):
            height, width = frame.shape[:2]
            
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img = cv2.GaussianBlur(img, (3,3), 0)
            img = cv2.Canny(img, 70, 210)
            
            vertices = np.array([[(0,height/2-50),(width,height/2-50),(width,height/2+50),(0,height/2+50)]], np.int32)
            img = ROI_img(img, vertices)
            
            
            hough_lines = hough_line(img,1,np.pi/180,50,50,30)
            
            merged_img = whighted_img(frame, hough_lines)
            cv2.imshow('merged_img', merged_img)
            
            if cv2.waitKey(1) & 0xFF == 27:
                break
            
cap.release()
cv2.destroyAllWindows()