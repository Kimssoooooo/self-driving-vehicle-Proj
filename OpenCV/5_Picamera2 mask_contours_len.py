import cv2
from picamera2 import Picamera2
import numpy as np

def main():
    picam2 = Picamera2()
    picam2.preview_configuration.main.size=(1920, 1080)
    picam2.preview_configuration.main.format="RGB888"
    picam2.preview_configuration.align()
    picam2.video_configuration.enable_raw(0)
    picam2.start()

    while( picam2.is_open == True ):
        frame = picam2.capture_array()
        frame = cv2.flip(frame, -1)
        frame = cv2.resize(frame, dsize = (800, 450), interpolation = cv2.INTER_AREA)
        cv2.imshow(' normal' , frame)
        
        crop_img = frame[200:450, 0:800]
        #cv2.imshow(' crop_img' , crop_img)
        
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        #cv2.imshow(' gray' , gray)
        
        blur = cv2.GaussianBlur(gray, (5, 5), 170)
        #cv2.imshow(' blur' , blur)
        
        ret, thresh1 = cv2.threshold(blur, 150,255, cv2.THRESH_BINARY_INV)
        #cv2.imshow(' BINARY' , thresh1)
        
        mask = cv2.erode(thresh1, None, iterations = 2)
        
        mask = cv2.dilate(mask, None, iterations = 2)
        
        cv2.imshow('mask', mask)
        
        contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
        
        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)
            
            cx = int(M ['m10'] / M ['m00'])
            cy = int(M ['m01'] / M ['m00'])
            
            cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
            cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
            
            cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
            print("central moments : ", cx)
            
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()

